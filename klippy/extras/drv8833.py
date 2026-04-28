# Support for DRV8833 based CANVAS-style motor control
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import math
import os

from . import pulse_counter

HALL_POLL_TIME = 0.001
CONTROL_INTERVAL = 0.010
MCU_REPORT_INTERVAL = 0.050
DEBUG_LOG_INTERVAL = 0.500
COUNTER_SAMPLE_TIME = 0.050
COUNTER_POLL_TIME = 0.001
DEFAULT_DUTY = 50.0
MAX_DUTY_STEP = 4.0
DEFAULT_PID_KP = 1.2
DEFAULT_PID_KI = 0.8
DEFAULT_PID_KD = 0.02
HALL_RESOLUTION_SCALE = 1000000
SPEED_SCALE = 1000
DUTY_SCALE = 10
PID_PARAM_SCALE = 1000
AUTOTUNE_SETTLE_TIME = 0.8
AUTOTUNE_SAMPLE_TIME = 0.4
AUTOTUNE_BAND_RATIO = 0.05
AUTOTUNE_MIN_BAND = 1.0
AUTOTUNE_RELAY_RATIO = 0.15
AUTOTUNE_MIN_RELAY_DUTY = 4.0
AUTOTUNE_MAX_RELAY_DUTY = 20.0
AUTOTUNE_BASELINE_STEPS = 6
AUTOTUNE_REQUIRED_PAIRS = 3
AUTOTUNE_TIMEOUT = 45.0


def _clamp(value, min_value, max_value):
    return max(min_value, min(max_value, value))


class PulseTracker:
    def __init__(self, printer, pin, resolution, sample_time=COUNTER_SAMPLE_TIME):
        self.resolution = resolution
        self.speed_mm_s = 0.0
        self.total_count = 0
        self.last_edge_time = None
        self.last_interval = 0.0
        self.counter = pulse_counter.MCU_counter(
            printer, pin, sample_time, COUNTER_POLL_TIME
        )
        self.mcu = self.counter._mcu
        self.counter.setup_callback(self._handle_counter)

    def _handle_counter(self, sample_time, count, count_time):
        delta_count = count - self.total_count
        self.total_count = count
        if delta_count <= 0:
            return
        if self.last_edge_time is not None:
            delta_time = count_time - self.last_edge_time
            if delta_time > 0.0:
                self.last_interval = delta_time / float(delta_count)
                self.speed_mm_s = (
                    float(delta_count) * self.resolution / delta_time
                )
        self.last_edge_time = count_time

    def get_count(self):
        return self.total_count

    def get_speed(self, eventtime):
        if self.last_edge_time is None:
            return 0.0
        stale_time = max(0.250, self.last_interval * 4.0)
        current_print_time = self.mcu.estimated_print_time(eventtime)
        if current_print_time - self.last_edge_time > stale_time:
            return 0.0
        return self.speed_mm_s


class MCUDrv8833Controller:
    def __init__(self, config, motor_fwd_params, motor_rwd_params, hall_params):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.mcu = motor_fwd_params["chip"]
        self.oid = self.mcu.create_oid()
        self.motor_fwd_pin = motor_fwd_params["pin"]
        self.motor_rwd_pin = motor_rwd_params["pin"]
        self.hall_pin = hall_params["pin"]
        self.hall_pullup = hall_params["pullup"]
        self.cycle_time = config.getfloat("motor_cycle_time", 0.002, above=0.0)
        self.hall_resolution = config.getfloat(
            "motor_hall_resolution", above=0.0
        )
        self.default_duty = config.getfloat(
            "default_duty", DEFAULT_DUTY, minval=0.0, maxval=100.0
        )
        self.max_duty_step = config.getfloat(
            "max_duty_step", MAX_DUTY_STEP, above=0.0, maxval=100.0
        )
        self.pid_kp = config.getfloat("pid_kp", DEFAULT_PID_KP, minval=0.0)
        self.pid_ki = config.getfloat("pid_ki", DEFAULT_PID_KI, minval=0.0)
        self.pid_kd = config.getfloat("pid_kd", DEFAULT_PID_KD, minval=0.0)
        self.last_active = False
        self.last_manual = False
        self.last_count = 0
        self.last_speed_mm_s = 0.0
        self.last_duty_cycle = 0.0
        self.last_status_time = 0.0
        self.status_sequence = 0
        self.status_callback = lambda: None
        self._set_cmd = None
        self._manual_cmd = None
        self._set_pid_cmd = None
        self.mcu.register_response(
            self._handle_status, "drv8833_status", self.oid
        )
        self.mcu.register_config_callback(self._build_config)

    def _pid_to_mcu(self, value):
        return int(round(value * PID_PARAM_SCALE * DUTY_SCALE))

    def _build_config(self):
        pwm_max = int(self.mcu.get_constant_float("PWM_MAX") + 0.5)
        cycle_ticks = self.mcu.seconds_to_clock(self.cycle_time)
        poll_ticks = self.mcu.seconds_to_clock(HALL_POLL_TIME)
        control_ticks = self.mcu.seconds_to_clock(CONTROL_INTERVAL)
        report_ticks = self.mcu.seconds_to_clock(MCU_REPORT_INTERVAL)
        hall_resolution = int(round(self.hall_resolution * HALL_RESOLUTION_SCALE))
        default_duty = int(round(self.default_duty * DUTY_SCALE))
        max_duty_step = int(round(self.max_duty_step * DUTY_SCALE))
        pid_kp = self._pid_to_mcu(self.pid_kp)
        pid_ki = self._pid_to_mcu(self.pid_ki)
        pid_kd = self._pid_to_mcu(self.pid_kd)
        self.mcu.add_config_cmd(
            "config_drv8833 oid=%d motor_fwd_pin=%s motor_rwd_pin=%s "
            "cycle_ticks=%d hall_pin=%s hall_pullup=%d poll_ticks=%d "
            "control_ticks=%d report_ticks=%d hall_resolution=%d "
            "pwm_max=%d default_duty=%d max_duty_step=%d"
            % (
                self.oid,
                self.motor_fwd_pin,
                self.motor_rwd_pin,
                cycle_ticks,
                self.hall_pin,
                self.hall_pullup,
                poll_ticks,
                control_ticks,
                report_ticks,
                hall_resolution,
                pwm_max,
                default_duty,
                max_duty_step,
            )
        )
        self.mcu.add_config_cmd(
            "drv8833_set_pid oid=%d kp=%d ki=%d kd=%d"
            % (self.oid, pid_kp, pid_ki, pid_kd),
            is_init=True,
        )
        self.mcu.add_config_cmd(
            "drv8833_set oid=%d enable=0 direction=1 target_speed=0" % (self.oid,),
            on_restart=True,
        )
        self._set_cmd = self.mcu.lookup_command(
            "drv8833_set oid=%c enable=%c direction=%c target_speed=%u"
        )
        self._manual_cmd = self.mcu.lookup_command(
            "drv8833_manual oid=%c enable=%c direction=%c duty=%hu"
        )
        self._set_pid_cmd = self.mcu.lookup_command(
            "drv8833_set_pid oid=%c kp=%u ki=%u kd=%u"
        )

    def _handle_status(self, params):
        self.last_active = bool(params["active"])
        self.last_manual = bool(params["manual"])
        self.last_count = params["count"]
        self.last_speed_mm_s = params["speed"] / float(SPEED_SCALE)
        self.last_duty_cycle = params["duty"] / float(DUTY_SCALE)
        self.last_status_time = self.reactor.monotonic()
        self.status_sequence += 1
        self.status_callback()

    def start(self, direction, target_speed):
        set_cmd = self._set_cmd
        if set_cmd is None:
            raise self.printer.command_error("drv8833 controller not configured")
        set_cmd.send(
            [
                self.oid,
                1,
                1 if direction > 0 else 0,
                int(round(target_speed * SPEED_SCALE)),
            ]
        )

    def manual_start(self, direction, duty_cycle):
        manual_cmd = self._manual_cmd
        if manual_cmd is None:
            raise self.printer.command_error("drv8833 controller not configured")
        manual_cmd.send(
            [
                self.oid,
                1,
                1 if direction > 0 else 0,
                int(round(_clamp(duty_cycle, 0.0, 100.0) * DUTY_SCALE)),
            ]
        )

    def set_pid(self, kp, ki, kd):
        set_pid_cmd = self._set_pid_cmd
        if set_pid_cmd is None:
            raise self.printer.command_error("drv8833 controller not configured")
        self.pid_kp = kp
        self.pid_ki = ki
        self.pid_kd = kd
        set_pid_cmd.send(
            [
                self.oid,
                self._pid_to_mcu(kp),
                self._pid_to_mcu(ki),
                self._pid_to_mcu(kd),
            ]
        )

    def stop(self):
        set_cmd = self._set_cmd
        if set_cmd is None:
            raise self.printer.command_error("drv8833 controller not configured")
        set_cmd.send([self.oid, 0, 1, 0])


class Drv8833PIDAutoTune:
    def __init__(self, lane, gcmd):
        self.lane = lane
        self.gcmd = gcmd
        self.reactor = lane.reactor
        self.controller = lane.hall_controller
        self.target_speed = gcmd.get_float("SPEED", 40.0, above=0.0)
        direction = gcmd.get("DIRECTION", "forwards").strip().lower()
        if direction not in ("forwards", "forward", "backwards", "backward"):
            raise gcmd.error("DIRECTION must be forwards or backwards")
        self.direction = 1 if direction.startswith("for") else -1
        self.band = gcmd.get_float(
            "BAND",
            max(AUTOTUNE_MIN_BAND, self.target_speed * AUTOTUNE_BAND_RATIO),
            above=0.0,
        )
        self.settle_time = gcmd.get_float(
            "SETTLE_TIME", AUTOTUNE_SETTLE_TIME, above=0.0
        )
        self.sample_time = gcmd.get_float(
            "SAMPLE_TIME", AUTOTUNE_SAMPLE_TIME, above=0.0
        )
        self.timeout = gcmd.get_float("TIMEOUT", AUTOTUNE_TIMEOUT, above=0.0)
        self.write_file = gcmd.get_int("WRITE_FILE", 0)
        self.eventtime = self.reactor.monotonic()
        self.data = []
        self.peaks = []
        self.switch_times = []

    def _pause(self, duration):
        self.eventtime = self.reactor.pause(self.eventtime + duration)
        return self.eventtime

    def _wait_for_status(self, timeout=1.0):
        start_seq = self.controller.status_sequence
        deadline = self.eventtime + timeout
        while self.controller.status_sequence == start_seq:
            self.eventtime = self.reactor.pause(
                min(deadline, self.eventtime + MCU_REPORT_INTERVAL)
            )
            if self.eventtime >= deadline and self.controller.status_sequence == start_seq:
                raise self.gcmd.error("Timed out waiting for drv8833 status update")
        sample = (
            self.controller.last_status_time or self.eventtime,
            self.controller.last_speed_mm_s,
            self.controller.last_duty_cycle,
        )
        self.data.append(sample + (self.target_speed,))
        return sample

    def _measure_speed(self, duty_cycle):
        self.lane._start_manual(self.eventtime, self.direction, duty_cycle)
        self._pause(self.settle_time)
        endtime = self.eventtime + self.sample_time
        samples = []
        while self.eventtime < endtime:
            _, speed, _ = self._wait_for_status()
            samples.append(speed)
        if not samples:
            samples.append(self.controller.last_speed_mm_s)
        avg_speed = sum(samples) / float(len(samples))
        self.gcmd.respond_info(
            "drv8833 %s tune sample: duty=%.2f%% speed=%.3fmm/s"
            % (self.lane.name, duty_cycle, avg_speed)
        )
        return avg_speed

    def _find_baseline_duty(self):
        max_speed = self._measure_speed(100.0)
        if max_speed < self.target_speed:
            raise self.gcmd.error(
                "Target speed %.3fmm/s is not reachable; max observed %.3fmm/s"
                % (self.target_speed, max_speed)
            )
        low = 0.0
        high = 100.0
        baseline = self.controller.default_duty
        for _ in range(AUTOTUNE_BASELINE_STEPS):
            baseline = 0.5 * (low + high)
            speed = self._measure_speed(baseline)
            if speed < self.target_speed:
                low = baseline
            else:
                high = baseline
        return 0.5 * (low + high)

    def _check_peak(self, sample_time, speed, target):
        peak_time, peak_speed, peak_kind = self.peaks[-1] if self.peaks else (0.0, target, None)
        if not self.peaks:
            peak_speed = target
            peak_time = sample_time
            peak_kind = None
        if speed > target:
            if peak_kind != "high" or speed > peak_speed:
                peak_time, peak_speed, peak_kind = sample_time, speed, "high"
        elif speed < target:
            if peak_kind != "low" or speed < peak_speed:
                peak_time, peak_speed, peak_kind = sample_time, speed, "low"
        if not self.peaks or (peak_time, peak_speed, peak_kind) != self.peaks[-1]:
            if self.peaks and self.peaks[-1][2] == peak_kind:
                self.peaks[-1] = (peak_time, peak_speed, peak_kind)
            elif peak_kind is not None:
                self.peaks.append((peak_time, peak_speed, peak_kind))

    def _collect_relay_response(self, low_duty, high_duty):
        target_low = self.target_speed - self.band
        target_high = self.target_speed + self.band
        driving_high = True
        self.lane._start_manual(self.eventtime, self.direction, high_duty)
        self.switch_times = [(self.eventtime, high_duty)]
        deadline = self.eventtime + self.timeout
        while self.eventtime < deadline:
            sample_time, speed, _ = self._wait_for_status()
            self._check_peak(sample_time, speed, self.target_speed)
            if driving_high and speed >= target_high:
                driving_high = False
                self.controller.manual_start(self.direction, low_duty)
                self.switch_times.append((sample_time, low_duty))
            elif not driving_high and speed <= target_low:
                driving_high = True
                self.controller.manual_start(self.direction, high_duty)
                self.switch_times.append((sample_time, high_duty))
            pairs = self._get_peak_pairs()
            if len(pairs) >= AUTOTUNE_REQUIRED_PAIRS:
                return pairs
        raise self.gcmd.error("drv8833 PID tune timed out before converging")

    def _get_peak_pairs(self):
        pairs = []
        last_low = None
        for sample_time, speed, kind in self.peaks:
            if kind == "low":
                last_low = (sample_time, speed)
            elif kind == "high" and last_low is not None:
                pairs.append((last_low, (sample_time, speed)))
                last_low = None
        return pairs

    def _calc_pid(self, relay_duty, pairs):
        pairs = pairs[-AUTOTUNE_REQUIRED_PAIRS:]
        high_times = [high[0] for _, high in pairs]
        if len(high_times) < 2:
            raise self.gcmd.error("Not enough high peaks collected for PID tuning")
        amplitudes = [0.5 * (high[1] - low[1]) for low, high in pairs]
        amplitude = sum(amplitudes) / float(len(amplitudes))
        if amplitude <= 0.0:
            raise self.gcmd.error("Measured relay oscillation amplitude is invalid")
        periods = [
            high_times[index] - high_times[index - 1]
            for index in range(1, len(high_times))
        ]
        Tu = sum(periods) / float(len(periods))
        Ku = 4.0 * relay_duty / (math.pi * amplitude)
        Kp = 0.6 * Ku
        Ki = Kp / (0.5 * Tu)
        Kd = Kp * (0.125 * Tu)
        return Kp, Ki, Kd, Ku, Tu, amplitude

    def write_file_data(self, filename):
        with open(filename, "w") as out:
            out.write("time,speed,duty,target\n")
            out.write(
                "\n".join(
                    "%.5f,%.5f,%.5f,%.5f" % sample for sample in self.data
                )
            )

    def run(self):
        baseline = self._find_baseline_duty()
        relay_duty = self.gcmd.get_float(
            "RELAY_DUTY",
            _clamp(
                baseline * AUTOTUNE_RELAY_RATIO,
                AUTOTUNE_MIN_RELAY_DUTY,
                AUTOTUNE_MAX_RELAY_DUTY,
            ),
            above=0.0,
            maxval=50.0,
        )
        low_duty = _clamp(baseline - relay_duty, 0.0, 100.0)
        high_duty = _clamp(baseline + relay_duty, 0.0, 100.0)
        relay_duty = 0.5 * (high_duty - low_duty)
        if high_duty - low_duty < 1.0:
            raise self.gcmd.error("Relay duty window is too small to tune")
        self.gcmd.respond_info(
            "drv8833 %s tune baseline=%.2f%% relay_low=%.2f%% relay_high=%.2f%% band=%.3fmm/s"
            % (self.lane.name, baseline, low_duty, high_duty, self.band)
        )
        pairs = self._collect_relay_response(low_duty, high_duty)
        Kp, Ki, Kd, Ku, Tu, amplitude = self._calc_pid(relay_duty, pairs)
        if self.write_file:
            filename = os.path.join(
                "/tmp", "drv8833_%s_pid_tune.csv" % (self.lane.name,)
            )
            self.write_file_data(filename)
            self.gcmd.respond_info("drv8833 tune data written to %s" % (filename,))
        return {
            "pid_kp": Kp,
            "pid_ki": Ki,
            "pid_kd": Kd,
            "ultimate_gain": Ku,
            "ultimate_period": Tu,
            "amplitude": amplitude,
            "baseline_duty": baseline,
            "relay_duty": relay_duty,
        }


class Drv8833CommandHelper:
    cmd_MOVE_DEBUG_help = "Run a timed DRV8833 debug move"
    cmd_DRV8833_PID_TUNE_help = "Run relay autotune for DRV8833 PID gains"
    cmd_SET_DRV8833_PID_help = "Update DRV8833 PID gains"

    def __init__(self, printer):
        self.printer = printer
        self.gcode = printer.lookup_object("gcode")
        self.gcode.register_command(
            "MOVE_DEBUG", self.cmd_MOVE_DEBUG, desc=self.cmd_MOVE_DEBUG_help
        )
        self.gcode.register_command(
            "DRV_PID_TUNE",
            self.cmd_DRV8833_PID_TUNE,
            desc=self.cmd_DRV8833_PID_TUNE_help,
        )
        self.gcode.register_command(
            "DRV_SET_PID",
            self.cmd_SET_DRV8833_PID,
            desc=self.cmd_SET_DRV8833_PID_help,
        )

    def _lookup_lane(self, gcmd):
        lanes = dict(self.printer.lookup_objects("drv8833"))
        if not lanes:
            raise gcmd.error("No [drv8833] sections configured")
        name = gcmd.get("NAME", None)
        if name is not None:
            lane = lanes.get("drv8833 %s" % (name,))
            if lane is None:
                lane = lanes.get(name)
            if lane is None:
                raise gcmd.error("Unknown drv8833 NAME '%s'" % (name,))
            return lane
        if len(lanes) == 1:
            return list(lanes.values())[0]
        names = [section.split(" ", 1)[-1] for section in sorted(lanes)]
        raise gcmd.error(
            "MOVE_DEBUG requires NAME=<drv8833>; available: %s"
            % (", ".join(names),)
        )

    def cmd_MOVE_DEBUG(self, gcmd):
        self._lookup_lane(gcmd).cmd_MOVE_DEBUG(gcmd)

    def cmd_DRV8833_PID_TUNE(self, gcmd):
        self._lookup_lane(gcmd).cmd_DRV8833_PID_TUNE(gcmd)

    def cmd_SET_DRV8833_PID(self, gcmd):
        self._lookup_lane(gcmd).cmd_SET_DRV8833_PID(gcmd)


class PrinterDrv8833:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object("gcode")
        self.section_name = config.get_name()
        section_parts = self.section_name.split(None, 1)
        self.name = (
            section_parts[1] if len(section_parts) > 1 else self.section_name
        )
        ppins = self.printer.lookup_object("pins")
        motor_fwd_params = ppins.lookup_pin(config.get("motor_fwd"))
        motor_rwd_params = ppins.lookup_pin(config.get("motor_rwd"))
        hall_params = ppins.lookup_pin(config.get("motor_hall"), can_pullup=True)
        self.mcu = motor_fwd_params["chip"]
        if motor_rwd_params["chip"] is not self.mcu:
            raise config.error(
                "drv8833 motor_fwd and motor_rwd must be on the same mcu"
            )
        if hall_params["chip"] is not self.mcu:
            raise config.error(
                "drv8833 motor_hall, motor_fwd, and motor_rwd must be on the same mcu"
            )
        self.hall_controller = MCUDrv8833Controller(
            config, motor_fwd_params, motor_rwd_params, hall_params
        )
        self.hall_controller.status_callback = self._handle_hall_status
        self.odometer = PulseTracker(
            self.printer,
            config.get("odometer"),
            config.getfloat("odometer_resolution", above=0.0),
        )
        self.active = False
        self.manual_mode = False
        self.direction = 1
        self.direction_name = "forwards"
        self.target_speed = 0.0
        self.debug_gcmd = None
        self.debug_next_log_time = 0.0
        self.debug_hall_start = 0
        self.debug_odometer_start = 0
        cmd_helper = self.printer.lookup_object("drv8833_command_helper", None)
        if cmd_helper is None:
            cmd_helper = Drv8833CommandHelper(self.printer)
            self.printer.add_object("drv8833_command_helper", cmd_helper)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart
        )
        self.printer.register_event_handler("klippy:shutdown", self._handle_stop)

    def get_status(self, eventtime):
        return {
            "active": self.active,
            "manual": self.manual_mode,
            "direction": self.direction_name,
            "target_speed": self.target_speed,
            "duty_cycle": self.hall_controller.last_duty_cycle,
            "hall_speed": self.hall_controller.last_speed_mm_s,
            "odometer_speed": self.odometer.get_speed(eventtime),
            "pid_kp": self.hall_controller.pid_kp,
            "pid_ki": self.hall_controller.pid_ki,
            "pid_kd": self.hall_controller.pid_kd,
        }

    def _handle_request_restart(self, print_time):
        self._stop(self.reactor.monotonic())

    def _handle_stop(self):
        self._stop(self.reactor.monotonic())

    def _handle_hall_status(self):
        if self.debug_gcmd is None:
            return
        eventtime = self.reactor.monotonic()
        if eventtime < self.debug_next_log_time:
            return
        self.debug_next_log_time = eventtime + DEBUG_LOG_INTERVAL
        msg = self._format_debug_message(eventtime)
        gcmd = self.debug_gcmd
        self.reactor.register_async_callback(
            lambda eventtime, m=msg, g=gcmd: g.respond_info(m)
        )

    def _start(self, eventtime, direction, target_speed):
        if self.active:
            raise self.printer.command_error(
                "drv8833 %s is already running" % (self.name,)
            )
        self.active = True
        self.manual_mode = False
        self.direction = direction
        self.direction_name = "forwards" if direction > 0 else "backwards"
        self.target_speed = target_speed
        self.hall_controller.start(direction, target_speed)

    def _start_manual(self, eventtime, direction, duty_cycle):
        if not self.active:
            self.active = True
        self.manual_mode = True
        self.direction = direction
        self.direction_name = "forwards" if direction > 0 else "backwards"
        self.target_speed = 0.0
        self.hall_controller.manual_start(direction, duty_cycle)

    def _stop(self, eventtime):
        if not self.active:
            return
        self.active = False
        self.manual_mode = False
        self.target_speed = 0.0
        self.hall_controller.stop()

    def _save_pid_gains(self, kp, ki, kd):
        configfile = self.printer.lookup_object("configfile")
        configfile.set(self.section_name, "pid_kp", "%.6f" % (kp,))
        configfile.set(self.section_name, "pid_ki", "%.6f" % (ki,))
        configfile.set(self.section_name, "pid_kd", "%.6f" % (kd,))

    def set_pid_gains(self, kp, ki, kd, save_to_config=False):
        self.hall_controller.set_pid(kp, ki, kd)
        if save_to_config:
            self._save_pid_gains(kp, ki, kd)

    def _get_debug_totals(self):
        hall_clicks = self.hall_controller.last_count - self.debug_hall_start
        odometer_clicks = self.odometer.get_count() - self.debug_odometer_start
        return {
            "hall_clicks": hall_clicks,
            "hall_mm": hall_clicks * self.hall_controller.hall_resolution,
            "odometer_clicks": odometer_clicks,
            "odometer_mm": odometer_clicks * self.odometer.resolution,
        }

    def _format_debug_message(self, eventtime):
        totals = self._get_debug_totals()
        return (
            "drv8833 %s: direction=%s target=%.3fmm/s hall_clicks=%d "
            "hall_speed=%.3fmm/s odometer_clicks=%d odometer_speed=%.3fmm/s "
            "duty=%.1f%%"
            % (
                self.name,
                self.direction_name,
                self.target_speed,
                totals["hall_clicks"],
                self.hall_controller.last_speed_mm_s,
                totals["odometer_clicks"],
                self.odometer.get_speed(eventtime),
                self.hall_controller.last_duty_cycle,
            )
        )

    cmd_MOVE_DEBUG_help = (
        "Run the DRV8833 lane for a fixed time while logging sensor counts, "
        "sensor speeds, and duty cycle"
    )

    def cmd_MOVE_DEBUG(self, gcmd):
        duration = gcmd.get_float("TIME", 5.0, above=0.0)
        direction = gcmd.get("DIRECTION", "forwards").strip().lower()
        speed = gcmd.get_float("SPEED", 40.0, minval=0.0)
        if direction not in ("forwards", "forward", "backwards", "backward"):
            raise gcmd.error("DIRECTION must be forwards or backwards")
        direction_value = 1 if direction.startswith("for") else -1
        eventtime = self.reactor.monotonic()
        self.debug_gcmd = gcmd
        self.debug_next_log_time = eventtime
        # The MCU resets hall count to zero at the start of each move.
        self.debug_hall_start = 0
        self.debug_odometer_start = self.odometer.get_count()
        self._start(eventtime, direction_value, speed)
        endtime = eventtime + duration
        try:
            while eventtime < endtime:
                eventtime = self.reactor.pause(endtime)
        finally:
            self.debug_gcmd = None
            self._stop(self.reactor.monotonic())
        totals = self._get_debug_totals()
        gcmd.respond_info(
            "drv8833 %s move complete: hall_clicks=%d hall_distance=%.3fmm "
            "odometer_clicks=%d odometer_distance=%.3fmm"
            % (
                self.name,
                totals["hall_clicks"],
                totals["hall_mm"],
                totals["odometer_clicks"],
                totals["odometer_mm"],
            )
        )

    def cmd_SET_DRV8833_PID(self, gcmd):
        kp = gcmd.get_float("KP", self.hall_controller.pid_kp, minval=0.0)
        ki = gcmd.get_float("KI", self.hall_controller.pid_ki, minval=0.0)
        kd = gcmd.get_float("KD", self.hall_controller.pid_kd, minval=0.0)
        save_to_config = bool(gcmd.get_int("SAVE", 0))
        self.set_pid_gains(kp, ki, kd, save_to_config=save_to_config)
        msg = (
            "drv8833 %s PID updated: pid_kp=%.6f pid_ki=%.6f pid_kd=%.6f"
            % (self.name, kp, ki, kd)
        )
        if save_to_config:
            msg += (
                "\nThe SAVE_CONFIG command will update the printer config file "
                "and restart the printer."
            )
        gcmd.respond_info(msg)

    def cmd_DRV8833_PID_TUNE(self, gcmd):
        if self.active:
            raise gcmd.error("drv8833 %s is already running" % (self.name,))
        tuner = Drv8833PIDAutoTune(self, gcmd)
        try:
            result = tuner.run()
        finally:
            self._stop(self.reactor.monotonic())
        self.set_pid_gains(
            result["pid_kp"],
            result["pid_ki"],
            result["pid_kd"],
            save_to_config=True,
        )
        gcmd.respond_info(
            "drv8833 %s PID tune at %.3fmm/s: pid_kp=%.6f pid_ki=%.6f pid_kd=%.6f\n"
            "Ku=%.6f Tu=%.6fs amplitude=%.6fmm/s baseline_duty=%.2f%% relay_duty=%.2f%%\n"
            "The SAVE_CONFIG command will update the printer config file and restart the printer."
            % (
                self.name,
                tuner.target_speed,
                result["pid_kp"],
                result["pid_ki"],
                result["pid_kd"],
                result["ultimate_gain"],
                result["ultimate_period"],
                result["amplitude"],
                result["baseline_duty"],
                result["relay_duty"],
            )
        )


def load_config_prefix(config):
    return PrinterDrv8833(config)