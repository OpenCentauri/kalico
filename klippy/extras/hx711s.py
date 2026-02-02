# HX711S Strain Gauge Sensor Support for Bed Probing
#
# Copyright (C) 2026 Timo V
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import logging
from . import probe
from klippy import mcu as klippy_mcu

# Calibration status: bit 7 = success, bits 0-3 = sensor errors
CALIBRATION_OK_BIT = 0x80

# Probing parameters based on Elegoo strain_gauge.cpp implementation
PROBE_SPEED = 2.0       # mm/s - matches Elegoo m_g29_speed
PROBE_LIFT_SPEED = 5.0  # mm/s - faster retract
PROBE_LIFT_HEIGHT = 1.0 # mm - lift between double-tap probes

# trsync trigger reasons
REASON_ENDSTOP_HIT = klippy_mcu.MCU_trsync.REASON_ENDSTOP_HIT
REASON_COMMS_TIMEOUT = klippy_mcu.MCU_trsync.REASON_COMMS_TIMEOUT


class HX711SEndstopWrapper:
    """Endstop wrapper using trsync for proper homing integration."""
    def __init__(self, config, hx711s):
        self._hx711s = hx711s
        self._printer = hx711s.printer
        self._mcu = hx711s.mcu
        # Use TriggerDispatch for proper trsync coordination
        self._dispatch = klippy_mcu.TriggerDispatch(self._mcu)
        # Register Z steppers
        probe.LookupZSteppers(config, self._dispatch.add_stepper)
        # Build homing command
        self._home_cmd = None
        self._query_cmd = None
        self._mcu.register_config_callback(self._build_config)

    def _build_config(self):
        # Lookup commands for trsync-based homing
        self._home_cmd = self._mcu.lookup_command(
            "hx711s_home oid=%c trsync_oid=%c trigger_reason=%c error_reason=%c")
        self._query_cmd = self._mcu.lookup_query_command(
            "hx711s_query_state oid=%c",
            "hx711s_state oid=%c is_triggered=%c trigger_ticks=%u",
            oid=self._hx711s.oid)

    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        # Clear Python-side trigger state before starting new probe
        self._hx711s.is_trigger = 0
        # Convert print_time to MCU clock for synchronization
        clock = self._mcu.print_time_to_clock(print_time)
        # Start trsync
        trigger_completion = self._dispatch.start(print_time)
        # Tell MCU to start homing with trsync - synchronized with motion start
        self._home_cmd.send([
            self._hx711s.oid,
            self._dispatch.get_oid(),
            REASON_ENDSTOP_HIT,
            REASON_COMMS_TIMEOUT
        ], reqclock=clock)
        return trigger_completion

    def home_wait(self, home_end_time):
        # Wait for trsync to complete
        self._dispatch.wait_end(home_end_time)
        # Query the trigger time from MCU BEFORE stopping anything
        params = self._query_cmd.send([self._hx711s.oid])
        trigger_ticks = params['trigger_ticks']
        is_triggered = params['is_triggered']
        logging.info("HX711S: home_wait query result: is_triggered=%s trigger_ticks=%u",
                    is_triggered, trigger_ticks)
        # Stop homing on MCU (must be before dispatch.stop() per MCU_endstop pattern)
        # MCU will start reporting real-time trigger state (not latched) after this
        self._home_cmd.send([self._hx711s.oid, 0, 0, 0])
        # Now stop trsync dispatch
        res = self._dispatch.stop()
        logging.info("HX711S: home_wait dispatch.stop() returned reason=%d", res)
        # Check result
        if res >= REASON_COMMS_TIMEOUT:
            raise self._printer.command_error(
                "HX711S: Communication timeout during homing")
        if res != REASON_ENDSTOP_HIT:
            return 0.
        # Convert and return timestamp
        if trigger_ticks:
            trigger_time = self._mcu.clock_to_print_time(
                self._mcu.clock32_to_clock64(trigger_ticks))
            logging.info("HX711S: home_wait returning trigger_time=%.6f (ticks=%u)",
                        trigger_time, trigger_ticks)
            return trigger_time
        logging.warning("HX711S: home_wait got trigger but trigger_ticks=0")
        return 0.

    def query_endstop(self, print_time):
        return self._hx711s.is_triggered()


class HX711SProbeSession:
    """Probe session using homing infrastructure for continuous motion."""
    def __init__(self, config, hx711s, param_helper, z_min):
        self._hx711s = hx711s
        self._printer = hx711s.printer
        self._param_helper = param_helper
        self._z_min = z_min
        self._results = []
        # Endstop wrapper now takes config for stepper registration
        self._endstop_wrapper = HX711SEndstopWrapper(config, hx711s)

    def start_probe_session(self, gcmd):
        return self

    def end_probe_session(self):
        self._results = []

    def _single_probe(self, phoming, speed):
        """Single probe using phoming.probing_move() for continuous motion."""
        toolhead = self._printer.lookup_object('toolhead')
        curpos = toolhead.get_position()

        # Reset trigger state
        self._hx711s.is_trigger = 0

        pos = list(curpos)
        pos[2] = self._z_min

        # probing_move returns the exact trigger position calculated from
        # stepper steps and trigger timestamp via trsync
        epos = phoming.probing_move(self._endstop_wrapper, pos, speed)

        logging.info("HX711S: Probe at z=%.6f", epos[2])
        return epos[2]

    def run_probe(self, gcmd):
        """Double-tap probe using phoming.probing_move() for continuous motion."""
        toolhead = self._printer.lookup_object('toolhead')
        phoming = self._printer.lookup_object('homing')
        params = self._param_helper.get_probe_params(gcmd)
        speed = params['probe_speed']

        # Elegoo's _probe_times pattern: retry if z1 and z2 differ too much
        MAX_Z_ERR = 0.1  # mm - tolerance between double-tap probes
        MAX_RETRIES = 3

        for attempt in range(MAX_RETRIES):
            # First probe
            toolhead.wait_moves()
            if not self._hx711s.calibration_start(30, 5.0):
                raise self._printer.command_error("HX711S: Calibration failed")

            z1 = self._single_probe(phoming, speed)

            # Lift and second probe
            curpos = toolhead.get_position()
            toolhead.manual_move([None, None, curpos[2] + PROBE_LIFT_HEIGHT],
                                PROBE_LIFT_SPEED)
            toolhead.wait_moves()

            # Small delay for sensor to stabilize after lift
            # The trigger detection algorithm needs fresh "not touching" samples
            self._hx711s.reactor.pause(self._hx711s.reactor.monotonic() + 0.5)

            z2 = self._single_probe(phoming, speed)

            # Check tolerance
            z_diff = abs(z1 - z2)
            if z_diff <= MAX_Z_ERR:
                # Good result
                z_avg = (z1 + z2) / 2.0
                logging.info("HX711S: Probe z1=%.4f z2=%.4f diff=%.4f avg=%.4f",
                            z1, z2, z_diff, z_avg)
                break
            else:
                # Results differ too much, retry
                logging.warning("HX711S: Probe retry %d - z1=%.4f z2=%.4f diff=%.4f > %.4f",
                               attempt + 1, z1, z2, z_diff, MAX_Z_ERR)
                # Lift before retry
                curpos = toolhead.get_position()
                toolhead.manual_move([None, None, curpos[2] + PROBE_LIFT_HEIGHT],
                                    PROBE_LIFT_SPEED)
        else:
            # Max retries exceeded, use last result anyway
            z_avg = (z1 + z2) / 2.0
            logging.warning("HX711S: Max retries exceeded, using avg=%.4f", z_avg)

        final_pos = toolhead.get_position()
        final_pos[2] = z_avg
        self._results.append(final_pos)

    def pull_probed_results(self):
        res = self._results
        self._results = []
        return res


class HX711S:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.reactor = self.printer.get_reactor()

        # Parse configuration
        self.sensor_count = config.getint('count', 3, minval=1, maxval=4)
        self.enable_test = config.getint('enable_test', 0, minval=0, maxval=15)
        self.sg_mode = config.getint('sg_mode', 4, minval=0, maxval=16)
        self.install_dir = config.getint('install_dir', 15, minval=0, maxval=15)
        self.enable_hpf = config.getint('enable_hpf', 1, minval=0, maxval=15)
        self.find_index_mode = config.getint('find_index_mode', 1,
                                             minval=0, maxval=15)
        self.enable_shake_filter = config.getint('enable_shake_filter', 1,
                                                 minval=0, maxval=1)
        self.enable_channels = config.getint('enable_channels', 31,
                                             minval=0, maxval=31)
        self.rest_ticks = config.getint('rest_ticks', 5000,
                                        minval=100, maxval=1500000)
        self.kalman_q = config.getint('kalman_q', 1,
                                      minval=-100000, maxval=100000)
        self.kalman_r = config.getint('kalman_r', 10,
                                      minval=-100000, maxval=100000)
        self.min_th = config.getint('min_th', 2000, minval=-20000, maxval=20000)
        self.max_th = config.getint('max_th', 6000, minval=-60000, maxval=60000)
        self.th_k = config.getint('th_k', 2000, minval=-6000, maxval=6000)
        self.k_slope = config.getint('k_slope', 1000,
                                     minval=-10000, maxval=10000)
        self.bias_slope = config.getint('bias_slope', 120,
                                        minval=-10000, maxval=10000)

        # Lookup and validate sensor pins
        ppins = self.printer.lookup_object('pins')
        self.clk_pin_objs = []
        self.sdo_pin_objs = []
        for i in range(self.sensor_count):
            clk_pin_name = config.get('sensor%d_clk_pin' % i)
            sdo_pin_name = config.get('sensor%d_sdo_pin' % i)
            clk = ppins.lookup_pin(clk_pin_name)
            sdo = ppins.lookup_pin(sdo_pin_name)
            self.clk_pin_objs.append(clk)
            self.sdo_pin_objs.append(sdo)

        # All pins must be on same MCU
        self.mcu = self.clk_pin_objs[0]['chip']
        for i in range(self.sensor_count):
            if (self.clk_pin_objs[i]['chip'] != self.mcu or
                self.sdo_pin_objs[i]['chip'] != self.mcu):
                raise config.error(
                    "HX711S sensor%d pins must be on same MCU" % i)
        self.oid = self.mcu.create_oid()

        # State
        self.is_calibration = 0
        self.is_trigger = 0
        self.trigger_tick = 0
        self.trigger_timestamp = 0.
        self.trigger_index = 0
        self._calibration_ack = False
        self._probe_cmd_ack = False

        # Register handlers and build config
        self.mcu.register_response(self._handle_debug_hx711s,
                                   "debug_hx711s", self.oid)
        self.mcu.register_response(self._handle_sg_resp, "sg_resp", self.oid)
        self.mcu.register_config_callback(self._build_config)

        # Get z_min for probing limits
        self._z_min = probe.lookup_minimum_z(config)
        self._position_endstop = config.getfloat('z_offset')

        # Create probe helpers
        self._probe_offsets = probe.ProbeOffsetsHelper(config)
        self._param_helper = probe.ProbeParameterHelper(config)
        self._cmd_helper = probe.ProbeCommandHelper(
            config, self, self._query_endstop)

        # Custom probe session for incremental probing
        self._probe_session = HX711SProbeSession(
            config, self, self._param_helper, self._z_min)

        # Register as the probe object
        self.printer.add_object('probe', self)

        # Register additional GCode commands for manual testing
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('HX_MULTI_CALIBRATE', self.cmd_HX711S_CALIBRATE,
                               desc="Calibrate HX711S strain gauge sensors")
        gcode.register_command('HX_MULTI_STATUS', self.cmd_HX711S_STATUS,
                               desc="Report HX711S sensor status")
        gcode.register_command('HX_MULTI_TEST', self.cmd_HX711S_TEST,
                               desc="Test HX711S trigger detection")

        logging.info("HX711S: Initialized with %d sensors", self.sensor_count)

    def _query_endstop(self, print_time):
        return self.is_triggered()

    def _build_config(self):
        # Pack configuration fields per MCU protocol
        hx711_count = ((self.install_dir & 0x0F) << 4) | \
                      (self.sensor_count & 0x0F)
        rest_ticks_packed = ((self.enable_test & 0x0F) << 28) | \
                           ((self.sg_mode & 0x0F) << 24) | \
                           (self.rest_ticks & 0xFFFFFF)
        channels_packed = ((self.enable_shake_filter & 0x0F) << 16) | \
                         ((self.find_index_mode & 0x0F) << 12) | \
                         ((self.enable_hpf & 0x0F) << 8) | \
                         (self.enable_channels & 0xFF)
        k_packed = ((self.k_slope & 0xFFFF) << 16) | \
                  ((self.bias_slope & 0xFF) << 8) | \
                  (self.th_k & 0xFF)

        self.mcu.add_config_cmd(
            "config_hx711s oid=%d hx711_count=%d channels=%d rest_ticks=%d "
            "kalman_q=%d kalman_r=%d max_th=%d min_th=%d k=%d"
            % (self.oid, hx711_count, channels_packed, rest_ticks_packed,
               self.kalman_q, self.kalman_r, self.max_th, self.min_th,
               k_packed))

        # Add sensor pins
        for i in range(self.sensor_count):
            clk = self.clk_pin_objs[i]
            sdo = self.sdo_pin_objs[i]
            self.mcu.add_config_cmd(
                "add_hx711s oid=%d index=%d clk_pin=%s sdo_pin=%s"
                % (self.oid, i, clk['pin'], sdo['pin']))

        # Lookup commands
        self._cmd_queue = self.mcu.alloc_command_queue()
        self._calibration_cmd = self.mcu.lookup_command(
            "calibration_sample oid=%c times_read=%hu", cq=self._cmd_queue)
        # Note: query_hx711s and sg_probe_check are deprecated with trsync
        # The hx711s_home command is looked up in HX711SEndstopWrapper

    def _handle_debug_hx711s(self, params):
        arg0, arg1 = params.get('arg[0]', 0), params.get('arg[1]', 0)
        if arg0:
            self._calibration_ack = True
        if arg1:
            self._probe_cmd_ack = True

    def _handle_sg_resp(self, params):
        self.is_calibration = params.get('vd', 0)
        self.trigger_index = params.get('it', 0)
        self.trigger_tick = params.get('nt', 0)
        new_trigger = params.get('r', 0)
        if self.trigger_tick:
            self.trigger_timestamp = self.mcu.clock_to_print_time(
                self.mcu.clock32_to_clock64(self.trigger_tick))
        # Only log on trigger transition (0 -> non-zero)
        if new_trigger > 0 and self.is_trigger == 0:
            logging.info("HX711S: TRIGGER 0x%02x idx=%d ts=%.6f",
                        new_trigger, self.trigger_index,
                        self.trigger_timestamp)
        self.is_trigger = new_trigger

    # Public API
    def calibration_start(self, samples=30, timeout=5.0):
        """Start calibration. Returns True on success."""
        self._calibration_ack = False
        self.is_calibration = 0
        self.is_trigger = 0
        self._calibration_cmd.send([self.oid, samples])

        # Wait for calibration complete (MCU sets bit 7 in sg_resp)
        endtime = self.reactor.monotonic() + timeout
        while self.reactor.monotonic() < endtime:
            if self.is_calibration & CALIBRATION_OK_BIT:
                return True
            self.reactor.pause(self.reactor.monotonic() + 0.05)
        logging.error("HX711S: Calibration timeout")
        return False

    def wait_for_trigger(self, timeout=10.0):
        """Wait for trigger. Returns True if triggered."""
        endtime = self.reactor.monotonic() + timeout
        while self.reactor.monotonic() < endtime:
            if self.is_trigger > 0:
                return True
            self.reactor.pause(self.reactor.monotonic() + 0.01)
        return False

    # Note: probe_trigger_start/stop are deprecated with trsync
    # The trsync mechanism handles trigger detection automatically

    def is_triggered(self):
        return self.is_trigger > 0

    def is_calibrated(self):
        return bool(self.is_calibration & CALIBRATION_OK_BIT)

    def get_trigger_timestamp(self):
        return self.trigger_timestamp

    def get_sensor_errors(self):
        """Return list of sensor indices with errors."""
        return [i for i in range(self.sensor_count)
                if self.is_calibration & (1 << i)]

    def get_status(self, eventtime):
        status = self._cmd_helper.get_status(eventtime)
        status.update({
            'is_calibrated': self.is_calibrated(),
            'is_triggered': self.is_triggered(),
            'trigger_index': self.trigger_index,
            'trigger_timestamp': self.trigger_timestamp,
            'sensor_errors': self.get_sensor_errors(),
            'sensor_count': self.sensor_count,
        })
        return status

    # Probe interface methods
    def get_probe_params(self, gcmd=None):
        return self._param_helper.get_probe_params(gcmd)

    def get_offsets(self):
        return self._probe_offsets.get_offsets()

    def start_probe_session(self, gcmd):
        return self._probe_session.start_probe_session(gcmd)

    # GCode commands
    def cmd_HX711S_CALIBRATE(self, gcmd):
        samples = gcmd.get_int('SAMPLES', 30, minval=5, maxval=1000)
        timeout = gcmd.get_float('TIMEOUT', 5.0, minval=1.0, maxval=30.0)
        gcmd.respond_info("HX711S: Calibrating with %d samples..." % samples)
        if self.calibration_start(samples, timeout):
            errors = self.get_sensor_errors()
            msg = "HX711S: Calibration OK"
            if errors:
                msg += " (sensor errors: %s)" % errors
            gcmd.respond_info(msg)
        else:
            raise gcmd.error("HX711S: Calibration failed!")

    def cmd_HX711S_STATUS(self, gcmd):
        errors = self.get_sensor_errors()
        gcmd.respond_info(
            "HX711S Status:\n"
            "  Sensors: %d\n"
            "  Calibrated: %s (0x%02x)\n"
            "  Triggered: %s (0x%02x)\n"
            "  Trigger index: %d\n"
            "  Sensor errors: %s\n"
            "  Trigger timestamp: %.6f"
            % (self.sensor_count,
               "YES" if self.is_calibrated() else "NO", self.is_calibration,
               "YES" if self.is_triggered() else "NO", self.is_trigger,
               self.trigger_index,
               errors if errors else "None",
               self.trigger_timestamp))


def load_config(config):
    return HX711S(config)
