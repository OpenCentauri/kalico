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

# trsync trigger reasons
REASON_ENDSTOP_HIT = klippy_mcu.MCU_trsync.REASON_ENDSTOP_HIT
REASON_COMMS_TIMEOUT = klippy_mcu.MCU_trsync.REASON_COMMS_TIMEOUT


class HX711SEndstopWrapper:
    """Endstop wrapper implementing Kalico's mcu_probe interface for
    trsync-based probing with HX711S strain gauge sensors."""
    def __init__(self, config, hx711s):
        self._hx711s = hx711s
        self._printer = hx711s.printer
        self._mcu = hx711s.mcu
        self._z_offset = config.getfloat('z_offset')
        # Use TriggerDispatch for proper trsync coordination
        self._dispatch = klippy_mcu.TriggerDispatch(self._mcu)
        # Discover Z steppers after MCU identification
        self._printer.register_event_handler(
            'klippy:mcu_identify', self._handle_mcu_identify)
        # Build homing command
        self._home_cmd = None
        self._query_cmd = None
        self._mcu.register_config_callback(self._build_config)

    def _handle_mcu_identify(self):
        kin = self._printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis('z'):
                self.add_stepper(stepper)

    def _build_config(self):
        self._home_cmd = self._mcu.lookup_command(
            "hx711s_home oid=%c trsync_oid=%c clock=%u"
            " trigger_reason=%c error_reason=%c")
        self._query_cmd = self._mcu.lookup_query_command(
            "hx711s_query_state oid=%c",
            "hx711s_state oid=%c is_triggered=%c trigger_ticks=%u",
            oid=self._hx711s.oid)

    # MCU endstop interface
    def get_mcu(self):
        return self._mcu

    def add_stepper(self, stepper):
        self._dispatch.add_stepper(stepper)

    def get_steppers(self):
        return self._dispatch.get_steppers()

    def home_start(self, print_time, sample_time, sample_count, rest_time,
                   triggered=True):
        self._hx711s.is_trigger = 0
        clock = self._mcu.print_time_to_clock(print_time)
        trigger_completion = self._dispatch.start(print_time)
        self._home_cmd.send([
            self._hx711s.oid,
            self._dispatch.get_oid(),
            clock,
            REASON_ENDSTOP_HIT,
            REASON_COMMS_TIMEOUT
        ], reqclock=clock)
        return trigger_completion

    def home_wait(self, home_end_time):
        self._dispatch.wait_end(home_end_time)
        params = self._query_cmd.send([self._hx711s.oid])
        trigger_ticks = params['trigger_ticks']
        is_triggered = params['is_triggered']
        logging.info("HX711S: home_wait query: is_triggered=%s"
                     " trigger_ticks=%u", is_triggered, trigger_ticks)
        # Stop homing on MCU before dispatch.stop()
        self._home_cmd.send([self._hx711s.oid, 0, 0, 0, 0])
        res = self._dispatch.stop()
        logging.info("HX711S: home_wait dispatch reason=%d", res)
        if res >= REASON_COMMS_TIMEOUT:
            raise self._printer.command_error(
                "HX711S: Communication timeout during homing")
        if res != REASON_ENDSTOP_HIT:
            return 0.
        if trigger_ticks:
            trigger_time = self._mcu.clock_to_print_time(
                self._mcu.clock32_to_clock64(trigger_ticks))
            logging.info("HX711S: trigger_time=%.6f (ticks=%u)",
                         trigger_time, trigger_ticks)
            return trigger_time
        logging.warning("HX711S: trigger but trigger_ticks=0")
        return 0.

    def query_endstop(self, print_time):
        return self._hx711s.is_triggered()

    def probing_move(self, pos, speed, gcmd):
        """Calibrate sensors then execute a trsync-based probing move."""
        toolhead = self._printer.lookup_object('toolhead')
        # Wait for pending moves (retract) to complete before calibrating
        # so the sensor baseline isn't contaminated by bed contact
        toolhead.wait_moves()
        toolhead.dwell(0.100)
        if not self._hx711s.calibration_start(30, 5.0):
            raise self._printer.command_error(
                "HX711S: Calibration failed before probe")
        phoming = self._printer.lookup_object('homing')
        return phoming.probing_move(self, pos, speed)

    def multi_probe_begin(self):
        pass

    def multi_probe_end(self):
        pass

    def probe_prepare(self, hmove):
        toolhead = self._printer.lookup_object('toolhead')
        toolhead.wait_moves()
        toolhead.dwell(0.100)
        pass

    def probe_finish(self, hmove):
        pass

    def get_position_endstop(self):
        return self._z_offset


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

        # Create endstop wrapper implementing Kalico's mcu_probe interface
        self._probe = HX711SEndstopWrapper(config, self)

        # Register as probe via Kalico's PrinterProbe
        self.printer.add_object(
            'probe', probe.PrinterProbe(config, self._probe))

        # Register sensor-specific GCode commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('HX_MULTI_CALIBRATE', self.cmd_HX711S_CALIBRATE,
                               desc="Calibrate HX711S strain gauge sensors")
        logging.info("HX711S: Initialized with %d sensors", self.sensor_count)

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

    def _handle_debug_hx711s(self, params):
        logging.debug("HX711S DEBUG: 0x%02x 0x%02x 0x%02x 0x%02x",
                      params.get('arg[0]', 0), params.get('arg[1]', 0),
                      params.get('arg[2]', 0), params.get('arg[3]', 0))

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

    def is_triggered(self):
        return self.is_trigger > 0

    def is_calibrated(self):
        return bool(self.is_calibration & CALIBRATION_OK_BIT)

    def get_sensor_errors(self):
        """Return list of sensor indices with errors."""
        return [i for i in range(self.sensor_count)
                if self.is_calibration & (1 << i)]

    def get_status(self, eventtime):
        return {
            'is_calibrated': self.is_calibrated(),
            'is_triggered': self.is_triggered(),
            'trigger_index': self.trigger_index,
            'trigger_timestamp': self.trigger_timestamp,
            'sensor_errors': self.get_sensor_errors(),
            'sensor_count': self.sensor_count,
        }

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


def load_config(config):
    return HX711S(config)
