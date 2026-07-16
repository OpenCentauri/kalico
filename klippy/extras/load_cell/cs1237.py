# CS1237 ADC load cell sensor support
#
# Copyright (C) 2024 OpenCentauri Contributors
# Modelled on hx71x.py by Gareth Farrington <gareth@waves.ky>
# Drop-in alternative to hx71x for use with [load_cell] tap interface.
#
# Usage (printer.cfg):
#
#   [load_cell]
#   sensor_type: cs1237
#   drdy_pin: <pin>    # DRDY / data pin
#   sclk_pin: <pin>    # SCLK pin
#   sample_rate: 40    # 10 | 40 | 640 | 1280  (default 40)
#   gain: 128          # 1 | 2 | 64 | 128       (default 128)
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy.mcu import MCU

from .. import bulk_sensor
from .interfaces import BulkAdcData, BulkAdcDataCallback, LoadCellSensor

# ---- Constants ----

UPDATE_INTERVAL = 0.10
SAMPLE_ERROR_DESYNC = -0x80000000
SAMPLE_ERROR_LONG_READ = 0x40000000

# CS1237 speed register encodings -> SPS
_SPEED_MAP = {10: 0, 40: 1, 640: 2, 1280: 3}

# CS1237 PGA gain register encodings
_GAIN_MAP = {1: 0, 2: 1, 64: 2, 128: 3}


class CS1237(LoadCellSensor):
    """CS1237 24-bit ADC load cell sensor driver.

    Implements the same LoadCellSensor interface as HX71x so it can be
    used as a drop-in replacement with [load_cell] / load_cell_probe.
    """

    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.consecutive_fails = 0

        # ---- Pin setup ----
        drdy_pin_name = config.get("drdy_pin")
        sclk_pin_name = config.get("sclk_pin")
        ppins = printer.lookup_object("pins")
        drdy_ppin = ppins.lookup_pin(drdy_pin_name)
        sclk_ppin = ppins.lookup_pin(sclk_pin_name)
        mcu: MCU = drdy_ppin["chip"]
        self.mcu: MCU = mcu
        self.oid = mcu.create_oid()
        if sclk_ppin["chip"] is not mcu:
            raise config.error(
                "%s: drdy_pin and sclk_pin must be on the same MCU"
                % self.name
            )
        self.drdy_pin = drdy_ppin["pin"]
        self.sclk_pin = sclk_ppin["pin"]

        # ---- Sample rate ----
        self.sps = config.getchoice(
            "sample_rate",
            {str(k): k for k in _SPEED_MAP},
            default="40",
        )
        self.speed_reg = _SPEED_MAP[self.sps]

        # ---- Gain ----
        self.gain_val = config.getchoice(
            "gain",
            {str(k): k for k in _GAIN_MAP},
            default="128",
        )
        self.gain_reg = _GAIN_MAP[self.gain_val]

        # ---- Bulk sensor setup ----
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth, "<i")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._finish_measurements,
            UPDATE_INTERVAL,
        )

        # ---- MCU command handles ----
        self.query_cs1237_cmd = None
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            "config_cs1237 oid=%d gain=%d speed=%d drdy_pin=%s sclk_pin=%s"
            % (
                self.oid,
                self.gain_reg,
                self.speed_reg,
                self.drdy_pin,
                self.sclk_pin,
            )
        )
        mcu.add_config_cmd(
            "query_cs1237 oid=%d rest_ticks=0" % (self.oid,),
            on_restart=True,
        )
        mcu.register_config_callback(self._build_config)

    # ---- LoadCellSensor interface ----

    def get_mcu(self) -> MCU:
        return self.mcu

    def get_samples_per_second(self) -> int:
        return self.sps

    def get_range(self) -> tuple:
        """CS1237 output is 24-bit two's complement."""
        return -0x800000, 0x7FFFFF

    def get_channel_count(self) -> int:
        return 1

    def add_client(self, callback: BulkAdcDataCallback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # ---- Internal ----

    def _build_config(self):
        self.query_cs1237_cmd = self.mcu.lookup_command(
            "query_cs1237 oid=%c rest_ticks=%u"
        )
        self.attach_probe_cmd = self.mcu.lookup_command(
            "cs1237_attach_load_cell_probe oid=%c load_cell_probe_oid=%c"
        )
        self.ffreader.setup_query_command(
            "query_cs1237_status oid=%c",
            oid=self.oid,
            cq=self.mcu.alloc_command_queue(),
        )

    def _convert_samples(self, samples):
        adc_factor = 1.0 / (1 << 23)
        count = 0
        for ptime, val in samples:
            if val == SAMPLE_ERROR_DESYNC or val == SAMPLE_ERROR_LONG_READ:
                self.last_error_count += 1
                break
            samples[count] = (round(ptime, 6), val, round(val * adc_factor, 9))
            count += 1
        del samples[count:]

    def _start_measurements(self):
        self.consecutive_fails = 0
        self.last_error_count = 0
        rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sps))
        self.query_cs1237_cmd.send([self.oid, rest_ticks])
        logging.info("cs1237 starting '%s' measurements", self.name)
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self.query_cs1237_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("cs1237 finished '%s' measurements", self.name)

    def _process_batch(self, eventtime) -> BulkAdcData:
        prev_overflows = self.ffreader.get_last_overflows()
        prev_error_count = self.last_error_count
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        overflows = self.ffreader.get_last_overflows() - prev_overflows
        errors = self.last_error_count - prev_error_count
        if errors > 0:
            logging.error("%s: forced sensor restart due to error", self.name)
            self._finish_measurements()
            self._start_measurements()
        elif overflows > 0:
            self.consecutive_fails += 1
            if self.consecutive_fails > 4:
                logging.error(
                    "%s: forced sensor restart due to overflows", self.name
                )
                self._finish_measurements()
                self._start_measurements()
        else:
            self.consecutive_fails = 0
        return {
            "data": samples,
            "errors": self.last_error_count,
            "overflows": self.ffreader.get_last_overflows(),
        }


CS1237_SENSOR_TYPES = {"cs1237": CS1237}
