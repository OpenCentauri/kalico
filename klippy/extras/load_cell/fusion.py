# Load Cell Fusion Sensor
#
# Fuses readings from named sensor sections into a single stream using the
# MCU-side load_cell_fusion module.
#
# Config example:
#   [load_cell bed]
#   sensor_type: load_cell_fusion
#   sensors: left, !right
#
# Copyright (C) 2026 Timo V
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy.extras import bulk_sensor
from klippy.extras.load_cell.hx71x import (
    SAMPLE_ERROR_DESYNC,
    SAMPLE_ERROR_LONG_READ,
    UPDATE_INTERVAL,
)
from klippy.extras.load_cell.interfaces import (
    BulkAdcData,
    BulkAdcDataCallback,
    LoadCellSensor,
)


class LoadCellFusion(LoadCellSensor):
    def _parse_sensor_ref(self, config, sensor_ref):
        invert = sensor_ref.startswith("!")
        sensor_name = sensor_ref[1:].strip() if invert else sensor_ref.strip()
        return sensor_name, invert

    def _load_sensor(self, config, sensor_name):
        try:
            return self.printer.load_object(config, sensor_name)
        except self.printer.config_error as error:
            if " " not in sensor_name:
                raise config.error(
                    "load_cell_fusion: unknown sensor '%s'"
                    " (expected full section name such as 'hx71x %s')"
                    % (sensor_name, sensor_name)
                ) from error
            raise

    def __init__(self, config):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.sensor_type = "load_cell_fusion"
        self.last_error_count = 0
        self.consecutive_fails = 0
        sensor_refs = [
            self._parse_sensor_ref(config, sensor_ref)
            for sensor_ref in config.getlist("sensors")
        ]
        self.sensors = [
            self._load_sensor(config, section_name)
            for section_name, _invert in sensor_refs
        ]
        if not self.sensors:
            raise config.error("load_cell_fusion: no sensors specified")
        self.mcu = self.sensors[0].get_mcu()
        self.sensor_sps = self.sensors[0].get_samples_per_second()
        self.sps = self.sensor_sps * len(self.sensors)
        self.range = self.sensors[0].get_range()
        for sensor in self.sensors[1:]:
            if sensor.get_mcu() is not self.mcu:
                raise config.error(
                    "load_cell_fusion: all sensors must be on the same MCU"
                )
            if sensor.get_samples_per_second() != self.sensor_sps:
                raise config.error(
                    "load_cell_fusion: all sensors must have the same "
                    "sample rate"
                )
            if sensor.get_range() != self.range:
                raise config.error(
                    "load_cell_fusion: all sensors must have the same range"
                )
        self.oid = self.mcu.create_oid()
        self.mcu.add_config_cmd("config_load_cell_fusion oid=%d" % self.oid)
        for sensor, (_section_name, invert) in zip(self.sensors, sensor_refs):
            self.mcu.add_config_cmd(
                sensor.build_attach_fusion_cmd(self.oid, invert)
            )
        self.mcu.add_config_cmd(
            "query_load_cell_fusion oid=%d active=0" % self.oid,
            on_restart=True,
        )
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        self.ffreader = bulk_sensor.FixedFreqReader(self.mcu, chip_smooth, "<i")
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer, self._process_batch, self._start_measurements,
            self._finish_measurements, UPDATE_INTERVAL
        )
        self.query_fusion_cmd = None
        self.attach_probe_cmd = None
        self.mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.query_fusion_cmd = self.mcu.lookup_command(
            "query_load_cell_fusion oid=%c active=%c")
        self.attach_probe_cmd = self.mcu.lookup_command(
            "load_cell_fusion_attach_probe oid=%c load_cell_probe_oid=%c")
        self.ffreader.setup_query_command(
            "query_load_cell_fusion_status oid=%c",
            oid=self.oid, cq=self.mcu.alloc_command_queue())

    def get_mcu(self):
        return self.mcu

    def get_samples_per_second(self) -> int:
        return self.sps

    def get_range(self) -> tuple[int, int]:
        return self.range

    def add_client(self, callback: BulkAdcDataCallback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

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
        rest_ticks = self.mcu.seconds_to_clock(1.0 / (10.0 * self.sensor_sps))
        for sensor in self.sensors:
            sensor.prepare_sampling()
        self.query_fusion_cmd.send([self.oid, 1])
        for sensor in self.sensors:
            sensor.start_sampling(rest_ticks)
        logging.info("%s starting '%s' measurements", self.sensor_type, self.name)
        self.ffreader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        for sensor in self.sensors:
            sensor.stop_sampling()
        self.query_fusion_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info("%s finished '%s' measurements", self.sensor_type, self.name)

    def _process_batch(self, eventtime) -> BulkAdcData:
        prev_overflows = self.ffreader.get_last_overflows()
        prev_error_count = self.last_error_count
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        overflows = self.ffreader.get_last_overflows() - prev_overflows
        errors = self.last_error_count - prev_error_count
        if errors > 0:
            logging.error("%s: Forced sensor restart due to error", self.name)
            self._finish_measurements()
            self._start_measurements()
        elif overflows > 0:
            self.consecutive_fails += 1
            if self.consecutive_fails > 4:
                logging.error(
                    "%s: Forced sensor restart due to overflows", self.name
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


LOAD_CELL_FUSION_SENSOR_TYPE = {"load_cell_fusion": LoadCellFusion}
