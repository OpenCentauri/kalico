# Multi-Sensor HX711 and HX717 Support
#
# Copyright (C) 2026 James Turton <james.turton@gmx.com>
# Original HX711 driver Copyright (C) 2024 Gareth Farrington <gareth@waves.ky>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging

from klippy.mcu import MCU

from .. import bulk_sensor
from .interfaces import BulkAdcData, BulkAdcDataCallback, LoadCellSensor

#
# Constants
#
UPDATE_INTERVAL = 0.10
SAMPLE_ERROR_DESYNC = -0x80000000
SAMPLE_ERROR_LONG_READ = 0x40000000
SAMPLE_ERROR_BAD_FRAME = 0x20000000
SAMPLE_ERROR_RECOVERED = 0x10000000


# Implementation of multiple HX711 and HX717 chips as one load cell
class HX711SBase(LoadCellSensor):
    def __init__(
        self,
        config,
        sensor_type,
        sample_rate_options,
        default_sample_rate,
        gain_options,
        default_gain,
    ):
        self.printer = printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.last_error_count = 0
        self.recovered_count = 0
        self.consecutive_fails = 0
        self.sensor_type = sensor_type
        self.bad_frame_count = 0
        self._bad_frame_channels = set()
        # Chip options
        ppins = printer.lookup_object("pins")
        sdo_pin_names = [p.strip() for p in config.get("sdo_pins").split(",")]
        sclk_pin_names = [p.strip() for p in config.get("sclk_pins").split(",")]
        if len(sdo_pin_names) != len(sclk_pin_names):
            raise config.error(
                f"{sensor_type}: sdo_pins and sclk_pins must have the same"
                " number of entries"
            )
        self.sensor_count = len(sdo_pin_names)
        if self.sensor_count < 1 or self.sensor_count > 4:
            raise config.error(
                f"{sensor_type}: must specify 1 to 4 sensor pin pairs"
            )
        # Resolve all pins and validate they share one MCU
        sdo_ppins = [ppins.lookup_pin(p) for p in sdo_pin_names]
        sclk_ppins = [ppins.lookup_pin(p) for p in sclk_pin_names]
        mcu: MCU = sdo_ppins[0]["chip"]
        self.mcu: MCU = mcu
        for ppin in sdo_ppins[1:] + sclk_ppins:
            if ppin["chip"] is not mcu:
                raise config.error(
                    f"{sensor_type}: all pins must be on the same MCU"
                )
        self.sdo_pins = [p["pin"] for p in sdo_ppins]
        self.sclk_pins = [p["pin"] for p in sclk_ppins]
        # Samples per second choices
        self.sps = config.getchoice(
            "sample_rate", sample_rate_options, default=default_sample_rate
        )
        # gain/channel choices
        self.gain_channel = int(
            config.getchoice("gain", gain_options, default=default_gain)
        )
        stuck_ms = config.getint("stuck_timeout_ms", 0, minval=0)
        if not stuck_ms:
            # ~2.5 conversion periods + margin at the configured rate
            stuck_ms = 2500 // self.sps + 5
        self.stuck_ms = stuck_ms
        # Hold single-sample force jumps larger than this from the probe
        # trigger feed. Must stay below trigger_force converted to counts
        # (trigger_force x counts_per_gram) or phantom impulses leak.
        self.spike_sum_threshold = config.getint(
            "impulse_threshold_counts", 5775, minval=1)
        # Post-wake settling window: 4 conversions + margin (datasheet:
        # output valid from the 4th conversion after power-up)
        self.settle_ms = config.getint(
            "settle_ms", 4000 // self.sps + 10, minval=1)
        self.oid = mcu.create_oid()
        ## Bulk Sensor Setup
        # Clock tracking
        chip_smooth = self.sps * UPDATE_INTERVAL * 2
        unpack_format = "<" + ("i" * self.sensor_count)
        self.ffreader = bulk_sensor.FixedFreqReader(mcu, chip_smooth,
                                                    unpack_format)
        # Process messages in batches
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._finish_measurements,
            UPDATE_INTERVAL,
        )
        # Command Configuration
        self.query_hx711s_cmd = None
        self.attach_probe_cmd = None
        mcu.add_config_cmd(
            f"config_hx711s oid={self.oid}"
            f" sensor_count={self.sensor_count}"
            f" gain_channel={self.gain_channel}"
        )
        for i, (sdo, sclk) in enumerate(zip(self.sdo_pins, self.sclk_pins)):
            mcu.add_config_cmd(
                f"add_hx711s oid={self.oid} index={i}"
                f" sdo_pin={sdo} sclk_pin={sclk}"
            )
        mcu.add_config_cmd(
            f"query_hx711s oid={self.oid} rest_ticks=0", on_restart=True
        )
        mcu.add_config_cmd(
            f"hx711s_set_tuning oid={self.oid}"
            f" stuck_ms={self.stuck_ms}"
            f" spike_sum_threshold={self.spike_sum_threshold}"
            f" settle_ms={self.settle_ms}"
        )
        mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.query_hx711s_cmd = self.mcu.lookup_command(
            "query_hx711s oid=%c rest_ticks=%u"
        )
        self.attach_probe_cmd = self.mcu.lookup_command(
            "hx711s_attach_load_cell_probe oid=%c load_cell_probe_oid=%c"
        )
        self.ffreader.setup_query_command(
            "query_hx711s_status oid=%c",
            oid=self.oid,
            cq=self.mcu.alloc_command_queue(),
        )

    def get_mcu(self) -> MCU:
        return self.mcu

    def get_samples_per_second(self) -> int:
        return self.sps

    # returns a tuple of the minimum and maximum value of the sensor, used to
    # detect if a data value is saturated
    def get_range(self) -> tuple[int, int]:
        return -0x800000, 0x7FFFFF

    def get_channel_count(self) -> int:
        return self.sensor_count

    # add_client interface, direct pass through to bulk_sensor API
    def add_client(self, callback: BulkAdcDataCallback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    # Measurement decoding
    def _convert_samples(self, samples):
        adc_factor = 1.0 / (1 << 23)
        count = 0
        for sample in samples:
            ptime = sample[0]
            channel_counts = sample[1:]
            val = channel_counts[0]
            if val == SAMPLE_ERROR_DESYNC:
                self.last_error_count += 1
                logging.error("%s: DESYNC at t=%.3f", self.name, ptime)
                break  # errors latch in the MCU, the rest are duplicates
            elif val == SAMPLE_ERROR_LONG_READ:
                self.last_error_count += 1
                logging.error("%s: READ_TOO_LONG at t=%.3f", self.name, ptime)
                break  # errors latch in the MCU, the rest are duplicates
            elif val == SAMPLE_ERROR_RECOVERED:
                self.recovered_count += 1
                logging.warning(
                    "%s: in-driver recovery (power cycle) at t=%.3f",
                    self.name, ptime)
                continue  # one-shot marker; later frames are valid data
            # A latched error fills every channel, but a bad frame marks only
            # the chip that produced it, so every channel has to be checked
            bad = [
                i
                for i, c in enumerate(channel_counts)
                if c == SAMPLE_ERROR_BAD_FRAME
            ]
            if bad:
                self.bad_frame_count += 1
                self._bad_frame_channels.update(bad)
                continue
            converted = [round(ptime, 6)]
            for ch in channel_counts:
                converted.append(ch)
                converted.append(round(ch * adc_factor, 9))
            samples[count] = tuple(converted)
            count += 1
        del samples[count:]

    # Start, stop, and process message batches
    def _start_measurements(self):
        self.consecutive_fails = 0
        self.last_error_count = 0
        self._bad_frame_channels.clear()
        # Start bulk reading
        rest_ticks = self.mcu.seconds_to_clock(
            1.0 / (10.0 * self.get_samples_per_second())
        )
        self.query_hx711s_cmd.send([self.oid, rest_ticks])
        logging.info(
            "%s starting '%s' measurements", self.sensor_type, self.name
        )
        # Initialize clock tracking
        self.ffreader.note_start()

    def _finish_measurements(self):
        # don't use serial connection after shutdown
        if self.printer.is_shutdown():
            return
        # Halt bulk reading
        self.query_hx711s_cmd.send_wait_ack([self.oid, 0])
        self.ffreader.note_end()
        logging.info(
            "%s finished '%s' measurements", self.sensor_type, self.name
        )

    def _process_batch(self, eventtime) -> BulkAdcData:
        prev_overflows = self.ffreader.get_last_overflows()
        prev_error_count = self.last_error_count
        prev_bad_frames = self.bad_frame_count
        samples = self.ffreader.pull_samples()
        self._convert_samples(samples)
        overflows = self.ffreader.get_last_overflows() - prev_overflows
        errors = self.last_error_count - prev_error_count
        # Report dropped samples once per batch rather than once each, so a
        # chip failing continuously cannot flood the log
        bad_frames = self.bad_frame_count - prev_bad_frames
        if bad_frames:
            logging.warning(
                "%s: dropped %d bad frame(s) on channel(s) %s",
                self.name, bad_frames, sorted(self._bad_frame_channels),
            )
            self._bad_frame_channels.clear()
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


class HX711S(HX711SBase):
    def __init__(self, config):
        super(HX711S, self).__init__(
            config,
            "hx711s",
            # HX711 sps options
            {80: 80, 10: 10},
            80,
            # HX711 gain/channel options
            {"A-128": 1, "B-32": 2, "A-64": 3},
            "A-128",
        )


class HX717S(HX711SBase):
    def __init__(self, config):
        super(HX717S, self).__init__(
            config,
            "hx717s",
            # HX717 sps options
            {320: 320, 80: 80, 20: 20, 10: 10},
            320,
            # HX717 gain/channel options
            {"A-128": 1, "B-64": 2, "A-64": 3, "B-8": 4},
            "A-128",
        )


HX711S_SENSOR_TYPES = {"hx711s": HX711S, "hx717s": HX717S}
