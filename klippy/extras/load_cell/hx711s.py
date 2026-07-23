# HX711S Multi-Sensor Support
#
# Support for 1-4 HX711 or HX717 ADC chips wired as a multi-channel
# load cell sensor. All chips must share the same RATE pin so they
# update simultaneously.
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import struct
from collections import Counter

from klippy.mcu import MCU

from .. import bulk_sensor
from .interfaces import (
    AdcFault,
    BulkAdcData,
    BulkAdcDataCallback,
    LoadCellSensor,
)

UPDATE_INTERVAL = 0.10
ADC_FACTOR = 1.0 / (1 << 23)

Q_SETTLING = 1 << 0
Q_NOT_READY = 1 << 1
Q_EXTRA_LOW = 1 << 2
Q_POST_READ_LOW = 1 << 3
Q_READ_OVERRUN = 1 << 4
Q_SATURATED = 1 << 5
Q_FRAME_FORMAT = 1 << 6
Q_CHANNEL_SHIFT = 8
Q_CHANNEL_MASK = 0xF << Q_CHANNEL_SHIFT
FRAME_TAG = 0xA7110000
FRAME_TAG_MASK = 0xFFFF0000
QUALITY_FLAGS = (
    (Q_SETTLING, "settling"),
    (Q_NOT_READY, "not_ready"),
    (Q_EXTRA_LOW, "extra_low"),
    (Q_POST_READ_LOW, "post_read_low"),
    (Q_READ_OVERRUN, "read_overrun"),
    (Q_SATURATED, "saturated"),
    (Q_FRAME_FORMAT, "frame_format"),
)


def quality_is_hard(quality: int) -> bool:
    # Channel bits and unknown future bits are conservatively hard.  The only
    # non-control frame that is expected during successful recovery is a pure
    # settling/qualification frame.
    return bool(quality & ~Q_SETTLING)


def quality_flags(quality: int) -> tuple[str, ...]:
    flags = tuple(name for bit, name in QUALITY_FLAGS if quality & bit)
    known = sum(bit for bit, _name in QUALITY_FLAGS) | Q_CHANNEL_MASK
    unknown = quality & ~known & 0xFFFFFFFF
    if unknown:
        flags += (f"unknown_0x{unknown:x}",)
    return flags


class TimestampedBulkReader:
    """Read records that carry their own MCU capture clock.

    HX711 recovery deliberately creates gaps, so treating record sequence as a
    continuous fixed-frequency sample clock corrupts timestamps around the
    event.  This reader leaves rate estimation out of the control path and
    uses the timestamp embedded in every MCU frame instead.
    """

    def __init__(self, mcu, unpack_fmt):
        self.mcu = mcu
        self.unpack = struct.Struct(unpack_fmt)
        self.bulk_queue = None
        self.query_status_cmd = None
        self.oid = None
        self.last_overflows = 0
        self._mcu_overflows = 0
        self._next_sequence = 0

    def setup_query_command(self, msgformat, oid, cq):
        self.oid = oid
        self.query_status_cmd = self.mcu.lookup_query_command(
            msgformat,
            "sensor_bulk_status oid=%c clock=%u query_ticks=%u"
            " next_sequence=%hu buffered=%u possible_overflows=%hu",
            oid=oid,
            cq=cq,
        )
        self.bulk_queue = bulk_sensor.BulkDataQueue(self.mcu, oid=oid)

    def get_last_overflows(self):
        return self.last_overflows

    def _update_status(self):
        params = self.query_status_cmd.send([self.oid])
        overflows = params["possible_overflows"]
        delta = (overflows - self._mcu_overflows) & 0xFFFF
        self._mcu_overflows = overflows
        self.last_overflows += delta
        return params

    def note_start(self):
        self.bulk_queue.clear_queue()
        self.last_overflows = 0
        params = self.query_status_cmd.send([self.oid])
        self._mcu_overflows = params["possible_overflows"]
        self._next_sequence = params["next_sequence"]

    def note_end(self):
        self.bulk_queue.clear_queue()

    def pull_samples(self):
        self._update_status()
        samples = []
        for params in self.bulk_queue.pull_queue():
            sequence = params["sequence"]
            missing = (sequence - self._next_sequence) & 0xFFFF
            if missing:
                logging.error(
                    "HX711 bulk stream skipped %d message(s)", missing
                )
                self.last_overflows += missing
            self._next_sequence = (sequence + 1) & 0xFFFF
            data = params["data"]
            if len(data) % self.unpack.size:
                logging.error(
                    "HX711 bulk record has %d trailing byte(s)",
                    len(data) % self.unpack.size,
                )
                self.last_overflows += 1
            for offset in range(
                0, len(data) - self.unpack.size + 1, self.unpack.size
            ):
                samples.append(self.unpack.unpack_from(data, offset))
        return samples


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
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.sensor_type = sensor_type
        self.consecutive_fails = 0
        self._health = Counter()
        self._quality_totals = Counter()
        self._channel_fault_totals = Counter()
        self._last_hard_fault = None
        self._last_batch_health = {}
        self._pending_log_flags = Counter()
        self._pending_log_channels = Counter()
        self._pending_log_hard = 0
        self._pending_log_settling = 0
        self._last_fault_log = float("-inf")

        ppins = self.printer.lookup_object("pins")
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

        self.sps = config.getchoice(
            "sample_rate", sample_rate_options, default=default_sample_rate
        )
        self.gain_channel = int(
            config.getchoice("gain", gain_options, default=default_gain)
        )
        self.oid = mcu.create_oid()

        # Bulk sensor setup
        # The final word is MCU-owned frame quality.  Raw counts stay in the
        # stream for diagnosis, but only quality==0 reaches control clients.
        unpack_format = "<I" + ("i" * self.sensor_count) + "I"
        self.bulk_reader = TimestampedBulkReader(mcu, unpack_format)
        self.batch_bulk = bulk_sensor.BatchBulkHelper(
            self.printer,
            self._process_batch,
            self._start_measurements,
            self._finish_measurements,
            UPDATE_INTERVAL,
        )

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
        mcu.register_config_callback(self._build_config)

    def _build_config(self):
        self.query_hx711s_cmd = self.mcu.lookup_command(
            "query_hx711s oid=%c rest_ticks=%u"
        )
        self.attach_probe_cmd = self.mcu.lookup_command(
            "hx711s_attach_load_cell_probe oid=%c load_cell_probe_oid=%c"
        )
        self.bulk_reader.setup_query_command(
            "query_hx711s_status oid=%c",
            oid=self.oid,
            cq=self.mcu.alloc_command_queue(),
        )

    def get_mcu(self) -> MCU:
        return self.mcu

    def get_samples_per_second(self) -> int:
        return self.sps

    def get_range(self) -> tuple[int, int]:
        return -0x800000, 0x7FFFFF

    def get_channel_count(self) -> int:
        return self.sensor_count

    def get_health(self):
        return {
            "frames": dict(self._health),
            "quality_flags": dict(self._quality_totals),
            "channel_faults": {
                str(channel): count
                for channel, count in self._channel_fault_totals.items()
            },
            "last_hard_fault": self._last_hard_fault,
            "last_batch": self._last_batch_health,
        }

    def add_client(self, callback: BulkAdcDataCallback):
        self.batch_bulk.add_client(callback)

    def attach_load_cell_probe(self, load_cell_probe_oid: int):
        self.attach_probe_cmd.send([self.oid, load_cell_probe_oid])

    def _convert_samples(self, samples):
        count = 0
        faults: list[AdcFault] = []
        for sample in samples:
            capture_clock = self.mcu.clock32_to_clock64(sample[0])
            ptime = self.mcu.clock_to_print_time(capture_clock)
            channel_counts = tuple(sample[1 : 1 + self.sensor_count])
            wire_quality = sample[1 + self.sensor_count]
            if wire_quality & FRAME_TAG_MASK != FRAME_TAG:
                quality = Q_FRAME_FORMAT
            else:
                quality = wire_quality & ~FRAME_TAG_MASK
            self._health["received"] += 1
            if quality:
                channels = tuple(
                    channel
                    for channel in range(self.sensor_count)
                    if quality & (1 << (Q_CHANNEL_SHIFT + channel))
                )
                flags = quality_flags(quality)
                hard = quality_is_hard(quality)
                fault: AdcFault = {
                    "time": round(ptime, 6),
                    "counts": channel_counts,
                    "quality": quality,
                    "wire_quality": wire_quality,
                    "flags": flags,
                    "channels": channels,
                    "hard": hard,
                }
                faults.append(fault)
                self._health["fault"] += 1
                self._health["hard" if hard else "settling"] += 1
                for flag in flags:
                    self._quality_totals[flag] += 1
                for channel in channels:
                    self._channel_fault_totals[channel] += 1
                if hard:
                    self._last_hard_fault = fault
                continue
            self._health["valid"] += 1
            converted = [round(ptime, 6)]
            for ch in channel_counts:
                converted.append(ch)
                converted.append(round(ch * ADC_FACTOR, 9))
            samples[count] = tuple(converted)
            count += 1
        del samples[count:]
        return faults

    def _log_faults(self, eventtime, faults):
        if not any(fault["hard"] for fault in faults):
            return
        for fault in faults:
            for flag in fault["flags"]:
                self._pending_log_flags[flag] += 1
            for channel in fault["channels"]:
                self._pending_log_channels[channel] += 1
            if fault["hard"]:
                self._pending_log_hard += 1
            else:
                self._pending_log_settling += 1
        if not self._pending_log_hard or eventtime < self._last_fault_log + 1.0:
            return
        flag_summary = ", ".join(
            f"{name}={count}"
            for name, count in sorted(self._pending_log_flags.items())
        )
        channel_summary = (
            ", ".join(
                f"ch{channel}={count}"
                for channel, count in sorted(self._pending_log_channels.items())
            )
            or "none"
        )
        logging.warning(
            "%s: HX711 faults: hard=%d recovery=%d; flags=[%s]; channels=[%s]",
            self.name,
            self._pending_log_hard,
            self._pending_log_settling,
            flag_summary,
            channel_summary,
        )
        self._pending_log_flags.clear()
        self._pending_log_channels.clear()
        self._pending_log_hard = self._pending_log_settling = 0
        self._last_fault_log = eventtime

    def _start_measurements(self):
        self.consecutive_fails = 0
        rest_ticks = self.mcu.seconds_to_clock(
            1.0 / (10.0 * self.get_samples_per_second())
        )
        self.query_hx711s_cmd.send([self.oid, rest_ticks])
        logging.info(
            "%s starting '%s' measurements", self.sensor_type, self.name
        )
        self.bulk_reader.note_start()

    def _finish_measurements(self):
        if self.printer.is_shutdown():
            return
        self.query_hx711s_cmd.send_wait_ack([self.oid, 0])
        self.bulk_reader.note_end()
        logging.info(
            "%s finished '%s' measurements", self.sensor_type, self.name
        )

    def _process_batch(self, eventtime) -> BulkAdcData:
        prev_overflows = self.bulk_reader.get_last_overflows()
        samples = self.bulk_reader.pull_samples()
        faults = self._convert_samples(samples)
        overflows = self.bulk_reader.get_last_overflows() - prev_overflows
        errors = sum(fault["hard"] for fault in faults)
        self._log_faults(eventtime, faults)
        if overflows > 0:
            self.consecutive_fails += 1
            if self.consecutive_fails > 4:
                logging.error("%s: repeated bulk overflows", self.name)
        else:
            self.consecutive_fails = 0
        self._last_batch_health = {
            "valid": len(samples),
            "hard_faults": errors,
            "recovery_frames": len(faults) - errors,
            "bulk_overflows": overflows,
        }
        return {
            "data": samples,
            "errors": errors,
            "overflows": overflows,
            "faults": faults,
        }


class HX711S(HX711SBase):
    def __init__(self, config):
        super().__init__(
            config,
            "hx711s",
            {80: 80, 10: 10},
            80,
            {"A-128": 1, "B-32": 2, "A-64": 3},
            "A-128",
        )


class HX717S(HX711SBase):
    def __init__(self, config):
        super().__init__(
            config,
            "hx717s",
            {320: 320, 80: 80, 20: 20, 10: 10},
            320,
            {"A-128": 1, "B-64": 2, "A-64": 3, "B-8": 4},
            "A-128",
        )


HX711S_SENSOR_TYPES = {"hx711s": HX711S, "hx717s": HX717S}
