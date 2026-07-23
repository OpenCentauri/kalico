import struct
from collections import Counter

from klippy.extras.load_cell.hx711s import (
    FRAME_TAG,
    Q_CHANNEL_SHIFT,
    Q_EXTRA_LOW,
    Q_FRAME_FORMAT,
    Q_NOT_READY,
    Q_SETTLING,
    HX711SBase,
    TimestampedBulkReader,
    quality_flags,
    quality_is_hard,
)
from klippy.extras.load_cell.load_cell import LoadCellSampleCollector


class FakeMcu:
    def clock32_to_clock64(self, clock):
        return clock

    def clock_to_print_time(self, clock):
        return clock / 1000.0


class FakeStatusCommand:
    def __init__(self, overflows=0):
        self.overflows = overflows

    def send(self, _args):
        return {"possible_overflows": self.overflows, "next_sequence": 0}


class FakeBulkQueue:
    def __init__(self, messages):
        self.messages = messages

    def pull_queue(self):
        messages, self.messages = self.messages, []
        return messages

    def clear_queue(self):
        self.messages = []


def make_hx711(sensor_count=2):
    sensor = HX711SBase.__new__(HX711SBase)
    sensor.sensor_count = sensor_count
    sensor.mcu = FakeMcu()
    sensor._health = Counter()
    sensor._quality_totals = Counter()
    sensor._channel_fault_totals = Counter()
    sensor._last_hard_fault = None
    return sensor


def test_quality_classification():
    assert not quality_is_hard(Q_SETTLING)
    assert quality_is_hard(Q_SETTLING | Q_NOT_READY)
    assert quality_is_hard(1 << Q_CHANNEL_SHIFT)
    assert quality_flags(Q_SETTLING | Q_EXTRA_LOW) == (
        "settling",
        "extra_low",
    )


def test_timestamped_conversion_keeps_fault_details_out_of_control_data():
    sensor = make_hx711()
    samples = [
        (1000, 100, 200, FRAME_TAG | Q_SETTLING),
        (
            2000,
            110,
            210,
            FRAME_TAG | Q_NOT_READY | (1 << (Q_CHANNEL_SHIFT + 1)),
        ),
        (3000, 120, 220, FRAME_TAG),
    ]

    faults = sensor._convert_samples(samples)

    assert len(faults) == 2
    assert faults[0]["time"] == 1.0
    assert not faults[0]["hard"]
    assert faults[1]["time"] == 2.0
    assert faults[1]["hard"]
    assert faults[1]["channels"] == (1,)
    assert faults[1]["flags"] == ("not_ready",)
    assert samples[0][0] == 3.0
    assert samples[0][1] == 120
    assert samples[0][3] == 220
    assert sensor._health == {
        "received": 3,
        "fault": 2,
        "settling": 1,
        "hard": 1,
        "valid": 1,
    }
    assert sensor._last_hard_fault == faults[1]


def test_untagged_wire_format_fails_closed():
    sensor = make_hx711()
    samples = [(1000, 100, 200, 0)]

    faults = sensor._convert_samples(samples)

    assert samples == []
    assert len(faults) == 1
    assert faults[0]["hard"]
    assert faults[0]["quality"] == Q_FRAME_FORMAT
    assert faults[0]["flags"] == ("frame_format",)


def test_timestamped_reader_preserves_real_gaps_between_capture_clocks():
    reader = TimestampedBulkReader(FakeMcu(), "<IiI")
    reader.oid = 1
    reader.query_status_cmd = FakeStatusCommand()
    reader.bulk_queue = FakeBulkQueue(
        [
            {
                "sequence": 0,
                "data": struct.pack("<IiI", 100, 10, 0)
                + struct.pack("<IiI", 9100, 20, 0),
            }
        ]
    )

    samples = reader.pull_samples()

    assert samples == [(100, 10, 0), (9100, 20, 0)]


class FakeReactor:
    pass


class FakePrinter:
    def get_reactor(self):
        return FakeReactor()


class FakeCollectorSensor:
    def get_mcu(self):
        return FakeMcu()


class FakeLoadCell:
    sensor = FakeCollectorSensor()

    def add_client(self, callback):
        self.callback = callback


def fault(time, hard):
    return {
        "time": time,
        "counts": (0, 0),
        "quality": Q_NOT_READY if hard else Q_SETTLING,
        "wire_quality": FRAME_TAG | (Q_NOT_READY if hard else Q_SETTLING),
        "flags": ("not_ready",) if hard else ("settling",),
        "channels": (),
        "hard": hard,
    }


def test_collector_filters_faults_to_requested_time_window():
    collector = LoadCellSampleCollector(FakePrinter(), FakeLoadCell())
    collector.min_time = 10.0
    collector.max_time = 20.0
    collector.is_started = True

    keep_running = collector._on_samples(
        {
            "data": [(11.0,), (21.0,)],
            "errors": 3,
            "overflows": 0,
            "faults": [
                fault(9.0, True),
                fault(12.0, False),
                fault(14.0, True),
                fault(21.0, True),
            ],
        }
    )
    samples, errors = collector._finish_collecting()

    assert not keep_running
    assert samples == [(11.0,)]
    # Only the hard fault inside [min_time, max_time] fails this collection.
    assert errors == (1, 0)


def test_collector_preserves_legacy_batch_errors_without_fault_records():
    collector = LoadCellSampleCollector(FakePrinter(), FakeLoadCell())
    collector.is_started = True

    collector._on_samples(
        {"data": [(1.0,)], "errors": 2, "overflows": 1, "faults": []}
    )

    _samples, errors = collector._finish_collecting()
    assert errors == (2, 1)
