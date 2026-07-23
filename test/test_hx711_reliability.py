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
from klippy.extras.probe import PrinterProbe


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


class ProbeCommandError(Exception):
    pass


class FakeProbePrinter:
    command_error = ProbeCommandError


class FakeGcmd:
    def __init__(self):
        self.messages = []

    def respond_info(self, message):
        self.messages.append(message)

    def error(self, message):
        return ProbeCommandError(message)


class FakeProbeRetrySession:
    def __init__(self):
        self.positions = [(10.0, 20.0, None), (10.0, 20.0, None)]
        self.evaluated = []

    def can_retry(self):
        return bool(self.positions)

    def get_probe_position(self):
        return self.positions.pop(0)

    def evaluate_probe(self, is_good):
        self.evaluated.append(is_good)
        return is_good

    def get_position(self):
        return (10.0, 20.0)

    def get_bad_probe_count(self):
        return len(self.evaluated)

    def scrub_nozzle(self):
        pass


def make_probe_runner(outcomes):
    probe = PrinterProbe.__new__(PrinterProbe)
    probe.printer = FakeProbePrinter()
    probe.retry_speed = 7.0
    probe.moves = []
    probe.retracts = 0
    outcomes = list(outcomes)

    def move(pos, speed):
        probe.moves.append((pos, speed))

    def retract(_gcmd):
        probe.retracts += 1

    def do_probe(_speed, _gcmd):
        outcome = outcomes.pop(0)
        if isinstance(outcome, Exception):
            raise outcome
        return outcome

    probe._move = move
    probe._retract = retract
    probe._probe = do_probe
    return probe


def test_probe_retries_one_invalid_hx711_sample():
    gcmd = FakeGcmd()
    retry_session = FakeProbeRetrySession()
    probe = make_probe_runner(
        [
            ProbeCommandError(
                "Load Cell Probe Error: invalid HX711 sample; "
                "see sensor fault diagnostics"
            ),
            ([10.0, 20.0, -0.42], True),
        ]
    )

    result = probe._run_probe_with_retries(5.0, retry_session, gcmd)

    assert result == [10.0, 20.0, -0.42]
    assert probe.retracts == 1
    assert len(probe.moves) == 2
    assert retry_session.evaluated == [True]
    assert gcmd.messages == ["HX711 invalid sample detected. Retrying..."]


def test_probe_does_not_retry_repeated_invalid_hx711_samples():
    gcmd = FakeGcmd()
    retry_session = FakeProbeRetrySession()
    probe = make_probe_runner(
        [
            ProbeCommandError("Load Cell Probe Error: invalid HX711 sample"),
            ProbeCommandError("Load Cell Probe Error: invalid HX711 sample"),
        ]
    )

    try:
        probe._run_probe_with_retries(5.0, retry_session, gcmd)
    except ProbeCommandError:
        pass
    else:
        assert False, "second invalid HX711 sample should remain a hard failure"

    assert probe.retracts == 1
    assert len(probe.moves) == 2
    assert retry_session.evaluated == []


def test_probe_does_not_retry_hard_load_cell_safety_error():
    gcmd = FakeGcmd()
    retry_session = FakeProbeRetrySession()
    probe = make_probe_runner(
        [
            ProbeCommandError(
                "Load Cell Probe Error: force exceeded drift_safety_limit "
                "before triggering!"
            )
        ]
    )

    try:
        probe._run_probe_with_retries(5.0, retry_session, gcmd)
    except ProbeCommandError:
        pass
    else:
        assert False, "safety failures must not be retried as transient samples"

    assert probe.retracts == 0
    assert len(probe.moves) == 1
    assert retry_session.evaluated == []
