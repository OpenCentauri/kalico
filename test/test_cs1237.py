from klippy.extras.load_cell.cs1237 import (
    CS1237,
    CS1237_CONFIG_READ,
    CS1237_CONFIG_WRITE,
    DEFAULT_GAIN,
    DEFAULT_REF_OUTPUT_ENABLE,
    DEFAULT_SAMPLE_RATE,
    SAMPLE_ERROR_DESYNC as CS1237_ERROR,
    _GAIN_MAP,
    _SPEED_MAP,
    _make_config_byte,
)


class _Reader:
    def __init__(self, error):
        self.error = error

    def get_last_overflows(self):
        return 0

    def pull_samples(self):
        return [(0.0, self.error)]


class _OverflowReader:
    def __init__(self):
        self.overflows = 0

    def get_last_overflows(self):
        return self.overflows

    def pull_samples(self):
        self.overflows = 1
        return []


class _ConfigCommand:
    def __init__(self, responses):
        self.responses = iter(responses)
        self.calls = []

    def send(self, data):
        self.calls.append(data)
        return next(self.responses)


def test_error_batch_survives_sensor_restart():
    sensor = CS1237.__new__(CS1237)
    setattr(sensor, "ffreader", _Reader(CS1237_ERROR))
    sensor.last_error_count = 0
    sensor.consecutive_fails = 0
    sensor.name = "test"
    sensor._finish_measurements = lambda: None
    sensor._start_measurements = lambda: setattr(sensor, "last_error_count", 0)

    result = sensor._process_batch(0.0)

    assert result["errors"] == 1
    assert result["overflows"] == 0


def test_overflow_batch_survives_sensor_restart():
    sensor = CS1237.__new__(CS1237)
    reader = _OverflowReader()
    setattr(sensor, "ffreader", reader)
    sensor.last_error_count = 0
    sensor.consecutive_fails = 4
    sensor.name = "test"
    sensor._finish_measurements = lambda: None
    sensor._start_measurements = lambda: setattr(reader, "overflows", 0)

    result = sensor._process_batch(0.0)

    assert result["errors"] == 0
    assert result["overflows"] == 1


def test_configure_sensor_retries_until_readback_matches():
    sensor = CS1237.__new__(CS1237)
    sensor.oid = 7
    sensor.config_byte = 0x7C
    command = _ConfigCommand(
        (
            {"success": 0, "value": 0},
            {"success": 1, "value": 0},
            {"success": 1, "value": 0x7C},
        )
    )
    setattr(sensor, "config_cs1237_cmd", command)

    sensor._configure_sensor()

    assert command.calls == [
        [7, CS1237_CONFIG_WRITE, 0x7C],
        [7, CS1237_CONFIG_WRITE, 0x7C],
        [7, CS1237_CONFIG_READ, 0],
    ]


def test_config_byte_disables_refout_for_default_and_scale_rates():
    assert _make_config_byte(
        _SPEED_MAP[DEFAULT_SAMPLE_RATE],
        _GAIN_MAP[DEFAULT_GAIN],
        DEFAULT_REF_OUTPUT_ENABLE,
    ) == 0x7C
    assert _make_config_byte(_SPEED_MAP[40], 3, False) == 0x5C
    assert _make_config_byte(3, 3, True) == 0x3C
