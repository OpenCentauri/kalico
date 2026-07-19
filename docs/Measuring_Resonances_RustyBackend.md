# Using the Rust backend for shaper calibration

Kalico's `[resonance_tester]` supports an optional Rust-based calibration
backend (`rusty`) powered by
[rusty-shaper](https://github.com/OpenCentauri/OpenCentauri/tree/paul/nightly/rusty-shaper).
The Rust backend replaces the numpy shaper-selection path with a compiled
native shared library (`librusty_shaper.so`), which is beneficial on
resource-constrained hosts such as those built around the Allwinner R528
SoC where importing numpy is slow or unavailable.

The standard numpy path is unaffected and remains the default.

## Prerequisites

1. **Build `librusty_shaper.so`** from the
   [OpenCentauri](https://github.com/OpenCentauri/OpenCentauri) repository
   using `compile.sh`. The script uses
   [`cross`](https://github.com/cross-rs/cross) to cross-compile for the
   target architecture (e.g. `armv7-unknown-linux-gnueabihf`) and copies
   both the `rusty-shaper` binary and `librusty_shaper.so` into
   `dist/<target>/`.

   ```bash
   # from the OpenCentauri repo root
   ./rusty-shaper/compile.sh
   # outputs:
   #   dist/armv7-unknown-linux-gnueabihf/rusty-shaper
   #   dist/armv7-unknown-linux-gnueabihf/librusty_shaper.so
   ```

2. **Install the shared library** on the host running Klippy. Place it in
   one of the locations Kalico searches at runtime (checked in order):

   | Path | Notes |
   |------|-------|
   | `<kalico_root>/lib/librusty_shaper.so` | Recommended; keeps it alongside Kalico |
   | Next to `klippy/extras/shaper_calibrate.py` | Useful for dev installs |
   | `/usr/lib/librusty_shaper.so` | System-wide |
   | `/usr/local/lib/librusty_shaper.so` | System-wide alternate |

   Example for a typical Kalico install:
   ```bash
   mkdir -p ~/kalico/lib
   cp dist/armv7-unknown-linux-gnueabihf/librusty_shaper.so ~/kalico/lib/
   ```

## Configuration

Add `calibration_backend: rusty` to your `[resonance_tester]` section in
`printer.cfg`:

```ini
[resonance_tester]
accel_chip: adxl345
probe_points: 150, 150, 20
calibration_backend: rusty
# Optional: restrict which shapers are evaluated
# calibration_shapers: mzv,ei,2hump_ei
```

With `calibration_backend: rusty` set, Kalico will **not** import numpy
during shaper calibration. The standard `SHAPER_CALIBRATE` GCode command
works identically to the numpy path from the user's perspective.

## GCode usage

The Rust backend is selected automatically when `calibration_backend: rusty`
is set in config. Both parameters can also be changed at runtime without a
restart and persisted with `SAVE_CONFIG`:

```
# Run calibration with the Rust backend, evaluating only mzv and ei:
SHAPER_CALIBRATE AXIS=x BACKEND=rusty SHAPERS=mzv,ei

# Persist the backend choice so it survives restarts:
SAVE_CONFIG
```

### `SHAPER_CALIBRATE` parameters (Kalico additions)

| Parameter | Values | Default | Description |
|-----------|--------|---------|-------------|
| `BACKEND` | `numpy`, `rusty` | value from config | Selects calibration backend for this run and persists via `SAVE_CONFIG` |
| `SHAPERS` | comma-separated shaper names | all shapers | Restricts which shapers are evaluated; persisted via `SAVE_CONFIG` |

## How it works

When `backend=rusty` is active, `ShaperCalibrate._rusty_calibrate()` loads
`librusty_shaper.so` via `ctypes.CDLL` (lazy, load-once per instance) and
calls one of two entry points depending on whether raw accelerometer CSV
or a pre-computed PSD CSV is provided:

- `rusty_shaper_calibrate_from_csv` — accepts a raw accelerometer CSV path
- `rusty_shaper_calibrate_from_psd` — accepts a pre-computed PSD CSV path

Both functions return a JSON string on the heap. Kalico parses the JSON and
frees the string via `rusty_shaper_free_string`. The `.so` itself remains
mapped for the lifetime of the `ShaperCalibrate` instance (typically the
duration of one `SHAPER_CALIBRATE` run), which is negligible in practice
given the library's small size.

## Troubleshooting

**`librusty_shaper.so not found`** — Verify the library exists in one of the
search paths listed above. Check with `ls -la ~/kalico/lib/librusty_shaper.so`.

**`rusty-shaper returned an error`** — Run the `rusty-shaper` CLI binary
directly against your CSV to get the full error message:
```bash
~/kalico/lib/../bin/rusty-shaper --csv /tmp/resonances_x_*.csv
```

**Falling back to numpy** — If you need numpy output (e.g. to generate a
PSD CSV graph), set `BACKEND=numpy` for that run. The Rust backend does not
write PSD CSV files.
