# HX711S Regression-Test Strategy (MCU C + klippy)

Scope: `src/sensor_hx711s.c` + `src/load_cell_probe.c` (kalico branch `hx711s-new3`)
and `klippy/extras/load_cell/hx711s.py` / `load_cell.py` / `tap_analysis.py`.
Goal: CI-runnable (<5 min), no hardware, catches regressions in fault handling,
the spike/impulse filter, and the probe trigger path.

---

## 1. Where the testable seams are (from reading the code)

MCU C, `sensor_hx711s.c` (625 lines). All host-visible behavior funnels through
three sinks, and each is a mockable 1-function seam:

| Seam | What it carries | Mock records |
|---|---|---|
| `sensor_bulk_report(&sb, oid)` | per-chip counts + `SAMPLE_ERROR_*` markers | bulk frames |
| `load_cell_probe_report_sample(lce, sum)` | filtered sum → trigger logic | (ticks, sum) |
| `trsync_do_trigger(ts, reason)` (in `load_cell_probe.c`) | trigger / error events | (ticks, reason) ← assertion point for phantom/latency |

Everything else the driver touches is a small board surface: `gpio_in_read`,
`gpio_out_write/toggle_noirq`, `timer_read_time`, `timer_from_us`,
`timer_is_before`, `sched_add/del_timer`, `sched_wake_task`,
`sched_check_wake`, `irq_*`, `sensor_bulk_reset`, `oid_alloc/lookup`,
`DECL_COMMAND/DECL_TASK`. That is the whole mocking surface (~15 functions).

Driver behavior under test (the assertions map to these exact code paths):
- Torn frame (post-`86cc56a8`): DOUT low after the final clock → discard frame,
  hold last counts, `bad_frame=1`, `bad_streak++`; streak >2 → per-chip
  `hx711s_recover_chip()` (the pre-rebase re-read loop was dropped per review).
- All-high `0xFFFFFFFF` frame: hold last counts, `BAD_FRAME` marker in bulk,
  streak >2 → recover. Never reaches the sum as -1.
- Stuck-low DOUT: frame reads all-zeros → `extras_mask` check fails → **immediate**
  recover (not held).
- Wedged DRDY: no completed read for `stuck_ticks` → inline staleness check in
  `hx711s_event()` wakes the task → recover (primary or secondary chip alike).
- Recovery (post-`86cc56a8`): power-cycle the FAULTED chip only, discard its
  next 4 conversions, next emitted sample marked `SAMPLE_ERROR_RECOVERED`
  (one-shot). Recovering chip is excused from the emission gate via
  `recovering_mask`; healthy chips keep streaming live. `recovery_streak`
  resets only when **every** chip has requalified (`have_counts == chip_mask`).
- Fatal: `recovery_streak > 3` OR a second chip faulting while one requalifies
  → latched `SAMPLE_ERROR_DESYNC`, stream stops, host restarts. These are the
  ONLY DESYNC paths.
- Spike filter (`report_sum`): single-sample sum jump > `spike_sum_threshold`
  (default 5775 counts ≈ 55 g) is held one sample; sign-aware confirm forwards
  the pending sample (real ramps, +1 sample latency) or drops it (impulses).

klippy Python, `load_cell/hx711s.py`: `_convert_samples()` handles the
`SAMPLE_ERROR_*` markers (DESYNC latches+breaks, RECOVERED counts+continues,
BAD_FRAME drops per-channel, adc_factor conversion). `_process_batch()` restarts
the sensor on errors / >4 consecutive overflow batches.

---

## 2. Running MCU C in CI without hardware — decision

Evaluated:

1. **kalico `simulator` target** (`src/simulator/`, 232 lines): builds the MCU
   for the host, but `gpio.c` is a stub — `gpio_in_read` returns a fixed value,
   there is no chip model and no way to script DOUT waveforms. Useful as a
   compile/link smoke test only; extending it = building the same harness as
   option 2 but inside the firmware tree. Rejected as test vehicle.
2. **Unit harness compiling `sensor_hx711s.c` + `load_cell_probe.c` against
   mocked board headers** — small `mock/` include dir shadowing `board/*`,
   `sched.h`, `command.h`, etc., plus a bit-level HX711 chip model. Tests the
   exact shipped C: bit-bang re-read loop, sign extension, every fault branch,
   the spike filter, and the real trigger latch in `load_cell_probe.c`.
   Deterministic virtual time; 197k samples run in seconds.
3. **Python golden model**: re-implement `report_sum()` (~40 lines) and replay
   the same captures. Cheap, but tests a *copy* — silently diverges the day
   someone edits the C. Not a primary.

**Decision: option 2 primary. Option 3 fallback** — kept honest by diffing its
output against the C harness on the same corpus (same first-crossing indices,
same forwarded samples); it exists so klippy-side pytest can iterate on filter
parameters without a C build.

The bit-level chip model is required anyway for torn-frame injection, so
replaying real captures goes through it too: real captures store post-decode
counts; the model re-encodes counts → 24+gain bits and drives DOUT per SCLK
edge. This exercises `hx711s_raw_read` exactly as hardware does.

---

## 3. Capture/replay data format

Compact gzipped JSONL, one file per capture (`.cap.jsonl.gz`):

```json
{"format":"hx711s-cap/1","sps":80,"chips":4,"counts_per_gram":105,
 "trigger_grams":75,"source":"hx711s-new2-artifacts/force.jsonl.gz",
 "labels":{"phantom_zones":[[t0,t1],...],"tap_windows":[[t0,t1],...]}}
[583.746012, 371384, 284000, 168495, -115042]
[583.757570, 371276, 284000, 168703, -115277]
...
```

- Header line: metadata + labels. Labels are **time ranges**, not per-sample
  flags — `phantom_zones` = samples where an unfiltered first-crossing trigger
  would have fired (known impulses), `tap_windows` = windows containing a real
  tap that must trigger.
- Sample lines: `[t_seconds, counts_chip0..3]` (raw 24-bit signed counts).
  197k samples ≈ 5–8 MB gz. Committed to the repo (or LFS / CI cache if size
  becomes a problem); the 43 MB raw archive stays in `hx711s-new3-artifacts`.
- Converter: `test/hx711s/tools/convert_force_jsonl.py` (~60 lines) — reads
  MESH_SOAK/`dump_force` `force.jsonl(.gz)` (rows of `[t, g0, c0, ...]`), picks
  each chip's counts column, emits `.cap.jsonl.gz`. Labels for the 197k set are
  transcribed once from the new2 REPORT (3 impulses ≥75 g at known t, 20 ≥60 g,
  465 tap windows from the 5 mesh runs) into the header; new campaign captures
  get labels from MESH_SOAK's tap logs. New captures drop in `captures/` and
  are picked up by glob — no test code changes.

---

## 4. Harness design (MCU side)

```
test/hx711s/
├── mcu_harness/
│   ├── Makefile              # gcc sensor_hx711s.c load_cell_probe.c harness.c chip_model.c -Imock
│   ├── mock/                 # shadows firmware includes
│   │   ├── autoconf.h        # CONFIG_MACH_AVR=0 etc.
│   │   ├── board/{gpio,irq,misc}.h   # → chip model + virtual time
│   │   ├── sched.h, command.h, basecmd.h, trsync.h, sensor_bulk.h
│   │   └── load_cell_probe.h         # real header
│   ├── chip_model.{c,h}      # 4 virtual HX711s: playback + fault ops
│   └── harness.c             # main: load case, run, assert, write report
├── cases/*.json              # test-case definitions (see §6)
├── captures/*.cap.jsonl.gz   # replay corpora
├── klippy/                   # stage 3 pytest
├── tools/convert_force_jsonl.py
└── reports/                  # CI artifacts (gitignored)
```

Harness mechanics:
- **Virtual clock**: `timer_read_time()` returns a harness-advanced tick
  counter. Main loop: pop earliest chip timer → advance time → run
  `hx711s_event()` → run `hx711s_capture_task()` → repeat until the script is
  exhausted. No real-time sleeps (`hx711s_delay*` no-op via `CONFIG_MACH_AVR=1`
  path or a time-advancing stub; recover's 100 µs spin is virtual).
- **Chip model**: per chip a queue of frames. Clean frame = `{counts}` → DOUT
  low when polled, bits shifted out on SCLK edges, DOUT high after final clock.
  Fault ops: `TORN(k)` (re-latch DOUT low after the read k times), `ALL_HIGH`
  (DOUT high through the whole frame), `STUCK_LOW` (DOUT permanently low),
  `WEDGE(n)` (DOUT high at poll for n conversion periods — never ready).
  Scripted per chip, so two-chip faults are two scripts overlapping in time.
- **Recorded sinks**: bulk frames (for `BAD_FRAME`/`RECOVERED`/`DESYNC` marker
  assertions), probe-feed samples (for sum-path assertions), trigger events
  `(ticks, reason)` from the `trsync_do_trigger` stub (for phantom/latency).
- **Config**: harness calls the real `command_config_hx711s` / `command_add_hx711s`
  / `hx711s_set_tuning` / `command_query_hx711s` arg paths, and
  `hx711s_attach_load_cell_probe`, so config validation runs too.
- **Latency metric**: sample index of the real trigger event minus sample index
  of first crossing in the unfiltered sum (recomputed by the harness from the
  same capture). Expected: exactly +1 on taps that hit the hold path, 0
  otherwise; histogram bins `[0, 1, 2, >2]`.

---

## 5. Fault-injection matrix (expected outcomes)

| # | Fault | Expected MCU outcome | Asserts on |
|---|---|---|---|
| F1 | torn frame, single | discard: hold last counts, `BAD_FRAME` in bulk, streak=1, no recovery | sum unchanged; counts not updated from torn frame |
| F2 | torn frames, streak ≤2 | hold last counts each, `BAD_FRAME` per frame, no recovery | bulk markers; sum frozen on last good |
| F3 | torn/all-high streak = 3 | per-chip recover: power-cycle THAT chip only, 4 discards on it, one `RECOVERED` sample, healthy chips keep streaming | exactly 1 RECOVERED; recovering_chip excluded from emission gate until requalified; healthy chips' counts keep flowing |
| F4 | all-high single frame | hold, `BAD_FRAME`, never −1 counts in sum | sum continuous; no trigger |
| F5 | stuck-low DOUT | immediate recover (extras check), RECOVERED; sustained → DESYNC after 4th recovery | no hold path; DESYNC only at streak>3 |
| F6 | wedged DRDY (each chip, incl. secondary) | watchdog recover after stuck_ticks; RECOVERED | recovery latency ≤ stuck_ticks + settle |
| F7 | impulse, 1 sample, ±(55–220 g), both signs, random phase | held then dropped: **0 trigger events**; impulse still visible in bulk | trigger count; bulk content |
| F8 | impulse 2 samples / sustained real ramp | confirmed, forwarded, trigger at +1 sample | latency histogram all ∈ {0,1} |
| F9 | baseline wander ±30 g, slow | no holds (per-sample Δ < threshold), no triggers below trigger_force | 0 triggers; sum tracks capture |
| F10 | two chips fault simultaneously (2nd faults while 1st requalifying) | latched `SAMPLE_ERROR_DESYNC` immediately (systemic fault, not two independent ones) | DESYNC on 2nd fault; no power-cycle of 2nd chip |
| F11 | persistent dead chip (never requalifies) | recover ×3 without a good frame → latched `SAMPLE_ERROR_DESYNC` on 4th attempt, emission stops | DESYNC at exactly streak 4, not before |

Rule encoded in every case: **DESYNC appears only on recovery_streak > 3 or a
second chip faulting mid-requalification** (and the host-side restart that
follows). Any other DESYNC = fail.

Vendor cross-check: Elegoo's stock bed-MCU firmware (`CentauriCarbon/mcu/src/hx711s.c`)
rejects the same fault classes via `sliding_window_avg_exception_filter` (stream),
`median_filter` (raw frames), and `check_tregger` shape validation (last-3 monotonic,
last-3 largest, slope >40°, `enable_shake_filter` for hand-tap/table vibration).
Their trigger never trusts single samples; F7/F8 assertions encode the same property
for the kalico architecture. See `../HX711-20260729.md` "Elegoo stock firmware analysis".

Replay case (R1, the 197k set): expected 0 trigger events inside
`phantom_zones`, ≥1 trigger inside every `tap_window`, trigger latency
distribution uniform +1, 0 RECOVERED, 0 DESYNC. (Matches the known offline
result: 24/24 sub-50ms noise crossings removed incl. the 3 trigger-class
impulses, 465/465 real taps still trigger.)

---

## 6. Sample test-case definitions

Cases are JSON; the harness globs `cases/*.json`. Three representative ones:

```json
// cases/replay_new2_197k.json
{"name": "replay_new2_197k",
 "type": "replay",
 "capture": "captures/hx711s-new2-197k.cap.jsonl.gz",
 "config": {"chips": 4, "sps": 80, "gain_channel": 1,
            "stuck_ms": 36, "settle_ms": 60,
            "spike_sum_threshold": 5775, "trigger_grams": 75},
 "expect": {"phantom_triggers": 0,
            "tap_windows_triggered": "all",
            "latency_hist": {"0": "any", "1": "any", "2": 0, ">2": 0},
            "recovered_count": 0, "desync": 0}}
```

```json
// cases/fault_impulse_single.json
{"name": "fault_impulse_single",
 "type": "synthetic",
 "config": {"chips": 4, "sps": 80, "spike_sum_threshold": 5775,
            "trigger_grams": 75},
 "stream": {"baseline_counts": 0,
            "events": [
              {"at_s": 1.0, "op": "IMPULSE", "chip_sum_delta": 7875,
               "width_samples": 1, "sign": +1},
              {"at_s": 2.0, "op": "IMPULSE", "chip_sum_delta": 23100,
               "width_samples": 1, "sign": -1},
              {"at_s": 3.0, "op": "RAMP", "chip_sum_delta": 9000,
               "width_samples": 30}]},
 "expect": {"triggers_before_s": {"t": 3.0, "count": 0},
            "trigger_in_window_s": [3.0, 3.5],
            "latency_samples": [0, 1],
            "recovered_count": 0, "desync": 0}}
```

```json
// cases/fault_two_chip_desync.json
{"name": "fault_two_chip_desync",
 "type": "synthetic",
 "config": {"chips": 4, "sps": 80, "stuck_ms": 36, "settle_ms": 60},
 "stream": {"baseline_counts": 1000,
            "events": [
              {"at_s": 0.5, "chip": 1, "op": "ALL_HIGH", "count": 3},
              {"at_s": 0.5, "chip": 2, "op": "WEDGE", "periods": 10}]},
 "expect": {"recovered_count": 1, "desync": 1,
            "bad_frame_chips": [1],
            "comment": "chip1 recovers; chip2 faults while chip1 requalifies -> hard fault",
            "triggers": 0}}
```

(`IMPULSE`/`RAMP` distribute a sum delta across chips as equal counts deltas;
the generator in the harness expands these ops into per-chip frame scripts.)

---

## 7. klippy-side tests (pytest, stage 3)

`test/hx711s/klippy/`, run by the existing `py.test -n auto` job
(pyproject `testpaths = ["test"]` picks it up):

- `test_hx711s_convert.py` — instantiate `HX711SBase` via `__new__` (skip
  config/pins — only counters are state), feed hand-built sample tuples through
  `_convert_samples`: DESYNC latches `last_error_count` and truncates the
  batch; RECOVERED increments `recovered_count` and the rest of the batch is
  still converted; per-channel BAD_FRAME rows dropped with `bad_frame_count`
  and channel set correct; normal rows get `adc_factor` floats appended.
- `test_spike_filter_golden.py` — Python golden model of `report_sum()`,
  replays every `captures/*.cap.jsonl.gz`, asserts first-crossing indices equal
  the C harness's recorded trigger events (cross-check keeps the golden model
  honest) and 0 crossings in phantom zones.
- `test_process_batch_restart.py` — `_process_batch` with a stub `ffreader`
  (~20-line fake): errors>0 → finish+start called; 5 consecutive overflow
  batches → restart; clean batch resets the streak.
- Optional (only if cheap): feed one captured tap window through
  `tap_analysis.py` and assert it validates. Skip if it drags in the full
  printer object graph.

Full end-to-end via `test/klippy` (klippy_testing harness) is **not** required
— the MCU probe trigger is already covered by the C harness, and wiring a
virtual MCU + real klippy costs more than it catches.

---

## 8. CI integration (kalico PR CI)

New job in `.github/workflows/ci-build_test.yaml` (reuses the pulled
`dangerklippers/klipper-build` image — has gcc, pytest, xdist):

```yaml
hx711s-regression:
  runs-on: ubuntu-latest
  steps:
    - uses: actions/checkout@v4
    - run: docker run -v $PWD:/klipper dangerklippers/klipper-build:latest \
           sh -c "make -C test/hx711s/mcu_harness run && py.test test/hx711s/klippy -q"
    - uses: actions/upload-artifact@v4
      if: always()
      with: {name: hx711s-regression, path: test/hx711s/reports/}
```

Runtime budget: harness build ~5 s, 197k-sample replay + ~15 fault cases
<30 s, pytest <1 min, docker pull cached → **well under 5 min**.
Artifacts (`reports/`): `results.json` (per-case pass/fail + assertion detail),
`latency_<case>.json` (histogram bins + per-tap latencies), and on failure the
recorded bulk/trigger streams for the failing case. Path-filter: run on
`src/sensor_hx711s.c`, `src/load_cell_probe.*`, `klippy/extras/load_cell/**`,
`test/hx711s/**` changes.

---

## 9. Stages

**Stage 1 — MVP (replay-only MCU harness).**
mock/ dir, chip model (clean playback + IMPULSE only), harness main loop,
converter, the 197k capture + labels, cases R1 + F7. Exit criterion: R1
reproduces the known result (0 phantom triggers, all taps, latency all +1)
against the shipped C. ~2–3 days, no klippy changes.

**Stage 2 — fault injection.**
TORN/ALL_HIGH/STUCK_LOW/WEDGE ops, per-chip scripting, F1–F11 cases, Python
golden model + cross-check test. Exit criterion: full §5 matrix green,
including DESYNC-only-at-streak-4.

**Stage 3 — klippy-side + CI wiring.**
§7 pytest files, workflow job, artifact upload, path filters. New captures
(MESH_SOAK campaigns) get converted and dropped into `captures/` with labels —
automatically picked up by R1-style glob cases.

Deliberately skipped: simulator-target extension, end-to-end klippy_testing
runs, per-sample label flags (time ranges suffice), property-based/fuzz
generation (add only if the matrix misses a real field failure).
