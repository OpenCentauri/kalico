# Automatic calibration of input shapers
#
# Copyright (C) 2020-2024  Dmitry Butyugin <dmbutyugin@google.com>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import collections
import ctypes
import importlib
import json
import math
import multiprocessing
import os
import traceback

from . import shaper_defs

MIN_FREQ = 5.0
MAX_FREQ = 200.0
WINDOW_T_SEC = 0.5
MAX_SHAPER_FREQ = 150.0

TEST_DAMPING_RATIOS = [0.075, 0.1, 0.15]

AUTOTUNE_SHAPERS = ["zv", "mzv", "ei", "2hump_ei", "3hump_ei"]

# Default search path for the Rust shared library.  The installer copies
# librusty_shaper.so next to Klippy's extras directory; we also check /usr/lib
# and the directory of this file as fallbacks.
_RUSTY_SO_SEARCH = [
    os.path.join(os.path.dirname(__file__), "..", "..", "lib", "librusty_shaper.so"),
    os.path.join(os.path.dirname(__file__), "librusty_shaper.so"),
    "/usr/lib/librusty_shaper.so",
    "/usr/local/lib/librusty_shaper.so",
]


def _find_rusty_so():
    """Return the first librusty_shaper.so path that exists, or None."""
    for candidate in _RUSTY_SO_SEARCH:
        if os.path.exists(os.path.realpath(candidate)):
            return os.path.realpath(candidate)
    return None


######################################################################
# Frequency response calculation and shaper auto-tuning
######################################################################


class CalibrationData:
    def __init__(self, freq_bins, psd_sum, psd_x, psd_y, psd_z):
        self.freq_bins = freq_bins
        self.psd_sum = psd_sum
        self.psd_x = psd_x
        self.psd_y = psd_y
        self.psd_z = psd_z
        self._psd_list = [self.psd_sum, self.psd_x, self.psd_y, self.psd_z]
        self._psd_map = {
            "x": self.psd_x,
            "y": self.psd_y,
            "z": self.psd_z,
            "all": self.psd_sum,
        }
        self.data_sets = 1

    def add_data(self, other):
        np = self.numpy
        joined_data_sets = self.data_sets + other.data_sets
        for psd, other_psd in zip(self._psd_list, other._psd_list):
            other_normalized = other.data_sets * np.interp(
                self.freq_bins, other.freq_bins, other_psd
            )
            psd *= self.data_sets
            psd[:] = (psd + other_normalized) * (1.0 / joined_data_sets)
        self.data_sets = joined_data_sets

    def set_numpy(self, numpy):
        self.numpy = numpy

    def normalize_to_frequencies(self):
        for psd in self._psd_list:
            psd /= self.freq_bins + 0.1
            low_freqs = self.freq_bins < 2.0 * MIN_FREQ
            psd[low_freqs] *= self.numpy.exp(
                -((2.0 * MIN_FREQ / (self.freq_bins[low_freqs] + 0.1)) ** 2)
                + 1.0
            )

    def get_psd(self, axis="all"):
        return self._psd_map[axis]


CalibrationResult = collections.namedtuple(
    "CalibrationResult",
    ("name", "freq", "vals", "vibrs", "smoothing", "score", "max_accel"),
)


class ShaperCalibrate:
    def __init__(self, printer, backend="numpy"):
        self.printer = printer
        self.error = printer.command_error if printer else Exception
        self.backend = backend.lower()
        if self.backend not in ("numpy", "rusty"):
            raise self.error(
                "calibration_backend must be 'numpy' or 'rusty', got '%s'"
                % (self.backend,)
            )
        # numpy is loaded lazily for the rusty path so we can skip the
        # import entirely when memory is tight.
        self.numpy = None
        self._rusty_lib = None
        if self.backend == "numpy":
            try:
                self.numpy = importlib.import_module("numpy")
            except ImportError:
                raise self.error(
                    "Failed to import `numpy` module, make sure it was "
                    "installed via `~/klippy-env/bin/pip install` (refer to "
                    "docs/Measuring_Resonances.md for more details)."
                )

    # ────────────────────────────────────────────────────────────
    # Rusty shared-library helpers
    # ────────────────────────────────────────────────────────────

    def _load_rusty_lib(self):
        """Load librusty_shaper.so (once) and configure ctypes signatures."""
        if self._rusty_lib is not None:
            return self._rusty_lib
        so_path = _find_rusty_so()
        if so_path is None:
            raise self.error(
                "librusty_shaper.so not found. Install it to one of: %s"
                % ", ".join(_RUSTY_SO_SEARCH)
            )
        lib = ctypes.CDLL(so_path)
        c_str = ctypes.c_char_p
        c_dbl = ctypes.c_double
        c_void_p = ctypes.c_void_p

        # rusty_shaper_calibrate_from_csv
        lib.rusty_shaper_calibrate_from_csv.argtypes = [
            c_str,   # csv_path
            c_str,   # shapers_csv  (nullable)
            c_dbl,   # damping_ratio
            c_str,   # test_dr_csv  (nullable)
            c_dbl,   # scv
            c_dbl,   # max_smoothing  (<=0 -> unconstrained)
            c_dbl,   # max_freq
            c_str,   # freq_range_csv  (nullable, "start:end:step")
            c_dbl,   # window_t
        ]
        lib.rusty_shaper_calibrate_from_csv.restype = ctypes.c_void_p

        # rusty_shaper_calibrate_from_psd
        lib.rusty_shaper_calibrate_from_psd.argtypes = [
            c_str, c_str, c_dbl, c_str, c_dbl, c_dbl, c_dbl, c_str,
        ]
        lib.rusty_shaper_calibrate_from_psd.restype = ctypes.c_void_p

        # rusty_shaper_free_string
        lib.rusty_shaper_free_string.argtypes = [c_void_p]
        lib.rusty_shaper_free_string.restype = None

        self._rusty_lib = lib
        return lib

    def _encode(self, s):
        """Encode a Python str to bytes for ctypes, or return None."""
        return s.encode() if s is not None else None

    def _rusty_calibrate(
        self,
        csv_path,
        shapers=None,
        damping_ratio=0.1,
        test_damping_ratios=None,
        scv=5.0,
        max_smoothing=None,
        max_freq=None,
        shaper_freqs=None,
        window_t=None,
        is_raw=True,
    ):
        """
        Call the Rust .so and return a parsed CalibrationOutput dict.

        :param shapers: list of shaper name strings, or None for defaults
        :param shaper_freqs: tuple (start, end, step) or None
        """
        lib = self._load_rusty_lib()

        shapers_csv = ",".join(shapers) if shapers else None
        test_dr_csv = (
            ",".join(str(r) for r in test_damping_ratios)
            if test_damping_ratios
            else None
        )
        freq_range_csv = (
            "%s:%s:%s" % (shaper_freqs[0], shaper_freqs[1], shaper_freqs[2])
            if shaper_freqs
            else None
        )
        max_sm = float(max_smoothing) if max_smoothing is not None else 0.0
        max_f = float(max_freq) if max_freq is not None else 0.0
        wt = float(window_t) if window_t is not None else 0.0

        if is_raw:
            ptr = lib.rusty_shaper_calibrate_from_csv(
                self._encode(csv_path),
                self._encode(shapers_csv),
                float(damping_ratio),
                self._encode(test_dr_csv),
                float(scv),
                max_sm,
                max_f,
                self._encode(freq_range_csv),
                wt,
            )
        else:
            ptr = lib.rusty_shaper_calibrate_from_psd(
                self._encode(csv_path),
                self._encode(shapers_csv),
                float(damping_ratio),
                self._encode(test_dr_csv),
                float(scv),
                max_sm,
                max_f,
                self._encode(freq_range_csv),
            )

        if ptr is None:
            raise self.error(
                "rusty-shaper returned an error for '%s'" % csv_path
            )

        try:
            raw_json = ctypes.cast(ptr, ctypes.c_char_p).value
            return json.loads(raw_json)
        finally:
            lib.rusty_shaper_free_string(ptr)

    # ────────────────────────────────────────────────────────────
    # Subprocess harness (unchanged from upstream)
    # ────────────────────────────────────────────────────────────

    def background_process_exec(self, method, args):
        if self.printer is None:
            return method(*args)
        import queuelogger

        parent_conn, child_conn = multiprocessing.Pipe()

        def wrapper():
            queuelogger.clear_bg_logging()
            try:
                res = method(*args)
            except:
                child_conn.send((True, traceback.format_exc()))
                child_conn.close()
                return
            child_conn.send((False, res))
            child_conn.close()

        calc_proc = multiprocessing.Process(target=wrapper)
        calc_proc.daemon = True
        calc_proc.start()
        reactor = self.printer.get_reactor()
        gcode = self.printer.lookup_object("gcode")
        eventtime = last_report_time = reactor.monotonic()
        while calc_proc.is_alive():
            if eventtime > last_report_time + 5.0:
                last_report_time = eventtime
                gcode.respond_info("Wait for calculations..", log=False)
            eventtime = reactor.pause(eventtime + 0.1)
        is_err, res = parent_conn.recv()
        if is_err:
            raise self.error("Error in remote calculation: %s" % (res,))
        calc_proc.join()
        parent_conn.close()
        return res

    # ────────────────────────────────────────────────────────────
    # Numpy path (upstream-identical)
    # ────────────────────────────────────────────────────────────

    def _split_into_windows(self, x, window_size, overlap):
        step_between_windows = window_size - overlap
        n_windows = (x.shape[-1] - overlap) // step_between_windows
        shape = (window_size, n_windows)
        strides = (x.strides[-1], step_between_windows * x.strides[-1])
        return self.numpy.lib.stride_tricks.as_strided(
            x, shape=shape, strides=strides, writeable=False
        )

    def _psd(self, x, fs, nfft):
        np = self.numpy
        window = np.kaiser(nfft, 6.0)
        scale = 1.0 / (window**2).sum()
        overlap = nfft // 2
        x = self._split_into_windows(x, nfft, overlap)
        x = window[:, None] * (x - np.mean(x, axis=0))
        result = np.fft.rfft(x, n=nfft, axis=0)
        result = np.conjugate(result) * result
        result *= scale / fs
        result[1:-1, :] *= 2.0
        psd = result.real.mean(axis=-1)
        freqs = np.fft.rfftfreq(nfft, 1.0 / fs)
        return freqs, psd

    def calc_freq_response(self, raw_values):
        np = self.numpy
        if raw_values is None:
            return None
        if isinstance(raw_values, np.ndarray):
            data = raw_values
        else:
            samples = raw_values.get_samples()
            if not samples:
                return None
            data = np.array(samples)

        N = data.shape[0]
        T = data[-1, 0] - data[0, 0]
        SAMPLING_FREQ = N / T
        M = 1 << int(SAMPLING_FREQ * WINDOW_T_SEC - 1).bit_length()
        if N <= M:
            return None

        psd_list = []
        for i in range(1, 4):
            freqs, psd = self._psd(data[:, i], SAMPLING_FREQ, M)
            psd_list.append(psd)

        psd_x, psd_y, psd_z = psd_list
        psd_sum = psd_x + psd_y + psd_z

        cal_data = CalibrationData(freqs, psd_sum, psd_x, psd_y, psd_z)
        cal_data.set_numpy(np)
        return cal_data

    def process_accelerometer_data(self, data):
        return self.background_process_exec(self.calc_freq_response, (data,))

    def _estimate_shaper(self, shaper, test_damping_ratio, freq_bins):
        np = self.numpy
        A, T = shaper
        inv_D = 1.0 / sum(A)
        omega = 2.0 * math.pi * freq_bins
        damping = test_damping_ratio * omega
        omega_d = omega * math.sqrt(1.0 - test_damping_ratio**2)
        W = A * np.exp(
            np.outer(-damping, (T[-1] - T))
        )
        S = W * np.sin(np.outer(omega_d, T))
        C = W * np.cos(np.outer(omega_d, T))
        return np.sqrt(S.sum(axis=1) ** 2 + C.sum(axis=1) ** 2) * inv_D

    def _get_shaper_smoothing(self, shaper, accel=5000, scv=5.0):
        half_accel = accel * 0.5
        A, T = shaper
        inv_D = 1.0 / sum(A)
        ts = sum([a * t for a, t in zip(A, T)]) * inv_D
        offset_90 = (
            sum([a * (scv + half_accel * abs(t - ts)) * abs(t - ts) for a, t in zip(A, T)])
            * inv_D
        )
        offset_180 = (
            sum([a * half_accel * (t - ts) ** 2 for a, t in zip(A, T)]) * inv_D
        )
        return max(offset_90 * math.sqrt(2.0), offset_180)

    def fit_shaper(self, shaper_cfg, calibration_data, max_smoothing):
        np = self.numpy
        test_freqs = np.arange(
            shaper_cfg.min_freq, MAX_SHAPER_FREQ, 0.2
        )
        freq_bins = calibration_data.freq_bins
        psd = calibration_data.psd_sum / calibration_data.psd_sum.max()
        best_res = None
        results = []
        for test_freq in test_freqs[::-1]:
            shaper = shaper_cfg.init_func(test_freq, shaper_defs.DEFAULT_DAMPING_RATIO)
            shaper_smoothing = self._get_shaper_smoothing(shaper)
            if (
                max_smoothing is not None
                and shaper_smoothing > max_smoothing
                and best_res is not None
            ):
                break
            shaper_vibrations = 0.0
            for test_damping_ratio in TEST_DAMPING_RATIOS:
                estimated_vibrations = np.interp(
                    freq_bins,
                    freq_bins,
                    self._estimate_shaper(shaper, test_damping_ratio, freq_bins) * psd,
                )
                shaper_vibrations = max(
                    shaper_vibrations, estimated_vibrations.sum()
                )
            max_accel = self.find_shaper_max_accel(shaper)
            res = CalibrationResult(
                name=shaper_cfg.name,
                freq=test_freq,
                vals=shaper,
                vibrs=shaper_vibrations,
                smoothing=shaper_smoothing,
                score=shaper_smoothing
                * (shaper_vibrations**1.5 + shaper_vibrations * 0.2 + 0.01),
                max_accel=max_accel,
            )
            results.append(res)
            if best_res is None or res.vibrs < best_res.vibrs:
                best_res = res
        if best_res is None:
            return None
        selected = best_res
        for res in results[::-1]:
            if res.vibrs < best_res.vibrs * 1.1 and res.score < selected.score:
                selected = res
        return selected

    def _get_max_accel(self, shaper, scv=5.0):
        return self._get_shaper_smoothing(shaper, scv=scv)

    def find_shaper_max_accel(self, shaper, scv=5.0):
        TARGET_SMOOTHING = 0.12
        def _check_smoothing(accel):
            return self._get_shaper_smoothing(shaper, accel=accel, scv=scv) <= TARGET_SMOOTHING
        lo, hi = 1.0, 1.0
        if not _check_smoothing(1e-9):
            return 0.0
        while not _check_smoothing(lo):
            hi = lo
            lo *= 0.5
        if hi == lo:
            while _check_smoothing(hi):
                hi *= 2.0
        while hi - lo > 1.0:
            mid = (lo + hi) * 0.5
            if _check_smoothing(mid):
                lo = mid
            else:
                hi = mid
        return lo

    def find_best_shaper(
        self,
        calibration_data,
        max_smoothing=None,
        scv=5.0,
        max_freq=None,
        shapers=None,
        logger=None,
    ):
        """Find the best shaper. shapers is an optional list of shaper name
        strings; None means use AUTOTUNE_SHAPERS."""
        best_shaper = None
        all_shapers = []
        shaper_list = shapers if shapers else AUTOTUNE_SHAPERS
        for shaper_name in shaper_list:
            shaper_cfg = next(
                (s for s in shaper_defs.INPUT_SHAPERS if s.name == shaper_name), None
            )
            if shaper_cfg is None:
                if logger:
                    logger("Warning: unknown shaper '%s', skipping" % shaper_name)
                continue
            calibration_data_filtered = calibration_data
            shaper = self.background_process_exec(
                self.fit_shaper, (shaper_cfg, calibration_data_filtered, max_smoothing)
            )
            if shaper is None:
                if logger:
                    logger(
                        "Note: shaper '%s' did not find a valid configuration"
                        % shaper_name
                    )
                continue
            if logger:
                logger(
                    "Fitted shaper '%s' frequency = %.1f Hz "
                    "(vibrations = %.1f%%, smoothing ~= %.3f)\n"
                    "To avoid too much smoothing with '%s', "
                    "suggested max_accel <= %.0f mm/sec^2"
                    % (
                        shaper.name,
                        shaper.freq,
                        shaper.vibrs * 100.0,
                        shaper.smoothing,
                        shaper.name,
                        (shaper.max_accel / 100.0) * 100.0,
                    )
                )
            all_shapers.append(shaper)
            if best_shaper is None or (
                shaper.score * 1.2 < best_shaper.score
                or (
                    shaper.score * 1.05 < best_shaper.score
                    and shaper.smoothing * 1.1 < best_shaper.smoothing
                )
            ):
                best_shaper = shaper
        return best_shaper, all_shapers

    def apply_params(self, input_shaper, axis, shaper_type, shaper_freq):
        gcode = self.printer.lookup_object("gcode")
        input_shaper.cmd_SET_INPUT_SHAPER(
            gcode.create_gcode_command(
                "SET_INPUT_SHAPER",
                "SET_INPUT_SHAPER",
                {
                    "SHAPER_TYPE_" + axis.upper(): shaper_type,
                    "SHAPER_FREQ_" + axis.upper(): "%.3f" % shaper_freq,
                },
            )
        )

    def save_params(self, configfile, axis, shaper_type, shaper_freq):
        if axis == "x":
            configfile.set("input_shaper", "shaper_type_x", shaper_type)
            configfile.set("input_shaper", "shaper_freq_x", "%.1f" % shaper_freq)
        else:
            configfile.set("input_shaper", "shaper_type_y", shaper_type)
            configfile.set("input_shaper", "shaper_freq_y", "%.1f" % shaper_freq)

    def save_calibration_data(
        self, output, calibration_data, all_shapers=None, max_freq=None, accel_per_hz=None
    ):
        np = self.numpy
        if np is None:
            # rusty path – no PSD data available to write a Kalico-format CSV;
            # callers that need a CSV should use the rusty binary directly.
            return
        freq_bins = calibration_data.freq_bins
        if max_freq is not None:
            freq_bins = freq_bins[freq_bins <= max_freq + 1]
        header = "freq,psd_x,psd_y,psd_z,psd_xyz"
        psd_x = calibration_data.psd_x[: len(freq_bins)]
        psd_y = calibration_data.psd_y[: len(freq_bins)]
        psd_z = calibration_data.psd_z[: len(freq_bins)]
        psd_sum = calibration_data.psd_sum[: len(freq_bins)]
        if all_shapers:
            for s in all_shapers:
                header += ",%s(%.1f)" % (s.name, s.freq)
        with open(output, "w") as csvout:
            csvout.write(header + "\n")
            for i in range(len(freq_bins)):
                row = "%.1f,%.6e,%.6e,%.6e,%.6e" % (
                    freq_bins[i],
                    psd_x[i],
                    psd_y[i],
                    psd_z[i],
                    psd_sum[i],
                )
                if all_shapers:
                    for s in all_shapers:
                        A, T = s.vals
                        shaper_response = self._estimate_shaper(
                            s.vals,
                            shaper_defs.DEFAULT_DAMPING_RATIO,
                            np.array([freq_bins[i]]),
                        )
                        row += ",%.6e" % shaper_response[0]
                csvout.write(row + "\n")
        if accel_per_hz is not None:
            with open(output, "a") as csvout:
                csvout.write(
                    "# accel_per_hz: %.6f\n" % accel_per_hz
                )
