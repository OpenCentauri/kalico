#!/usr/bin/env python3
"""Campaign A/B runner: N x (heat soak + G28 + BED_MESH_CALIBRATE) on carbon2u.

usage: mesh_campaign.py <label> <temp> <runs>
Writes per-run results to ~/carbon/HX711/<label>/results.jsonl and captures
klippy.log fault deltas per run. Continues on failure; logs it.
"""
import json, os, subprocess, sys, time, urllib.request, urllib.parse

HOST = "http://carbon2u"
LABEL, TEMP, RUNS = sys.argv[1], int(sys.argv[2]), int(sys.argv[3])
OUT = os.path.expanduser(f"~/carbon/HX711/{LABEL}")
os.makedirs(OUT, exist_ok=True)
FAULT_KEYS = ["desync", "DESYNC", "SAMPLE_ERROR", "bad_frame", "RECOVERED",
              "Bad tap", "watchdog", "timed out", "Probe Error", "shutdown",
              "MCU 'bed' error", "torn"]


def req(path, data=None, timeout=20):
    url = HOST + path
    r = urllib.request.Request(url, data=data)
    return json.loads(urllib.request.urlopen(r, timeout=timeout).read())


def gcode(script, critical=False):
    # POST can hang on busy klippy; a timeout just means "submitted".
    # NEVER retry long macros — a retried timeout = duplicate submission.
    q = urllib.parse.quote(script)
    try:
        return req(f"/printer/gcode/script?script={q}", timeout=10)
    except Exception:
        if critical:
            raise
        return None


def state():
    try:
        d = req("/printer/objects/query?idle_timeout&print_stats&bed_mesh&toolhead&heater_bed", timeout=10)
        s = d["result"]["status"]
        return s
    except Exception as e:
        return {"error": str(e)}


def wait_idle(timeout=900, want_printing_first=False):
    t0 = time.time()
    if want_printing_first:  # wait until it actually starts moving
        while time.time() - t0 < 60:
            s = state()
            if s.get("idle_timeout", {}).get("state") == "Printing":
                break
            if s.get("print_stats", {}).get("state") == "error":
                return False
            time.sleep(2)
    while time.time() - t0 < timeout:
        s = state()
        it = s.get("idle_timeout", {}).get("state")
        ps = s.get("print_stats", {}).get("state")
        if ps == "error":
            return False
        if it in ("Idle", "Ready") and ps in ("standby", "idle", "complete", None):
            return True
        time.sleep(3)
    return False


def klippy_faults_since(mark_path):
    try:
        out = subprocess.run(
            ["ssh", "root@carbon2u",
             "awk 'NR>" + str(mark_path) + "' /board-resource/klippy.log | grep -iE '"
             + "|".join(FAULT_KEYS) + "' | grep -v 'runout' | tail -30"],
            capture_output=True, text=True, timeout=30)
        return [l for l in out.stdout.strip().splitlines() if l]
    except Exception as e:
        return [f"fault-check-error: {e}"]


def klippy_lines():
    try:
        out = subprocess.run(["ssh", "root@carbon2u", "wc -l < /board-resource/klippy.log"],
                             capture_output=True, text=True, timeout=20)
        return int(out.stdout.strip() or 0)
    except Exception:
        return 0


def wait_ready():
    for _ in range(120):
        try:
            d = req("/server/info", timeout=5)
            if d["result"].get("klippy_state") == "ready":
                return True
        except Exception:
            pass
        time.sleep(5)
    return False


def wait_temp(target, timeout=900):
    t0 = time.time()
    while time.time() - t0 < timeout:
        s = state()
        t = s.get("heater_bed", {}).get("temperature", 0)
        if abs(t - target) < 1.5:
            return True
        time.sleep(10)
    return False


def main():
    log = open(os.path.join(OUT, "results.jsonl"), "a")
    print(f"waiting for klippy ready...")
    if not wait_ready():
        print("FATAL: klippy never became ready"); sys.exit(1)
    print(f"heating bed to {TEMP}C...")
    gcode(f"M140 S{TEMP}")
    if not wait_temp(TEMP):
        print("FATAL: bed never reached temp"); sys.exit(1)
    gcode("G28")
    wait_idle(600)

    for run in range(1, RUNS + 1):
        mark = klippy_lines()
        t0 = time.time()
        rec = {"run": run, "temp": TEMP, "t0": t0}
        # POST may hang on a long macro; a timeout here just means "submitted".
        # Guard: never send while a previous macro is still running.
        if state().get("idle_timeout", {}).get("state") != "Printing":
            try:
                gcode("BED_MESH_CALIBRATE")
            except Exception as e:
                rec["post_timeout"] = str(e)
        else:
            rec["note"] = "already printing at run start; skipped send"
        ok = wait_idle(900, want_printing_first=True)
        # stall double-check: position frozen while "Printing" for 2 min
        if not ok:
            s1 = state().get("toolhead", {}).get("position", [0]*4)
            time.sleep(120)
            s2 = state().get("toolhead", {}).get("position", [0]*4)
            if s1 != s2:
                ok = wait_idle(600)  # still moving, give it more time
        rec["duration_s"] = round(time.time() - t0, 1)
        rec["ok"] = ok
        rec["faults"] = klippy_faults_since(mark)
        if not ok:
            rec["state"] = state()
            # try to recover for next run
            try:
                gcode("FIRMWARE_RESTART")
            except Exception:
                pass
            time.sleep(20)
            wait_ready()
            try:
                gcode(f"M140 S{TEMP}")
                wait_temp(TEMP)
                gcode("G28")
                wait_idle(600)
            except Exception as e2:
                rec["recovery_error"] = str(e2)
        log.write(json.dumps(rec) + "\n"); log.flush()
        print(f"run {run}/{RUNS}: {'OK' if ok else 'FAIL'} {rec['duration_s']}s faults={len(rec['faults'])}", flush=True)
    gcode("M140 S0")
    print("campaign done")


if __name__ == "__main__":
    main()
