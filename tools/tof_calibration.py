#!/usr/bin/env python3
"""TOF (VL53L1X) calibration for Track 3: measured error and raw-vs-filtered plot.

Modes:
  error   Flash firmware/bringup/digital. Hold a flat target at a measured
          distance, type that distance (mm), and the script captures N valid
          samples (status 0) into calibration/data/tof_error.csv.
          At a prompt:  number = take a point,  u = undo last point,  q = quit.
  filter  Record TOF raw + filtered for --seconds into calibration/data/tof_filter.csv.
          With the full firmware (firmware/tsp05) it records the firmware's own
          filtered value from the telemetry line. With the bring-up sketch it
          records raw only and the report applies the same filter in Python.
          Move the target between 3 or 4 distances, holding each ~2 s.
  report  Read both CSVs and write calibration/plots/tof_error.png,
          calibration/plots/tof_filter.png and calibration/tof_results.md
          (error tables ready for the README).

Examples:
  python tools/tof_calibration.py error                 # auto-detect the Uno
  python tools/tof_calibration.py error --port COM4     # Windows
  python tools/tof_calibration.py filter --seconds 15
  python tools/tof_calibration.py report
  python tools/tof_calibration.py error --sim           # rehearse without hardware

--sim writes to calibration/sim_data/ (git-ignored), never to data/.
Close the Arduino Serial Monitor first: only one program can hold the port.
Needs pyserial for hardware; report needs numpy and matplotlib
(pip install -r tools/requirements.txt).
"""
from __future__ import annotations

import argparse
import collections
import csv
import math
import random
import re
import statistics
import sys
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONFIG_H = REPO / "firmware" / "tsp05" / "config.h"
DATA_DIR = REPO / "calibration" / "data"
SIM_DIR = REPO / "calibration" / "sim_data"
PLOTS_DIR = REPO / "calibration" / "plots"
RESULTS_MD = REPO / "calibration" / "tof_results.md"
ERROR_CSV, FILTER_CSV = "tof_error.csv", "tof_filter.csv"
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x10C4}   # Arduino, CH340 clones, CP210x

# Telemetry field order (docs/protocol.md)
TELEMETRY = ("ms", "mode", "flex_raw", "flex_deg", "flex_filt_deg", "tof_raw_mm",
             "tof_filt_mm", "th_raw", "temp_C", "slot_count", "slot_rpm", "servo_deg",
             "dc_deg", "dc_rpm", "dc_pwm", "step_deg", "step_target_deg", "interlock")
I_MS, I_RAW, I_FILT = (1 + TELEMETRY.index(f) for f in ("ms", "tof_raw_mm", "tof_filt_mm"))


def config_defines() -> dict[str, float]:
    out = {}
    for line in CONFIG_H.read_text().splitlines():
        m = re.match(r"^\s*#define\s+(\w+)\s+(-?[\d.]+)(?:f|UL|L|U)?\b", line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


C = config_defines()
MEDIAN_N = int(C.get("TOF_MEDIAN_N", 5))
ALPHA = C.get("TOF_EMA_ALPHA", 0.3)
PERIOD_MS = C.get("TOF_PERIOD_MS", 40)


def firmware_filter(raw: list[float]) -> list[float]:
    """Same filter as sensors_digital.cpp: median of the last N, then EMA."""
    out, hist, filt = [], [], None
    for x in raw:
        hist = (hist + [x])[-MEDIAN_N:]
        med = statistics.median(hist)
        filt = med if filt is None else filt + ALPHA * (med - filt)
        out.append(filt)
    return out


# ---------------------------------------------------------------------------
# Sample sources. read(n) returns the next n samples arriving after the call,
# as (ms, raw_mm, filt_mm or None).
# ---------------------------------------------------------------------------
class SerialSource:
    """Reads the port continuously in a background thread, so samples never
    pile up in OS/driver buffers while you type at a prompt (stale data)."""

    def __init__(self, port: str, baud: int):
        try:
            import serial
        except ImportError:
            sys.exit("pyserial is not installed: pip install -r tools/requirements.txt")
        self.ser = serial.Serial(port, baud, timeout=1)
        print(f"Opened {port}; waiting for the Uno to restart...")
        self._cv = threading.Condition()
        self._buf: collections.deque = collections.deque(maxlen=20000)
        self._count = 0
        self._error: BaseException | None = None
        self.kind = None          # "bringup" or "telemetry", from the first data line
        threading.Thread(target=self._pump, daemon=True).start()

    def _parse(self, line: str):
        parts = line.split(",")
        if parts[0] == "T" and len(parts) == len(TELEMETRY) + 1:
            self.kind = self.kind or "telemetry"
            return int(parts[I_MS]), float(parts[I_RAW]), float(parts[I_FILT])
        if len(parts) == 4 and parts[0].isdigit():            # ms,tof_mm,status,slot
            if int(parts[2]) != 0:
                return None                                   # invalid range: skip
            self.kind = self.kind or "bringup"
            return int(parts[0]), float(parts[1]), None
        raise ValueError(line)

    def _pump(self) -> None:
        bad = 0
        try:
            while True:
                raw = self.ser.readline()
                if not raw:
                    continue
                line = raw.decode(errors="replace").strip()
                if not line:
                    continue
                if line.startswith("#"):
                    print(f"  uno: {line[1:].strip()}")
                    continue
                try:
                    sample = self._parse(line)
                except ValueError:
                    bad += 1                       # e.g. a half line after a reset
                    if bad > 50:
                        raise RuntimeError(f"too many unrecognised lines, last: {line!r}")
                    continue
                if sample is None:
                    continue
                with self._cv:
                    self._buf.append(sample)
                    self._count += 1
                    self._cv.notify_all()
        except BaseException as e:
            with self._cv:
                self._error = e
                self._cv.notify_all()

    def read(self, n: int):
        with self._cv:
            start, last, t_last = self._count, self._count, time.time()
            while self._count - start < n:
                if self._error:
                    raise RuntimeError(f"serial reader stopped: {self._error}")
                self._cv.wait(timeout=0.5)
                if self._count != last:
                    last, t_last = self._count, time.time()
                elif time.time() - t_last > 3:
                    raise TimeoutError("no valid TOF samples for 3 s: check the port, that "
                                       "a sketch is running, and that the target is in range")
            got = self._count - start
            return list(self._buf)[-got:][:n]


class SimSource:
    """Pretend VL53L1X: +6 mm bias, noise growing with distance, 2 % outliers."""

    def __init__(self, seed: int = 3):
        self.rng, self.ms, self.truth = random.Random(seed), 0, 300.0
        self.kind = "bringup"

    def set_truth(self, mm: float) -> None:
        self.truth = mm

    def read(self, n: int):
        out = []
        for _ in range(n):
            self.ms += int(PERIOD_MS)
            x = self.truth + 6 + self.rng.gauss(0, 1.5 + self.truth / 250)
            if self.rng.random() < 0.02:
                x += self.rng.choice((-1, 1)) * self.rng.uniform(60, 150)
            out.append((self.ms, float(round(x)), None))
        return out


def find_port() -> str:
    try:
        from serial.tools import list_ports
    except ImportError:
        sys.exit("pyserial is not installed: pip install -r tools/requirements.txt")
    ports = list(list_ports.comports())
    likely = [p for p in ports if p.vid in ARDUINO_VIDS]
    if len(likely) == 1:
        return likely[0].device
    names = ", ".join(p.device for p in ports) or "none"
    sys.exit(f"Could not pick the Uno automatically (ports: {names}). Use --port COMx.")


# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
def log_error(src, path: Path, n: int) -> None:
    new = not path.exists()
    path.parent.mkdir(parents=True, exist_ok=True)
    print(f"\nWriting {path.relative_to(REPO)}. Suggested distances: "
          "50 100 200 300 400 600 800 1000 mm.\n"
          "Measure from the front face of the sensor to a flat, matte target held square to it.")
    with path.open("a", newline="") as f:
        w = csv.writer(f)
        if new:
            w.writerow(["ref_mm", "sample", "tof_mm"])
        history: list[int] = []                  # rows written per point, for undo
        while True:
            ans = input("\nReference distance (mm), u = undo, q = quit: ").strip().lower()
            if ans == "q":
                break
            if ans == "u":
                if history:
                    f.flush()
                    rows = list(csv.reader(path.open()))
                    keep = rows[:len(rows) - history.pop()]
                    with path.open("w", newline="") as g:
                        csv.writer(g).writerows(keep)
                    f.seek(0, 2)
                    print("  removed the last point")
                continue
            try:
                ref = float(ans)
            except ValueError:
                print("  type a number, u or q")
                continue
            if isinstance(src, SimSource):
                src.set_truth(ref)
            src.read(3)                          # let the reading settle after moving
            samples = src.read(n)
            vals = [s[1] for s in samples]
            for i, v in enumerate(vals):
                w.writerow([f"{ref:g}", i, f"{v:g}"])
            f.flush()
            history.append(len(vals))
            mean, sd = statistics.mean(vals), statistics.stdev(vals)
            print(f"  saved {len(vals)}: mean {mean:.1f} mm, error {mean - ref:+.1f} mm, "
                  f"std {sd:.1f} mm")


def log_filter(src, path: Path, seconds: float) -> None:
    n = max(10, int(seconds * 1000 / PERIOD_MS))
    if isinstance(src, SimSource):
        rows = []
        for d in (150, 400, 250, 600):           # target moved between 4 distances
            src.set_truth(d)
            rows += src.read(n // 4)
    else:
        print(f"Recording ~{seconds:g} s: move the target between 3-4 distances, "
              "holding each about 2 s...")
        rows = src.read(n)
    path.parent.mkdir(parents=True, exist_ok=True)
    t0 = rows[0][0]
    with path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "raw_mm", "filt_mm"])
        for ms, raw, filt in rows:
            w.writerow([f"{(ms - t0) / 1000:.3f}", f"{raw:g}", "" if filt is None else f"{filt:g}"])
    kind = "firmware filtered value" if rows[0][2] is not None else "raw only (filter applied at report)"
    print(f"saved {len(rows)} samples to {path.relative_to(REPO)} ({kind})")


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------
def robust_noise(x) -> float:
    """Per-sample noise from first differences, robust to the few big steps
    where the target moved: 1.4826 * MAD(diff) / sqrt(2)."""
    import numpy as np
    d = np.diff(np.asarray(x, float))
    return float(1.4826 * np.median(np.abs(d - np.median(d))) / math.sqrt(2))


def report(data: Path, plots: Path, results: Path) -> None:
    try:
        import numpy as np
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("report needs numpy and matplotlib: pip install -r tools/requirements.txt")
    plots.mkdir(parents=True, exist_ok=True)
    md = [f"## TOF (VL53L1X) results\n",
          f"Generated by `tools/tof_calibration.py report` from `{data.relative_to(REPO)}`.\n"]

    err_path = data / ERROR_CSV
    if err_path.exists():
        by_ref: dict[float, list[float]] = collections.defaultdict(list)
        for row in csv.DictReader(err_path.open()):
            by_ref[float(row["ref_mm"])].append(float(row["tof_mm"]))
        refs = sorted(by_ref)
        means = np.array([np.mean(by_ref[r]) for r in refs])
        sds = np.array([np.std(by_ref[r], ddof=1) if len(by_ref[r]) > 1 else 0 for r in refs])
        errs = means - np.array(refs)
        rows = [f"| {r:g} | {len(by_ref[r])} | {m:.1f} | {e:+.1f} | {s:.1f} | {100 * e / r:+.1f} % |"
                for r, m, e, s in zip(refs, means, errs, sds)]
        rms = float(np.sqrt(np.mean(errs ** 2)))
        md += ["### Measured error\n", "![TOF error](plots/tof_error.png)\n",
               "| Reference (mm) | n | Mean reading (mm) | Error (mm) | Std dev (mm) | Error (% of reference) |",
               "|---|---|---|---|---|---|", *rows, "",
               f"Over {refs[0]:g} to {refs[-1]:g} mm: mean error {errs.mean():+.1f} mm, "
               f"RMS error {rms:.1f} mm, max |error| {np.abs(errs).max():.1f} mm, "
               f"worst std dev {sds.max():.1f} mm. Raw (unfiltered) readings, status-0 only.\n"]
        if len(refs) >= 2:
            off = float(errs.mean())
            rms_corr = float(np.sqrt(np.mean((errs - off) ** 2)))
            md += [f"With a constant offset correction of {-off:+.1f} mm the RMS error would be "
                   f"{rms_corr:.1f} mm.\n"]
        fig, ax = plt.subplots(figsize=(7.2, 4.2))
        ax.axhline(0, color="#999", lw=1)
        ax.errorbar(refs, errs, yerr=sds, fmt="o", capsize=4, color="#2b6cb0",
                    label="Mean error ± 1 std dev")
        ax.set_xlabel("Reference distance (mm)")
        ax.set_ylabel("TOF reading − reference (mm)")
        ax.set_title("VL53L1X measured error (short mode, 33 ms budget)")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plots / "tof_error.png", dpi=150)
        plt.close(fig)
        print(f"wrote {(plots / 'tof_error.png').relative_to(REPO)}")
    else:
        md.append(f"_No {ERROR_CSV} yet._\n")

    filt_path = data / FILTER_CSV
    if filt_path.exists():
        rows = list(csv.DictReader(filt_path.open()))
        t = np.array([float(r["t_s"]) for r in rows])
        raw = np.array([float(r["raw_mm"]) for r in rows])
        have_fw = all(r["filt_mm"] for r in rows)
        filt = np.array([float(r["filt_mm"]) for r in rows]) if have_fw else np.array(firmware_filter(list(raw)))
        n_raw, n_filt = robust_noise(raw), robust_noise(filt)
        # Outliers: raw samples far from a centred median of 5 of their neighbours
        # (centred, so the steps where the target moved don't count)
        pad = np.pad(raw, 2, mode="edge")
        centred = np.median(np.lib.stride_tricks.sliding_window_view(pad, 5), axis=1)
        outliers = int(np.sum(np.abs(raw - centred) > max(5 * n_raw, 20)))
        tau = -PERIOD_MS / math.log(1 - ALPHA)
        source = "firmware output" if have_fw else "same filter applied in Python to the raw log"
        md += ["### Filter: raw vs filtered\n", "![TOF filter](plots/tof_filter.png)\n",
               "| Filter | Raw noise (mm) | Filtered noise (mm) | Noise reduction | Outliers in raw |",
               "|---|---|---|---|---|",
               f"| median of {MEDIAN_N} → EMA α = {ALPHA:g} | {n_raw:.2f} | {n_filt:.2f} | "
               f"{100 * (1 - n_filt / n_raw):.0f} % | {outliers} |", "",
               f"Filtered values: {source}. Noise = per-sample standard deviation from the "
               f"sample-to-sample change, robust to the steps where the target moved. "
               f"Lag cost: the median delays a step by about {(MEDIAN_N - 1) // 2} samples "
               f"({(MEDIAN_N - 1) // 2 * PERIOD_MS:.0f} ms) and the EMA adds a time constant "
               f"τ = −Δt / ln(1 − α) = {tau:.0f} ms at Δt = {PERIOD_MS:.0f} ms.\n"]
        fig, ax = plt.subplots(figsize=(8, 4.2))
        ax.plot(t, raw, ".", ms=3, color="#a0aec0", label="Raw")
        ax.plot(t, filt, "-", lw=1.8, color="#2b6cb0", label=f"Filtered (median {MEDIAN_N} → EMA {ALPHA:g})")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("Distance (mm)")
        ax.set_title("VL53L1X raw vs filtered")
        ax.grid(alpha=0.3)
        ax.legend()
        fig.tight_layout()
        fig.savefig(plots / "tof_filter.png", dpi=150)
        plt.close(fig)
        print(f"wrote {(plots / 'tof_filter.png').relative_to(REPO)}")
    else:
        md.append(f"_No {FILTER_CSV} yet._\n")

    results.write_text("\n".join(md) + "\n", encoding="utf-8")
    print(f"wrote {results.relative_to(REPO)}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["error", "filter", "report"])
    p.add_argument("--port", help="serial port, e.g. COM4 or /dev/ttyACM0 (default: auto)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--samples", type=int, default=20, help="error: samples per distance")
    p.add_argument("--seconds", type=float, default=15, help="filter: recording length")
    p.add_argument("--sim", action="store_true", help="simulated sensor, writes to calibration/sim_data/")
    a = p.parse_args()

    data = SIM_DIR if a.sim else DATA_DIR
    if a.mode == "report":
        plots = SIM_DIR / "plots" if a.sim else PLOTS_DIR
        results = SIM_DIR / "tof_results.md" if a.sim else RESULTS_MD
        report(data, plots, results)
        return
    src = SimSource() if a.sim else SerialSource(a.port or find_port(), a.baud)
    if a.mode == "error":
        log_error(src, data / ERROR_CSV, a.samples)
    else:
        log_filter(src, data / FILTER_CSV, a.seconds)


if __name__ == "__main__":
    main()
