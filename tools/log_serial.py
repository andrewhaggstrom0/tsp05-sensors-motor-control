#!/usr/bin/env python3
"""Calibration logger for Track 4.

Reads the stream from firmware/bringup/analog ("ms,flex_adc,th_adc" at 50 Hz),
asks you for the reference value at each point, averages the samples, and
appends one row per point to calibration/data/. Rows are written as soon as
they are taken, so a crash or a bumped cable never loses earlier points.

Modes:
  therm   type the thermometer reading (degC) at each point -> thermistor.csv
          waits for the reading to settle before capturing
  flex    type the bend angle (degrees) at each point       -> flex.csv
          bend/unbend direction and trial number are tracked for you
  stream  record the flex sensor for N seconds while you bend it -> flex_filter.csv
          (the raw signal for the raw-vs-filtered plot)

At a prompt:  a number = take a point,  u = undo last row,  q = quit.

Examples:
  python3 tools/log_serial.py therm                    # auto-detect the Uno
  python3 tools/log_serial.py flex --port /dev/ttyACM0  # Linux / WSL
  python3 tools/log_serial.py flex --port COM4          # Windows
  python3 tools/log_serial.py stream --seconds 15
  python3 tools/log_serial.py therm --sim              # no hardware: simulated sensor

Needs pyserial for real hardware (pip install -r tools/requirements.txt).
--sim needs nothing beyond the standard library.
"""
from __future__ import annotations

import argparse
import collections
import csv
import math
import random
import statistics
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analog_model as m  # noqa: E402

DATA_DIR = m.REPO / "calibration" / "data"
SIM_DIR = m.REPO / "calibration" / "sim_data"
SAMPLE_HZ = 50

# CSV schemas (documented in calibration/README.md). Only measured or
# reference values are stored; anything derived from config.h constants
# (model temperature, model angle, errors) is computed at plot time, so
# changing a constant never leaves stale numbers in a CSV.
SCHEMAS = {
    "therm": ["ref_C", "adc_mean", "adc_std", "n"],
    "flex": ["trial", "direction", "ref_deg", "adc_mean", "adc_std", "n"],
    "stream": ["t_s", "flex_adc"],
}
DEFAULT_FILES = {"therm": "thermistor.csv", "flex": "flex.csv", "stream": "flex_filter.csv"}

# USB vendor IDs: Arduino LLC/SRL, WCH CH340 clones, Silicon Labs CP210x
ARDUINO_VIDS = {0x2341, 0x2A03, 0x1A86, 0x10C4}


# ----------------------------------------------------------------------------
# Sample sources
# ----------------------------------------------------------------------------
class SerialSource:
    """The real Uno running firmware/bringup/analog.

    A background thread reads the port continuously, so data never piles up
    in the operating system, driver, or usbipd (WSL) buffers while you are
    typing at a prompt. read(n) then returns only samples that arrive after it
    is called. (Clearing the buffer with reset_input_buffer() is not enough
    over usbipd: data still in flight arrives after the clear, so captures
    started with seconds-old readings.)
    """

    def __init__(self, port: str, baud: int):
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("pyserial is not installed: pip install -r tools/requirements.txt")
        self.ser = serial.Serial(port, baud, timeout=1)
        # Opening the port resets the Uno; wait for its banner.
        deadline = time.time() + 4
        while time.time() < deadline:
            line = self.ser.readline().decode(errors="replace").strip()
            if line.startswith("# tsp05 analog bringup"):
                print(f"Connected to {port}: {line[2:]}")
                break
        else:
            print(f"Warning: no bring-up banner from {port}. Is "
                  "firmware/bringup/analog flashed? Continuing anyway.")
        self._cv = threading.Condition()
        self._buf: collections.deque = collections.deque(maxlen=SAMPLE_HZ * 600)
        self._count = 0          # samples received since connecting
        self._error: BaseException | None = None
        threading.Thread(target=self._pump, daemon=True).start()

    def _pump(self) -> None:
        bad = 0
        try:
            while True:
                raw = self.ser.readline()
                if not raw:
                    continue                      # read() reports the stall
                line = raw.decode(errors="replace").strip()
                if not line or line.startswith("#"):
                    continue
                try:
                    ms, flex, th = line.split(",")
                    sample = (int(ms), float(flex), float(th))
                except ValueError:
                    bad += 1                      # e.g. a line cut by a reset
                    if bad > 25:
                        raise RuntimeError(f"too many malformed lines, last: {line!r}")
                    continue
                with self._cv:
                    self._buf.append(sample)
                    self._count += 1
                    self._cv.notify_all()
        except BaseException as e:               # surface it in read()
            with self._cv:
                self._error = e
                self._cv.notify_all()

    def read(self, n: int) -> list[tuple[int, float, float]]:
        """The next n samples that arrive after this call, as (ms, flex_adc, th_adc)."""
        with self._cv:
            start = self._count
            last_progress, last_count = time.time(), start
            while self._count - start < n:
                if self._error:
                    raise RuntimeError(f"serial reader stopped: {self._error}")
                self._cv.wait(timeout=0.5)
                if self._count != last_count:
                    last_progress, last_count = time.time(), self._count
                elif time.time() - last_progress > 2:
                    raise TimeoutError("no data for 2 s: check the port, the baud rate, "
                                       "and that the bring-up sketch is running")
            got = self._count - start
            out = list(self._buf)[-got:][:n]
        # The Uno timestamps every sample; a jump means samples went missing.
        gaps = [b[0] - a[0] for a, b in zip(out, out[1:])]
        if gaps and max(gaps) > 5 * 1000 // SAMPLE_HZ:
            print(f"  Warning: {max(gaps)} ms gap in the data (samples were lost). "
                  "Redo this point.")
        return out

    def set_truth(self, **_kw) -> None:
        pass  # the real world sets the truth


class SimSource:
    """A pretend sensor, so the whole workflow can be rehearsed without hardware.

    Deliberately imperfect, so the plots and error tables look like real data:
      thermistor: +0.4 degC bias, first-order lag (tau 3 s), 0.6 count noise
      flex: this unit is 8% stiffer-reading than the placeholder table, with a
            slight curve, 4% hysteresis while unbending, 1 count noise
    """

    def __init__(self, seed: int = 1):
        self.rng = random.Random(seed)
        self.ms = 0
        self.temp_target = self.temp = 22.0
        self.angle = 0.0
        self.unbending = False

    def set_truth(self, temp_c: float | None = None, angle_deg: float | None = None,
                  unbending: bool = False) -> None:
        if temp_c is not None:
            self.temp_target = temp_c
        if angle_deg is not None:
            self.angle = angle_deg
        self.unbending = unbending

    def _flex_adc(self, angle: float) -> float:
        a = max(angle, 0.0) / 90.0
        r = 10500.0 * (1 + 0.92 * a + 0.12 * a * a) * (1.04 if self.unbending else 1.0)
        return min(m.ADC_MAX, max(0.0, m.adc_from_flex_ohms(r) + self.rng.gauss(0, 1)))

    def read(self, n: int) -> list[tuple[int, float, float]]:
        out = []
        dt = 1.0 / SAMPLE_HZ
        for _ in range(n):
            self.ms += int(1000 * dt)
            self.temp += (self.temp_target - self.temp) * (1 - math.exp(-dt / 3.0))
            th = m.adc_from_therm_c(self.temp + 0.4) + self.rng.gauss(0, 0.6)
            out.append((self.ms, round(self._flex_adc(self.angle)), round(th, 2)))
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
    listing = "\n".join(f"  {p.device}  {p.description}" for p in ports) or "  (none)"
    sys.exit(f"Could not pick a port automatically. Pass --port. Ports found:\n{listing}")


# ----------------------------------------------------------------------------
# CSV helpers
# ----------------------------------------------------------------------------
def read_rows(path: Path) -> list[dict]:
    if not path.exists() or path.stat().st_size == 0:
        return []
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def write_rows(path: Path, fields: list[str], rows: list[dict]) -> None:
    with path.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)


def open_output(path: Path, fields: list[str], fresh: bool) -> list[dict]:
    """Returns rows already in the file (empty if new). Checks the header."""
    path.parent.mkdir(parents=True, exist_ok=True)
    if fresh and path.exists():
        bak = path.with_suffix(path.suffix + ".bak")
        path.replace(bak)
        print(f"Moved old {path.name} to {bak.name}")
    rows = read_rows(path)
    if rows and list(rows[0].keys()) != fields:
        sys.exit(f"{path} has columns {list(rows[0].keys())}, expected {fields}. "
                 "Use --new to start a fresh file (the old one is kept as .bak).")
    if rows:
        print(f"Appending to {show(path)} ({len(rows)} rows already there)")
    else:
        write_rows(path, fields, [])
        print(f"Writing {show(path)}")
    return rows


def show(path: Path) -> str:
    """Path relative to the repo when inside it, else as given."""
    try:
        return str(path.resolve().relative_to(m.REPO))
    except ValueError:
        return str(path)


def stats(values: list[float]) -> tuple[float, float]:
    return statistics.fmean(values), (statistics.stdev(values) if len(values) > 1 else 0.0)


# ----------------------------------------------------------------------------
# Modes
# ----------------------------------------------------------------------------
def wait_for_settle(src, tol: float, timeout_s: float) -> None:
    """Thermistors take seconds to reach the water temperature. Wait until two
    consecutive 1 s means agree within tol counts."""
    prev = None
    calm = 0
    for _ in range(int(timeout_s)):          # each read() is 1 s of samples
        mean, _ = stats([s[2] for s in src.read(SAMPLE_HZ)])
        if prev is not None:
            delta = mean - prev
            print(f"  settling: th_adc {mean:7.2f}  (change {delta:+.2f})")
            calm = calm + 1 if abs(delta) < tol else 0
            if calm >= 2:
                return
        prev = mean
    print(f"  Not settled after {timeout_s:.0f} s; capturing anyway. "
          "Stir the water and keep the probe still.")


def parse_command(prompt: str, lo: float, hi: float) -> float | str:
    while True:
        try:
            s = input(prompt).strip().lower()
        except EOFError:
            return "q"
        if s in ("q", "u"):
            return s
        try:
            v = float(s)
        except ValueError:
            print("  Enter a number, u (undo) or q (quit).")
            continue
        if not lo <= v <= hi:
            print(f"  Out of range ({lo:g} to {hi:g}).")
            continue
        return v


def run_therm(src, path: Path, args) -> None:
    fields = SCHEMAS["therm"]
    rows = open_output(path, fields, args.new)
    print("\nFor each point: put the thermistor next to the thermometer bulb, "
          "type the thermometer reading.\nSuggested: ice water (stirred), room, "
          "body (hold it), warm water at 2 to 3 temperatures.\n")
    while True:
        cmd = parse_command("therm> reference degC (u = undo, q = quit): ", -10, 100)
        if cmd == "q":
            break
        if cmd == "u":
            if rows:
                print(f"  Removed {rows.pop()}")
                write_rows(path, fields, rows)
            continue
        src.set_truth(temp_c=cmd)
        wait_for_settle(src, args.settle_tol, args.settle_timeout)
        mean, std = stats([s[2] for s in src.read(args.samples)])
        row = {"ref_C": f"{cmd:.2f}", "adc_mean": f"{mean:.2f}",
               "adc_std": f"{std:.2f}", "n": args.samples}
        rows.append(row)
        write_rows(path, fields, rows)
        model = m.therm_c_from_adc(mean)
        print(f"  saved: ref {cmd:.2f} C, adc {mean:.2f} +/- {std:.2f} "
              f"-> Beta model {model:.2f} C (error {model - cmd:+.2f} C)")
    print(f"{len(rows)} points in {show(path)}")


def run_flex(src, path: Path, args) -> None:
    fields = SCHEMAS["flex"]
    rows = open_output(path, fields, args.new)
    trial = max((int(r["trial"]) for r in rows), default=0) + 1
    prev = None
    direction = "bend"
    print(f"\nTrial {trial}. Tape the crimped end down, flat. Bend the sensor in steps "
          "and hold it at each angle\n(protractor or a printed angle guide), type the "
          "angle, keep holding until 'saved'. Then come back\ndown in steps. 0 = flat. "
          "Bend/unbend direction is tracked for you.\n")
    while True:
        cmd = parse_command("flex> bend angle in degrees (u = undo, q = quit): ", -30, 180)
        if cmd == "q":
            break
        if cmd == "u":
            if rows:
                print(f"  Removed {rows.pop()}")
                write_rows(path, fields, rows)
            continue
        if prev is not None:
            if cmd > prev and direction == "unbend":
                trial += 1  # bending again after unbending starts a new trial
                print(f"  Starting trial {trial}")
            if cmd != prev:
                direction = "bend" if cmd > prev else "unbend"
        prev = cmd
        src.set_truth(angle_deg=cmd, unbending=(direction == "unbend"))
        src.read(int(args.flex_settle_s * SAMPLE_HZ))   # let the reading settle
        mean, std = stats([s[1] for s in src.read(args.samples)])
        row = {"trial": trial, "direction": direction, "ref_deg": f"{cmd:.1f}",
               "adc_mean": f"{mean:.2f}", "adc_std": f"{std:.2f}", "n": args.samples}
        rows.append(row)
        write_rows(path, fields, rows)
        r = m.flex_ohms_from_adc(mean)
        print(f"  saved: trial {trial} {direction}, {cmd:g} deg, adc {mean:.1f} +/- {std:.1f} "
              f"(R = {r / 1000:.2f} k) -> table {m.flex_deg_from_adc(mean):.1f} deg")
    print(f"{len(rows)} points in {show(path)}")


def run_stream(src, path: Path, args) -> None:
    fields = SCHEMAS["stream"]
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.exists():
        bak = path.with_suffix(path.suffix + ".bak")
        path.replace(bak)
        print(f"Moved old {path.name} to {bak.name}")
    n = int(args.seconds * SAMPLE_HZ)
    if isinstance(src, SimSource):
        samples = []
        for k in range(n):   # a few presses of different strength
            t = k / SAMPLE_HZ
            phase = t % 4.0
            level = (30.0, 60.0, 90.0)[int(t // 4.0) % 3]
            src.set_truth(angle_deg=level if 0.5 < phase < 2.5 else 0.0)
            samples += src.read(1)
    else:
        try:
            input(f"Press Enter, then bend and release the flex sensor a few times "
                  f"for {args.seconds:g} s (different angles, hold each ~2 s)...")
        except EOFError:
            pass
        samples = src.read(n)
    t0 = samples[0][0]
    write_rows(path, fields, [{"t_s": f"{(ms - t0) / 1000:.3f}", "flex_adc": f"{flex:g}"}
                              for ms, flex, _ in samples])
    print(f"Recorded {len(samples)} samples ({args.seconds:g} s) to "
          f"{show(path)}")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("mode", choices=["therm", "flex", "stream"])
    p.add_argument("--port", help="serial port (default: auto-detect the Uno)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", type=Path, help="output CSV (default: calibration/data/<mode file>)")
    p.add_argument("--new", action="store_true", help="start a fresh file (old one kept as .bak)")
    p.add_argument("--samples", type=int, help="samples averaged per point "
                   "(default 100 therm, 50 flex)")
    p.add_argument("--settle-tol", type=float, default=0.5,
                   help="therm: counts of change per second that counts as settled")
    p.add_argument("--settle-timeout", type=float, default=90, help="therm: max seconds to wait")
    p.add_argument("--flex-settle-s", type=float, default=1.0,
                   help="flex: seconds to wait after reaching each angle")
    p.add_argument("--seconds", type=float, default=15, help="stream: recording length")
    p.add_argument("--sim", action="store_true", help="simulated sensor, no hardware")
    args = p.parse_args()
    if args.samples is None:
        args.samples = 100 if args.mode == "therm" else 50

    # Simulated data goes to its own (git-ignored) folder so it can never be
    # mistaken for, or overwrite, real calibration data.
    default_dir = SIM_DIR if args.sim else DATA_DIR
    path = args.out or default_dir / DEFAULT_FILES[args.mode]
    if args.sim:
        print("SIMULATED sensor (rehearsal only)")
        src = SimSource()
    else:
        src = SerialSource(args.port or find_port(), args.baud)

    {"therm": run_therm, "flex": run_flex, "stream": run_stream}[args.mode](src, path, args)


if __name__ == "__main__":
    main()
