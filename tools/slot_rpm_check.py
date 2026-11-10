#!/usr/bin/env python3
"""Slot-sensor RPM check against the stepper (Track 3).

Flash the full firmware (firmware/tsp05), put the disc on the stepper shaft so
its tab(s) pass through the slot, close the Serial Monitor, then:

  python tools/slot_rpm_check.py --port COM6              # 10 revolutions (P,3600)
  python tools/slot_rpm_check.py --port COM6 --revs 5
  python tools/slot_rpm_check.py --sim                     # rehearse without hardware

It switches to GUI mode (M,0), sends a relative stepper move (P,<deg>), records
every telemetry line until the move ends, and compares:
  reference RPM  = slope of step_deg over time while moving (the firmware's own
                   step count, timestamped by the Uno's clock: the speed the
                   stepper really ran at, including any slowdown of loop())
  measured RPM   = slot_rpm from sensors_digital.cpp, once a full revolution
                   has been seen
Writes calibration/data/slot_rpm.csv and calibration/plots/slot_rpm.png
(--sim: calibration/sim_data/).
"""
from __future__ import annotations

import argparse
import csv
import random
import statistics
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TELEMETRY = ("ms", "mode", "flex_raw", "flex_deg", "flex_filt_deg", "tof_raw_mm",
             "tof_filt_mm", "th_raw", "temp_C", "slot_count", "slot_rpm", "servo_deg",
             "dc_deg", "dc_rpm", "dc_pwm", "step_deg", "step_target_deg", "interlock")
IDX = {f: 1 + i for i, f in enumerate(TELEMETRY)}


def parse(line: str):
    parts = line.strip().split(",")
    if parts[0] != "T" or len(parts) != len(TELEMETRY) + 1:
        return None
    try:
        return (int(parts[IDX["ms"]]) / 1000.0, float(parts[IDX["step_deg"]]),
                int(float(parts[IDX["slot_count"]])), float(parts[IDX["slot_rpm"]]),
                int(float(parts[IDX["mode"]])))
    except ValueError:
        return None


def record_serial(port: str, baud: int, deg: float, timeout_s: float):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is not installed: pip install -r tools/requirements.txt")
    ser = serial.Serial(port, baud, timeout=0.2)
    print(f"Opened {port}; waiting for telemetry (the Uno restarts when the port opens)...")
    t_end = time.time() + 8
    first = None
    while time.time() < t_end and first is None:
        first = parse(ser.readline().decode(errors="replace"))
    if first is None:
        sys.exit("No T lines within 8 s: is the full firmware (firmware/tsp05) flashed?")
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b"M,0\n")
    time.sleep(0.3)
    ser.reset_input_buffer()
    start = None
    while start is None:
        start = parse(ser.readline().decode(errors="replace"))
    target = start[1] + deg
    print(f"Sending P,{deg:g}: stepper from {start[1]:.2f} to {target:.2f} deg ...")
    ser.write(f"P,{deg:g}\n".encode())
    rows, done_at, t_stop = [start], None, time.time() + timeout_s
    while time.time() < t_stop:
        r = parse(ser.readline().decode(errors="replace"))
        if r is None:
            continue
        rows.append(r)
        if done_at is None and abs(r[1] - target) < 0.5:
            done_at = time.time()
            print("  move finished, recording 2 s more")
        if done_at is not None and time.time() - done_at > 2.0:
            break
    else:
        print("  Warning: timed out before the move finished")
    ser.close()
    return rows


def record_sim(deg: float, tabs: int = 1, true_rpm: float = 29.6):
    """Stepper at 29.6 rpm with 20 Hz telemetry; slot sensor sees `tabs` per rev."""
    rows, t, step, count, rpm, last_edge = [], 0.0, 0.0, 0, 0.0, None
    rng = random.Random(2)
    deg_per_s = true_rpm * 6
    while True:
        t += 0.05
        new = min(deg, step + deg_per_s * 0.05)
        for k in range(int(step * tabs / 360) + 1, int(new * tabs / 360) + 1):
            edge_t = t - (new - k * 360 / tabs) / deg_per_s + rng.gauss(0, 0.002)
            if last_edge is not None:
                rpm = 60 / (tabs * (edge_t - last_edge))
            last_edge, count = edge_t, count + 1
        step = new
        rows.append((t, round(step / 0.225) * 0.225, count, round(rpm, 1), 0))
        if step >= deg and t > deg / deg_per_s + 2:
            return rows


def analyse(rows, deg: float):
    t = [r[0] for r in rows]
    step = [r[1] for r in rows]
    s0, s1 = step[0], step[0] + deg
    # Reference: least-squares slope of step_deg vs time, away from start/stop
    lo, hi = s0 + min(30, deg * 0.1), s1 - min(30, deg * 0.1)
    pts = [(ti, si) for ti, si in zip(t, step) if lo <= si <= hi]
    if len(pts) < 5:
        sys.exit("Too little motion recorded to compute the reference speed.")
    mt = statistics.mean(p[0] for p in pts)
    ms = statistics.mean(p[1] for p in pts)
    slope = sum((p[0] - mt) * (p[1] - ms) for p in pts) / sum((p[0] - mt) ** 2 for p in pts)
    ref_rpm = slope / 6.0
    move_t = (pts[-1][0] - pts[0][0]) * deg / (pts[-1][1] - pts[0][1])
    # Measured: slot_rpm while moving, once a non-zero value has appeared
    moving = [r for r in rows if lo <= r[1] <= hi and r[3] > 0]
    vals = [r[3] for r in moving]
    events = rows[-1][2] - rows[0][2]
    return ref_rpm, move_t, vals, events


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--revs", type=float, default=10)
    p.add_argument("--sim", action="store_true")
    a = p.parse_args()
    deg = 360 * a.revs

    if a.sim:
        rows = record_sim(deg)
        out_dir = REPO / "calibration" / "sim_data"
    else:
        if not a.port:
            sys.exit("Give the port, e.g. --port COM6")
        rows = record_serial(a.port, a.baud, deg, timeout_s=deg / 30 + 60)
        out_dir = REPO / "calibration" / "data"
    if any(r[4] != 0 for r in rows):
        print("Warning: mode was not 0 (GUI) for the whole run; the move may have been ignored")

    ref_rpm, move_t, vals, events = analyse(rows, deg)
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "slot_rpm.csv"
    t0 = rows[0][0]
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "step_deg", "slot_count", "slot_rpm"])
        for r in rows:
            w.writerow([f"{r[0] - t0:.3f}", f"{r[1]:.3f}", r[2], f"{r[3]:.2f}"])

    print(f"\nMove: {deg:g} deg ({a.revs:g} rev) in {move_t:.2f} s")
    print(f"Reference (stepper, from step_deg): {ref_rpm:.2f} rpm")
    print(f"Slot events during the run: {events} (expected {a.revs:g} x SLOT_TABS)")
    if vals:
        mean = statistics.mean(vals)
        print(f"slot_rpm while moving: mean {mean:.2f} rpm, min {min(vals):.2f}, max {max(vals):.2f} "
              f"({len(vals)} telemetry samples)")
        print(f"Error: {mean - ref_rpm:+.2f} rpm ({100 * (mean - ref_rpm) / ref_rpm:+.2f} %)")
    else:
        print("slot_rpm stayed 0 while moving: check the tab passes through the slot")
    print(f"saved {csv_path.relative_to(REPO)}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return
    plots = out_dir / "plots" if a.sim else REPO / "calibration" / "plots"
    plots.mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(figsize=(8, 4.2))
    ax.plot([r[0] - t0 for r in rows], [r[3] for r in rows], "-", color="#2b6cb0", lw=1.8,
            label="slot_rpm (slot sensor)")
    ax.axhline(ref_rpm, color="#c05621", ls="--", lw=1.4,
               label=f"Reference from stepper: {ref_rpm:.2f} rpm")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Speed (rpm)")
    ax.set_title(f"Slot sensor RPM vs stepper ({a.revs:g} revolutions)")
    ax.grid(alpha=0.3)
    ax.legend(loc="lower center")
    fig.tight_layout()
    path = plots / "slot_rpm.png"
    try:
        fig.savefig(path, dpi=150)
        print(f"wrote {path.relative_to(REPO)}")
    except OSError as e:
        print(f"Could not write {path.name} ({e}); close any viewer that has it open and rerun")
    plt.close(fig)


if __name__ == "__main__":
    main()
