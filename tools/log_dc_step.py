#!/usr/bin/env python3
"""Records a DC motor step response from firmware/bringup/dc_pid.

Sends one command (e.g. p90 or v60), records the stream for --seconds, and
writes calibration/data/dc_step_<command>.csv for plot_calibration.py --timeseries.

Examples:
  python3 tools/log_dc_step.py p90
  python3 tools/log_dc_step.py v60 --seconds 4 --port /dev/ttyACM0
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

from log_serial import find_port

ROOT = Path(__file__).resolve().parent.parent
FIELDS = ["ms", "mode", "target", "deg", "rpm", "pwm"]


def open_port(port: str, baud: int):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is not installed: pip install -r tools/requirements.txt")
    ser = serial.Serial(port, baud, timeout=1)
    deadline = time.time() + 4
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith("# tsp05 dc bringup"):
            return ser
    sys.exit(f"No dc bring-up banner from {port}. Is firmware/bringup/dc_pid flashed?")


def record(ser, command: str, seconds: float, pre_s: float) -> list[dict]:
    rows: list[dict] = []
    sent = False
    start = time.time()
    while time.time() - start < pre_s + seconds:
        if not sent and time.time() - start >= pre_s:
            ser.write((command + "\n").encode())
            sent = True
        line = ser.readline().decode(errors="replace").strip()
        if not line or line.startswith("#") or line.startswith("ms"):
            continue
        parts = line.split(",")
        if len(parts) != len(FIELDS):
            continue
        try:
            rows.append(dict(zip(FIELDS, (float(p) for p in parts))))
        except ValueError:
            continue
    ser.write(b"x\n")
    return rows


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("command", help="p<deg> or v<rpm>, e.g. p90 or v-60")
    p.add_argument("--seconds", type=float, default=3.0)
    p.add_argument("--pre", type=float, default=0.5, help="seconds recorded before the step")
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--out", type=Path)
    args = p.parse_args()

    if args.command[:1] not in ("p", "v"):
        sys.exit("command must start with p (position) or v (velocity)")
    out = args.out or ROOT / "calibration" / "data" / f"dc_step_{args.command.replace('-', 'neg')}.csv"

    ser = open_port(args.port or find_port(), args.baud)
    rows = record(ser, args.command, args.seconds, args.pre)
    if not rows:
        sys.exit("No samples received.")

    t0 = rows[0]["ms"]
    with out.open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t_s", "target", "deg", "rpm", "pwm"])
        for r in rows:
            w.writerow([f"{(r['ms'] - t0) / 1000:.3f}", r["target"], r["deg"], r["rpm"], int(r["pwm"])])
    print(f"Wrote {len(rows)} samples to {out}")
    series = "target deg" if args.command[0] == "p" else "target rpm"
    shown = out.resolve().relative_to(ROOT) if out.resolve().is_relative_to(ROOT) else out
    print(f"Plot: python3 tools/plot_calibration.py --timeseries {shown} --series {series}")


if __name__ == "__main__":
    main()
