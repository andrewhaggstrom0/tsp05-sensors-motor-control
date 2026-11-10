#!/usr/bin/env python3
"""Measures steady DC motor speed at a sweep of PWM values, using firmware/bringup/dc_m1_encoder.

Steps the PWM down from 255 in each direction (the motor never stops, so no
breakaway is needed), waits for the speed to settle at each step, averages
the M1 sketch's counts_per_s readings, and writes calibration/data/dc_speed_map.csv.
The motor is always stopped at the end, including on Ctrl-C.

Examples:
  python3 tools/log_dc_speed_map.py
  python3 tools/log_dc_speed_map.py --port /dev/ttyACM0 --settle 2 --average 1.5
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
import time
from pathlib import Path

from log_serial import find_port

ROOT = Path(__file__).resolve().parent.parent
PWMS = [255, 230, 205, 180, 155, 130, 110, 95, 85, 75, 68, 63]
BANNER = "M1 encoder test"


def is_data(line: str, fields: int) -> bool:
    """True for a sketch data line: the expected number of numeric fields."""
    parts = line.split(",")
    try:
        return len(parts) == fields and all(float(p) is not None for p in parts)
    except ValueError:
        return False


def open_port(port: str, baud: int):
    try:
        import serial
    except ImportError:
        sys.exit("pyserial is not installed: pip install -r tools/requirements.txt")
    ser = serial.Serial(port, baud, timeout=1)
    ser.dtr = False
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.dtr = True
    deadline = time.time() + 4
    data_lines = 0
    while time.time() < deadline:
        line = ser.readline().decode(errors="replace").strip()
        if line.startswith(BANNER):
            return ser
        data_lines += is_data(line, 4)
        if data_lines >= 3:
            return ser
    sys.exit(f"No M1 banner from {port}. Is firmware/bringup/dc_m1_encoder flashed?")


def send(ser, pwm: int) -> None:
    ser.write(f"{pwm}\n".encode())


def read_cps(ser, seconds: float, pwm: int) -> list[float]:
    """Collects counts_per_s from lines whose pwm column matches the command."""
    values: list[float] = []
    end = time.time() + seconds
    while time.time() < end:
        parts = ser.readline().decode(errors="replace").strip().split(",")
        if len(parts) != 4:
            continue
        try:
            if int(parts[1]) == pwm:
                values.append(float(parts[3]))
        except ValueError:
            continue
    return values


def sweep(ser, sign: int, settle: float, average: float) -> list[dict]:
    rows = []
    for mag in PWMS:
        pwm = sign * mag
        send(ser, pwm)
        read_cps(ser, settle, pwm)
        cps = read_cps(ser, average, pwm)
        if not cps:
            print(f"  pwm {pwm:5d}: no readings")
            continue
        mean, std = statistics.mean(cps), statistics.pstdev(cps)
        rows.append({"direction": "forward" if sign > 0 else "reverse", "pwm": pwm,
                     "counts_per_s_mean": round(mean, 1), "counts_per_s_std": round(std, 1), "n": len(cps)})
        print(f"  pwm {pwm:5d}: {mean:7.1f} counts/s  (std {std:.1f}, n {len(cps)})")
        if abs(mean) < 1.0:
            print("  motor stopped; ending this direction")
            break
    send(ser, 0)
    time.sleep(1.5)
    return rows


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--settle", type=float, default=1.5, help="seconds to wait after each PWM change")
    p.add_argument("--average", type=float, default=1.0, help="seconds of readings averaged per PWM")
    p.add_argument("--out", type=Path, default=ROOT / "calibration" / "data" / "dc_speed_map.csv")
    args = p.parse_args()

    ser = open_port(args.port or find_port(), args.baud)
    rows: list[dict] = []
    try:
        for sign in (1, -1):
            print("forward:" if sign > 0 else "reverse:")
            rows += sweep(ser, sign, args.settle, args.average)
    finally:
        send(ser, 0)
        ser.close()

    if not rows:
        sys.exit("No samples recorded.")
    with args.out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(f"Wrote {len(rows)} points to {args.out}")


if __name__ == "__main__":
    main()
