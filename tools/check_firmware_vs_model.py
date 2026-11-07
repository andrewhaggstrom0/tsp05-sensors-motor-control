#!/usr/bin/env python3
"""Compare the firmware's conversions (C++, float) with tools/analog_model.py.

Builds firmware/tests/dump_conversions.cpp with g++, runs it for every ADC
value 0..1023, and checks each result against the Python model.
Exit code 0 = agree, 1 = mismatch.

Usage:
    python3 tools/check_firmware_vs_model.py
"""
import csv
import io
import math
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analog_model as m  # noqa: E402

TESTS = m.REPO / "firmware" / "tests"
BIN = TESTS / "build" / "dump_conversions"

TOL_C = 0.05   # degC: float (Uno) vs double (Python) rounding
TOL_DEG = 0.05  # degrees


def build_and_run() -> str:
    BIN.parent.mkdir(exist_ok=True)
    subprocess.run(
        ["g++", "-std=c++17", "-O1", "-I", str(TESTS / "shim"),
         "-I", str(m.REPO / "firmware" / "tsp05"),
         str(TESTS / "dump_conversions.cpp"),
         str(m.REPO / "firmware" / "tsp05" / "sensors_analog.cpp"),
         "-o", str(BIN)],
        check=True)
    return subprocess.run([str(BIN)], check=True, capture_output=True, text=True).stdout


def main() -> int:
    rows = list(csv.DictReader(io.StringIO(build_and_run())))
    bad = 0
    worst_c = worst_d = 0.0
    for r in rows:
        adc = int(r["adc"])
        fw_d, py_d = float(r["flex_deg"]), m.flex_deg_from_adc(adc)
        worst_d = max(worst_d, abs(fw_d - py_d))
        if abs(fw_d - py_d) > TOL_DEG:
            bad += 1
            print(f"Flex mismatch at adc {adc}: firmware {fw_d:.4f} deg, model {py_d:.4f} deg")
        if (r["flex_fault"] == "1") != m.flex_adc_is_fault(adc):
            bad += 1
            print(f"Flex fault mismatch at adc {adc}")

        fw_fault, py_fault = r["therm_fault"] == "1", m.therm_adc_is_fault(adc)
        if fw_fault != py_fault:
            bad += 1
            print(f"Fault mismatch at adc {adc}: firmware {fw_fault}, model {py_fault}")
        if not py_fault:
            fw_c, py_c = float(r["therm_C"]), m.therm_c_from_adc(adc)
            if not math.isfinite(fw_c):
                bad += 1
                print(f"Non-finite firmware temp at adc {adc}")
                continue
            worst_c = max(worst_c, abs(fw_c - py_c))
            if abs(fw_c - py_c) > TOL_C:
                bad += 1
                print(f"Temp mismatch at adc {adc}: firmware {fw_c:.4f} C, model {py_c:.4f} C")

    # Round trip: model inverse then forward should return the input
    for t in (0, 10, 25, 37, 50):
        back = m.therm_c_from_adc(m.adc_from_therm_c(t))
        if abs(back - t) > 1e-6:
            bad += 1
            print(f"Thermistor round trip failed: {t} -> {back}")
    for d in (0, 15, 45, 90):
        back = m.flex_deg_from_adc(m.adc_from_flex_deg(d))
        if abs(back - d) > 1e-6:
            bad += 1
            print(f"Flex round trip failed: {d} -> {back}")

    print(f"{len(rows)} ADC values checked; worst diff {worst_c:.4f} C, {worst_d:.4f} deg; "
          f"{bad} problems")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
