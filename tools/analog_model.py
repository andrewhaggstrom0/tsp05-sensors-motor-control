#!/usr/bin/env python3
"""Reference model of the Track 4 analog sensors (FSR on A0, thermistor on A1).

Mirrors firmware/tsp05/sensors_analog.cpp in Python so that:
  * the firmware math can be cross-checked (tools/check_firmware_vs_model.py),
  * plotting scripts can draw model curves,
  * the fake Arduino can produce physically realistic ADC counts
    (the inverse functions run physical units -> counts).

Constants are read from firmware/tsp05/config.h and the FSR table from
firmware/tsp05/sensors_analog.cpp, so there is one source of truth.
Standard library only.

Usage:
    python3 tools/analog_model.py            # print a reference table
"""
from __future__ import annotations

import math
import re
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
CONFIG_H = REPO / "firmware" / "tsp05" / "config.h"
MODULE_CPP = REPO / "firmware" / "tsp05" / "sensors_analog.cpp"


def _parse_defines(path: Path) -> dict[str, float]:
    """Numeric #defines from a C header: '#define NAME 123.0f // comment'."""
    out: dict[str, float] = {}
    pat = re.compile(r"^\s*#define\s+(\w+)\s+(-?[\d.]+)f?\b")
    for line in path.read_text().splitlines():
        m = pat.match(line)
        if m:
            out[m.group(1)] = float(m.group(2))
    return out


def _parse_float_array(path: Path, name: str) -> list[float]:
    """Values of 'static const float NAME[] = {a, b, c};'."""
    m = re.search(rf"{name}\[\]\s*=\s*\{{([^}}]*)\}}", path.read_text())
    if not m:
        raise ValueError(f"{name} not found in {path}")
    return [float(v.strip().rstrip("f")) for v in m.group(1).split(",") if v.strip()]


C = _parse_defines(CONFIG_H)
ADC_MAX = C["ADC_MAX"]
FSR_FIXED_R = C["FSR_FIXED_R_OHMS"]
THERM_FIXED_R = C["THERM_FIXED_R_OHMS"]
THERM_R0 = C["THERM_R0_OHMS"]
THERM_T0_K = C["THERM_T0_K"]
THERM_BETA = C["THERM_BETA"]
THERM_ADC_SHORT = C["THERM_ADC_SHORT"]
THERM_ADC_OPEN = C["THERM_ADC_OPEN"]
FSR_ADC_DEADZONE = C["FSR_ADC_DEADZONE"]
FSR_MAX_N = C["FSR_MAX_N"]
FSR_EMA_ALPHA = C["FSR_EMA_ALPHA"]

FSR_G_US = _parse_float_array(MODULE_CPP, "FSR_G_US")
FSR_N = _parse_float_array(MODULE_CPP, "FSR_N")


# ----------------------------------------------------------------------------
# Forward: ADC counts -> physical units (same as the firmware)
# ----------------------------------------------------------------------------
def fsr_ohms_from_adc(adc: float) -> float:
    """5V -> FSR -> A0 -> R_fixed -> GND.  -1 means open (no press)."""
    if adc <= 0:
        return -1.0
    if adc >= ADC_MAX:
        return 0.0
    return FSR_FIXED_R * (ADC_MAX - adc) / adc


def _interp(x: float, xs: list[float], ys: list[float]) -> float:
    """Piecewise-linear, extrapolating the last segment (matches firmware)."""
    i = 1
    while i < len(xs) - 1 and x > xs[i]:
        i += 1
    x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def fsr_newtons_from_adc(adc: float) -> float:
    if adc <= FSR_ADC_DEADZONE:
        return 0.0
    r = max(fsr_ohms_from_adc(adc), 1.0)
    g_us = 1e6 / r
    return min(max(_interp(g_us, FSR_G_US, FSR_N), 0.0), FSR_MAX_N)


def therm_ohms_from_adc(adc: float) -> float:
    """5V -> R_fixed -> A1 -> thermistor -> GND.  -1 means out of range."""
    if adc <= 0 or adc >= ADC_MAX:
        return -1.0
    return THERM_FIXED_R * adc / (ADC_MAX - adc)


def therm_c_from_ohms(ohms: float) -> float:
    if ohms <= 0:
        return math.nan
    inv_t = 1.0 / THERM_T0_K + math.log(ohms / THERM_R0) / THERM_BETA
    return 1.0 / inv_t - 273.15


def therm_c_from_adc(adc: float) -> float:
    return therm_c_from_ohms(therm_ohms_from_adc(adc))


def therm_adc_is_fault(adc: float) -> bool:
    return adc <= THERM_ADC_SHORT or adc >= THERM_ADC_OPEN


def ema(prev: float, sample: float, alpha: float = FSR_EMA_ALPHA) -> float:
    return prev + alpha * (sample - prev)


# ----------------------------------------------------------------------------
# Inverse: physical units -> ADC counts (for the fake Arduino and synthetic
# calibration data). Returns a float; round it yourself if you need counts.
# ----------------------------------------------------------------------------
def therm_ohms_from_c(temp_c: float) -> float:
    t_k = temp_c + 273.15
    return THERM_R0 * math.exp(THERM_BETA * (1.0 / t_k - 1.0 / THERM_T0_K))


def adc_from_therm_c(temp_c: float) -> float:
    r = therm_ohms_from_c(temp_c)
    return ADC_MAX * r / (THERM_FIXED_R + r)


def adc_from_fsr_newtons(force_n: float) -> float:
    """Inverse of the FSR table. 0 N -> 0 counts (FSR open)."""
    if force_n <= 0:
        return 0.0
    force_n = min(force_n, FSR_MAX_N)
    g_us = _interp(force_n, FSR_N, FSR_G_US)   # table is monotonic, so invert it
    r = 1e6 / g_us
    return ADC_MAX * FSR_FIXED_R / (r + FSR_FIXED_R)


def _main() -> None:
    print(f"Constants from {CONFIG_H.relative_to(REPO)}: B={THERM_BETA:g}, "
          f"R0={THERM_R0:g}, R_fixed(therm)={THERM_FIXED_R:g}, R_fixed(fsr)={FSR_FIXED_R:g}")
    print(f"FSR table (uS -> N): {list(zip(FSR_G_US, FSR_N))}\n")
    print("Thermistor:  temp_C -> adc")
    for t in (0, 10, 20, 25, 30, 37, 45, 60):
        print(f"  {t:5.1f} C  -> {adc_from_therm_c(t):7.1f} counts")
    print("\nFSR:  force_N -> adc")
    for f in (0, 0.5, 1, 2, 5, 10, 20, 50):
        print(f"  {f:5.1f} N  -> {adc_from_fsr_newtons(f):7.1f} counts")


if __name__ == "__main__":
    _main()
