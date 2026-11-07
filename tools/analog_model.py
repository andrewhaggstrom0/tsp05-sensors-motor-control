#!/usr/bin/env python3
"""Reference model of the Track 4 analog sensors (flex sensor on A0, thermistor on A1).

Mirrors firmware/tsp05/sensors_analog.cpp in Python so that:
  * the firmware math can be cross-checked (tools/check_firmware_vs_model.py),
  * plotting scripts can draw model curves,
  * the fake Arduino can produce physically realistic ADC counts
    (the inverse functions run physical units -> counts).

Constants are read from firmware/tsp05/config.h and the flex table from
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
FLEX_FIXED_R = C["FLEX_FIXED_R_OHMS"]
THERM_FIXED_R = C["THERM_FIXED_R_OHMS"]
THERM_R0 = C["THERM_R0_OHMS"]
THERM_T0_K = C["THERM_T0_K"]
THERM_BETA = C["THERM_BETA"]
THERM_ADC_SHORT = C["THERM_ADC_SHORT"]
THERM_ADC_OPEN = C["THERM_ADC_OPEN"]
FLEX_ADC_OPEN = C["FLEX_ADC_OPEN"]
FLEX_ADC_SHORT = C["FLEX_ADC_SHORT"]
FLEX_DEG_MIN = C["FLEX_DEG_MIN"]
FLEX_DEG_MAX = C["FLEX_DEG_MAX"]
FLEX_EMA_ALPHA = C["FLEX_EMA_ALPHA"]

FLEX_R_OHMS = _parse_float_array(MODULE_CPP, "FLEX_R_OHMS")
FLEX_DEG = _parse_float_array(MODULE_CPP, "FLEX_DEG")


# ----------------------------------------------------------------------------
# Forward: ADC counts -> physical units (same as the firmware)
# ----------------------------------------------------------------------------
def flex_ohms_from_adc(adc: float) -> float:
    """5V -> flex -> A0 -> R_fixed -> GND.  -1 means out of range."""
    if adc <= 0 or adc >= ADC_MAX:
        return -1.0
    return FLEX_FIXED_R * (ADC_MAX - adc) / adc


def _interp(x: float, xs: list[float], ys: list[float]) -> float:
    """Piecewise-linear, extrapolating the end segments (matches firmware)."""
    i = 1
    while i < len(xs) - 1 and x > xs[i]:
        i += 1
    x0, x1, y0, y1 = xs[i - 1], xs[i], ys[i - 1], ys[i]
    return y0 + (y1 - y0) * (x - x0) / (x1 - x0)


def flex_deg_from_ohms(ohms: float, r_table=None, deg_table=None) -> float:
    d = _interp(ohms, r_table or FLEX_R_OHMS, deg_table or FLEX_DEG)
    if not d >= FLEX_DEG_MIN:          # also catches NaN, like the firmware
        d = FLEX_DEG_MIN
    return min(d, FLEX_DEG_MAX)


def flex_deg_from_adc(adc: float) -> float:
    r = flex_ohms_from_adc(adc)
    return FLEX_DEG_MIN if r < 0 else flex_deg_from_ohms(r)


def flex_adc_is_fault(adc: float) -> bool:
    return adc <= FLEX_ADC_OPEN or adc >= FLEX_ADC_SHORT


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


def ema(prev: float, sample: float, alpha: float = FLEX_EMA_ALPHA) -> float:
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


def flex_ohms_from_deg(deg: float) -> float:
    """Inverse of the flex table (the table is monotonic, so swap the axes)."""
    return _interp(deg, FLEX_DEG, FLEX_R_OHMS)


def adc_from_flex_ohms(ohms: float) -> float:
    return ADC_MAX * FLEX_FIXED_R / (ohms + FLEX_FIXED_R)


def adc_from_flex_deg(deg: float) -> float:
    return adc_from_flex_ohms(flex_ohms_from_deg(deg))


def _main() -> None:
    print(f"Constants from {CONFIG_H.relative_to(REPO)}: B={THERM_BETA:g}, "
          f"R0={THERM_R0:g}, R_fixed(therm)={THERM_FIXED_R:g}, R_fixed(flex)={FLEX_FIXED_R:g}")
    print(f"Flex table (ohm -> deg): {list(zip(FLEX_R_OHMS, FLEX_DEG))}\n")
    print("Thermistor:  temp_C -> adc")
    for t in (0, 10, 20, 25, 30, 37, 45, 60):
        print(f"  {t:5.1f} C  -> {adc_from_therm_c(t):7.1f} counts")
    print("\nFlex:  bend deg -> adc")
    for d in (0, 15, 30, 45, 60, 75, 90):
        print(f"  {d:5.1f} deg -> {adc_from_flex_deg(d):7.1f} counts")


if __name__ == "__main__":
    _main()
