#!/usr/bin/env python3
"""Draw the Track 4 analog wiring (flex sensor on A0, thermistor on A1).

Resistor values come from firmware/tsp05/config.h, so the figure always shows
the parts actually in use. Writes docs/figures/analog_wiring.png.

    pip install schemdraw matplotlib
    python3 tools/draw_wiring.py
"""
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import schemdraw  # noqa: E402
import schemdraw.elements as elm  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analog_model as m  # noqa: E402

OUT = m.REPO / "docs" / "figures" / "analog_wiring.png"
INK, MUTED, ACCENT = "#0b0b0b", "#52514e", "#2a78d6"


def kohm(r: float) -> str:
    return f"{r / 1000:.2f} k\u03a9"


schemdraw.config(fontsize=11, color=INK, lw=1.6, margin=0.6)
with schemdraw.Drawing(show=False) as d:
    # ---- Flex sensor divider -> A0 --------------------------------------
    d += elm.Vdd().label("5V")
    d += elm.ResistorVar().down().label(["Flex sensor", "Adafruit 182", "~11 k\u03a9 flat,", "~23 k\u03a9 at 90\u00b0"],
                                        loc="bottom", ofst=0.3)
    node_a0 = d.here
    d += elm.Dot()
    d += elm.Resistor().down().label(["R_fixed", kohm(m.FLEX_FIXED_R)], loc="bottom", ofst=0.3)
    d += elm.Ground()
    d += elm.Line().at(node_a0).left().length(1.5)
    d += elm.Dot(open=True).label("A0", loc="left", color=ACCENT)

    # ---- Thermistor divider -> A1 ---------------------------------------
    d += elm.Vdd().at((6.5, 0)).label("5V")
    d += elm.Resistor().down().label(["R_fixed", kohm(m.THERM_FIXED_R)], loc="bottom", ofst=0.3)
    node_a1 = d.here
    d += elm.Dot()
    d += elm.Thermistor().down().label(["Thermistor", "10 k\u03a9 NTC @ 25 \u00b0C", f"B = {m.THERM_BETA:g} K",
                                        "(long leads)"], loc="bottom", ofst=0.3)
    d += elm.Ground()
    d += elm.Line().at(node_a1).right().length(1.5)
    d += elm.Dot(open=True).label("A1", loc="right", color=ACCENT)

    # ---- Notes ------------------------------------------------------------
    d += elm.Label().at((3.25, -8.6)).label(
        "Bend \u2192 R_flex rises \u2192 A0 falls      Warmer \u2192 R_therm falls \u2192 A1 falls",
        color=MUTED)
    d += elm.Label().at((3.25, -9.3)).label(
        "Both dividers share the Uno's 5V and GND (ratiometric: supply drift cancels)",
        color=MUTED)
    d.save(str(OUT), dpi=200)
print(f"wrote {OUT.relative_to(m.REPO)}")
