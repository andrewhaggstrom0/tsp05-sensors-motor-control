#!/usr/bin/env python3
"""Turn calibration CSVs into the graded figures and error tables.

One command regenerates everything from whatever CSVs exist:

    python3 tools/plot_calibration.py

  calibration/data/thermistor.csv  -> plots/thermistor_transfer.png  (graded format:
                                      degC on x, ADC counts on y) + error table
  calibration/data/fsr.csv         -> plots/fsr_calibration.png + fitted firmware
                                      table + error table
  calibration/data/fsr_filter.csv  -> plots/fsr_filter.png (raw vs EMA filtered)
  all tables                       -> calibration/results.md (paste into README)

Conversions come from tools/analog_model.py, which reads config.h and the FSR
table in sensors_analog.cpp, so the figures always match the firmware.

Other options:
    --data calibration/sim_data        rehearse on simulated data (outputs stay
                                       in calibration/sim_data/, git-ignored)
    --apply-fsr-table                  write the fitted FSR table into
                                       firmware/tsp05/sensors_analog.cpp
    --alphas 0.1 0.4                   extra EMA alphas to compare on the filter plot

Any time series CSV, including one exported from Foxglove (other tracks: TOF
raw vs filtered, DC step response):
    python3 tools/plot_calibration.py --timeseries export.csv \
        --series tof/raw_mm tof/filt_mm --title "TOF filter" --ylabel mm

Needs numpy and matplotlib (pip install -r tools/requirements.txt).
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import math
import re
import sys
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("Agg")  # no display needed
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analog_model as m  # noqa: E402

DATA_DIR = m.REPO / "calibration" / "data"
CAL_DIR = m.REPO / "calibration"

# ----------------------------------------------------------------------------
# Figure style: one surface, recessive axes, fixed series order (never cycled)
# ----------------------------------------------------------------------------
SURFACE = "#fcfcfb"
INK = "#0b0b0b"
INK_2 = "#52514e"
GRID = "#e4e3df"
SERIES = ["#2a78d6", "#eb6834", "#1baf7a", "#6250d6"]  # blue, orange, aqua, violet
MODEL = INK_2  # model / fit curves: neutral ink, so data keeps the colour

plt.rcParams.update({
    "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
    "axes.edgecolor": GRID, "axes.labelcolor": INK_2, "axes.titlecolor": INK,
    "axes.titlesize": 12, "axes.titleweight": "bold", "axes.titlelocation": "left",
    "axes.labelsize": 10, "axes.grid": True, "grid.color": GRID, "grid.linewidth": 0.8,
    "axes.spines.top": False, "axes.spines.right": False,
    "xtick.color": INK_2, "ytick.color": INK_2, "xtick.labelsize": 9, "ytick.labelsize": 9,
    "legend.frameon": False, "legend.fontsize": 9, "legend.labelcolor": INK_2,
    "lines.linewidth": 2, "lines.solid_capstyle": "round", "font.size": 10,
})


def new_fig(title: str, subtitle: str = ""):
    fig, ax = plt.subplots(figsize=(7.2, 4.4))
    ax.set_title(title, pad=22 if subtitle else 10)
    if subtitle:
        ax.text(0, 1.02, subtitle, transform=ax.transAxes, color=INK_2, fontsize=9,
                va="bottom")
    return fig, ax


def save(fig, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=200)
    plt.close(fig)
    print(f"  wrote {show(path)}")


def show(path: Path) -> str:
    try:
        return str(path.resolve().relative_to(m.REPO))
    except ValueError:
        return str(path)


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="") as f:
        return list(csv.DictReader(f))


def md_table(header: list[str], rows: list[list]) -> str:
    out = ["| " + " | ".join(header) + " |", "|" + "---|" * len(header)]
    out += ["| " + " | ".join(str(c) for c in r) + " |" for r in rows]
    return "\n".join(out)


# ----------------------------------------------------------------------------
# Thermistor
# ----------------------------------------------------------------------------
def thermistor(data: Path, plots: Path) -> str:
    rows = read_csv(data)
    ref = np.array([float(r["ref_C"]) for r in rows])
    adc = np.array([float(r["adc_mean"]) for r in rows])
    std = np.array([float(r["adc_std"]) for r in rows])
    if len(ref) < 2:
        sys.exit(f"{data}: need at least 2 points")
    model = np.array([m.therm_c_from_adc(a) for a in adc])
    err = model - ref

    # Best-fit B (least squares on 1/T - 1/T0 = ln(R/R0) / B), with
    # leave-one-out error so the fitted-B number is not graded on its own data.
    x = np.log(np.array([m.therm_ohms_from_adc(a) for a in adc]) / m.THERM_R0)
    y = 1.0 / (ref + 273.15) - 1.0 / m.THERM_T0_K

    def beta_c(inv_b, xi):
        return 1.0 / (1.0 / m.THERM_T0_K + xi * inv_b) - 273.15

    inv_b = float(x @ y / (x @ x))
    loo = []
    for i in range(len(x)):
        keep = np.arange(len(x)) != i
        ib = float(x[keep] @ y[keep] / (x[keep] @ x[keep]))
        loo.append(beta_c(ib, x[i]) - ref[i])
    loo = np.array(loo)

    # Figure: graded format, degC on x, ADC counts on y
    fig, ax = new_fig("Thermistor transfer function",
                      f"A1 reading vs reference temperature. Beta model: B = {m.THERM_BETA:g} K, "
                      f"R0 = {m.THERM_R0 / 1000:g} kΩ, {m.THERM_FIXED_R / 1000:g} kΩ divider")
    t = np.linspace(min(ref.min(), 0) - 5, max(ref.max(), 40) + 5, 300)
    ax.plot(t, [m.adc_from_therm_c(v) for v in t], color=MODEL, linewidth=1.5,
            label=f"Beta model (B = {m.THERM_BETA:g} K)")
    # Sample std is ~1 count, far smaller than the markers; it is in the table.
    ax.plot(ref, adc, "o", color=SERIES[0], markersize=7, markeredgecolor=SURFACE,
            markeredgewidth=1.5, label="Measured (mean of samples)", zorder=3)
    ax.set_xlabel("Reference temperature (°C)")
    ax.set_ylabel("A1 reading (ADC counts)")
    ax.set_ylim(0, 1023)
    ax.legend(loc="upper right")
    save(fig, plots / "thermistor_transfer.png")

    rms = float(np.sqrt(np.mean(err ** 2)))
    table = md_table(
        ["Reference (°C)", "ADC counts", "Beta model (°C)", "Error (°C)"],
        [[f"{r:.1f}", f"{a:.1f} ± {s:.1f}", f"{c:.2f}", f"{e:+.2f}"]
         for r, a, s, c, e in zip(ref, adc, std, model, err)])
    summary = (f"Datasheet B = {m.THERM_BETA:g} K: mean error {err.mean():+.2f} °C, "
               f"max |error| {np.abs(err).max():.2f} °C, RMS {rms:.2f} °C "
               f"over {len(ref)} points.\n\n"
               f"Best-fit B from this data = {1 / inv_b:.0f} K. Leave-one-out RMS error with "
               f"a fitted B = {np.sqrt(np.mean(loo ** 2)):.2f} °C (each point predicted by a "
               f"B fitted without it, so this is not graded on its own data).")
    print(f"  thermistor: RMS {rms:.2f} C with datasheet B; best-fit B {1 / inv_b:.0f}")
    return (f"### Thermistor\n\n![Thermistor transfer function](plots/thermistor_transfer.png)"
            f"\n\n{table}\n\n{summary}\n")


# ----------------------------------------------------------------------------
# FSR
# ----------------------------------------------------------------------------
def conductance_us(adc: float) -> float:
    if adc <= m.FSR_ADC_DEADZONE:
        return 0.0
    return 1e6 / max(m.fsr_ohms_from_adc(adc), 1.0)


def fit_fsr(g: np.ndarray, f: np.ndarray) -> dict:
    """Force vs conductance through the origin: a straight line, or two
    straight segments joined at a knee. Chosen by leave-one-out RMS so the
    extra knee only wins if it predicts unseen points better."""

    def design(gv, knee):
        if knee is None:
            return gv[:, None]
        return np.column_stack([np.minimum(gv, knee), np.maximum(gv - knee, 0.0)])

    def solve(gv, fv, knee):
        coef, *_ = np.linalg.lstsq(design(gv, knee), fv, rcond=None)
        return coef

    def loo_rms(knee):
        e = []
        for i in range(len(g)):
            keep = np.arange(len(g)) != i
            c = solve(g[keep], f[keep], knee)
            if np.any(c <= 0):
                return math.inf
            e.append((design(g[i:i + 1], knee) @ c)[0] - f[i])
        return float(np.sqrt(np.mean(np.square(e))))

    best = {"knee": None, "loo": loo_rms(None)}
    gs = np.unique(g[g > 0])
    for knee in gs[2:-2]:   # at least 2 distinct points on each side
        r = loo_rms(knee)
        if r < 0.9 * best["loo"]:   # must clearly beat the simpler model
            best = {"knee": float(knee), "loo": r}
    coef = solve(g, f, best["knee"])
    if np.any(coef <= 0):
        best["knee"], coef = None, solve(g, f, None)
        best["loo"] = loo_rms(None)
    best["coef"] = coef
    best["predict"] = lambda gv: design(np.atleast_1d(np.asarray(gv, float)), best["knee"]) @ coef

    # Firmware table: breakpoints in microsiemens -> newtons
    g_end = float(np.ceil(g.max() / 100.0) * 100.0)
    pts = [0.0] + ([best["knee"]] if best["knee"] else []) + [g_end]
    best["table_g"] = [round(p, 1) for p in pts]
    best["table_n"] = [round(float(best["predict"](p)[0]), 3) for p in pts]
    return best


def fsr(data: Path, plots: Path) -> tuple[str, dict]:
    rows = read_csv(data)
    force = np.array([float(r["force_N"]) for r in rows])
    adc = np.array([float(r["adc_mean"]) for r in rows])
    direction = np.array([r["direction"] for r in rows])
    trial = np.array([int(r["trial"]) for r in rows])
    mass = np.array([float(r["mass_g"]) for r in rows])
    g = np.array([conductance_us(a) for a in adc])
    if np.count_nonzero(force > 0) < 3:
        sys.exit(f"{data}: need at least 3 loaded points")

    fit = fit_fsr(g, force)
    pred = fit["predict"](g)
    err = pred - force

    fig, ax = new_fig("FSR calibration: force vs conductance",
                      f"Conductance G = 1/R_fsr from A0 with a {m.FSR_FIXED_R / 1000:g} kΩ "
                      f"divider. {len(np.unique(trial))} trial(s), {len(force)} points")
    gg = np.linspace(0, g.max() * 1.05, 300)
    kind = "two-segment fit" if fit["knee"] else "linear fit"
    ax.plot(gg, fit["predict"](gg), color=MODEL, linewidth=1.5, label=f"Firmware table ({kind})")
    for d, color, marker in (("load", SERIES[0], "o"), ("unload", SERIES[1], "^")):
        sel = direction == d
        if sel.any():
            ax.plot(g[sel], force[sel], marker, color=color, markersize=6.5,
                    markeredgecolor=SURFACE, markeredgewidth=1.2,
                    label=f"Measured, {d}ing", zorder=3)
    ax.set_xlabel("FSR conductance (µS)")
    ax.set_ylabel("Applied force (N)")
    ax.set_xlim(left=0)
    ax.set_ylim(bottom=0)
    ax.legend(loc="upper left")
    save(fig, plots / "fsr_calibration.png")

    # Hysteresis: same trial and mass, unloading vs loading
    hyst = []
    for t in np.unique(trial):
        for ms in np.unique(mass[(trial == t) & (mass > 0)]):
            ld = (trial == t) & (mass == ms) & (direction == "load")
            ul = (trial == t) & (mass == ms) & (direction == "unload")
            if ld.any() and ul.any():
                applied = force[ld].mean()
                hyst.append((pred[ul].mean() - pred[ld].mean()) / applied)
    loaded = force > 0
    pct = 100 * err[loaded] / force[loaded]
    rms = float(np.sqrt(np.mean(err ** 2)))

    table = md_table(
        ["Trial", "Direction", "Mass (g)", "Reference (N)", "ADC counts", "Fit (N)", "Error (N)", "Error (%)"],
        [[t, d, f"{ms:g}", f"{fr:.3f}", f"{a:.1f}", f"{p:.3f}", f"{e:+.3f}",
          f"{100 * e / fr:+.1f}" if fr > 0 else "n/a"]
         for t, d, ms, fr, a, p, e in zip(trial, direction, mass, force, adc, pred, err)])
    summary = (f"Fit: {kind} of force vs conductance through the origin. RMS error "
               f"{rms:.3f} N, mean |error| {np.mean(np.abs(pct)):.1f} % of the applied force "
               f"(loaded points), leave-one-out RMS {fit['loo']:.3f} N.")
    if hyst:
        summary += (f" Hysteresis: unloading reads {100 * np.mean(hyst):+.1f} % of the applied "
                    f"force relative to loading at the same mass (mean over {len(hyst)} pairs).")
    print(f"  fsr: {kind}, RMS {rms:.3f} N, mean |error| {np.mean(np.abs(pct)):.1f} %")

    snippet = ("static const float FSR_G_US[] = {" + ", ".join(f"{v:.1f}f" for v in fit["table_g"])
               + "};\nstatic const float FSR_N[]    = {"
               + ", ".join(f"{v:.3f}f" for v in fit["table_n"]) + "};")
    md = (f"### FSR\n\n![FSR calibration](plots/fsr_calibration.png)\n\n{table}\n\n{summary}\n\n"
          f"Firmware table (`firmware/tsp05/sensors_analog.cpp`):\n\n```cpp\n{snippet}\n```\n")
    return md, fit


def apply_fsr_table(fit: dict, source: Path) -> None:
    cpp = m.REPO / "firmware" / "tsp05" / "sensors_analog.cpp"
    s = cpp.read_text()
    gv = ", ".join(f"{v:.1f}f" for v in fit["table_g"])
    nv = ", ".join(f"{v:.3f}f" for v in fit["table_n"])
    s, n1 = re.subn(r"static const float FSR_G_US\[\] = \{[^}]*\};",
                    f"static const float FSR_G_US[] = {{{gv}}};", s)
    s, n2 = re.subn(r"static const float FSR_N\[\]\s*= \{[^}]*\};",
                    f"static const float FSR_N[]    = {{{nv}}};", s)
    if n1 != 1 or n2 != 1:
        sys.exit("Could not find the FSR table in sensors_analog.cpp; not changed.")
    stamp = f"Fitted by tools/plot_calibration.py from {show(source)} on {dt.date.today()}."
    # First apply: swap the placeholder note. Later applies: refresh the stamp line.
    s = re.sub(r"PLACEHOLDER from the Adafruit FSR guide's approximation\. Replace with the\n"
               r"// fit from step A4 \(calibration/data/fsr\.csv\)\. ",
               stamp + "\n// ", s, count=1)
    s = re.sub(r"Fitted by tools/plot_calibration\.py [^\n]*? on \d{4}-\d{2}-\d{2}\.", stamp, s, count=1)
    cpp.write_text(s)
    print(f"  updated FSR table in {show(cpp)}. Now run firmware/tests/run_tests.sh and "
          "python3 tools/check_firmware_vs_model.py")


# ----------------------------------------------------------------------------
# FSR filter
# ----------------------------------------------------------------------------
def ema_series(x: np.ndarray, alpha: float) -> np.ndarray:
    y = np.empty_like(x)
    y[0] = x[0]
    for i in range(1, len(x)):
        y[i] = y[i - 1] + alpha * (x[i] - y[i - 1])
    return y


def fsr_filter(data: Path, plots: Path, extra_alphas: list[float]) -> str:
    rows = read_csv(data)
    t = np.array([float(r["t_s"]) for r in rows])
    raw = np.array([m.fsr_newtons_from_adc(float(r["fsr_adc"])) for r in rows])
    dt_s = float(np.median(np.diff(t)))

    def tau_ms(a):
        return -1000 * dt_s / math.log(1 - a)

    alphas = [m.FSR_EMA_ALPHA] + [a for a in extra_alphas if a != m.FSR_EMA_ALPHA]

    # Noise is measured only while the force is held steady: skip released
    # samples, real changes (presses), and the time the slowest filter needs
    # to catch up afterwards (5 time constants). Otherwise the press edges,
    # which are signal, would be counted as noise.
    d = np.diff(raw)
    pressed = (raw[:-1] > 0) & (raw[1:] > 0)
    sigma = 1.4826 * float(np.median(np.abs(d[pressed]))) if pressed.any() else 0.0
    change = np.abs(d) > max(6 * sigma, 0.05)
    settle = int(math.ceil(5 * max(tau_ms(a) for a in alphas[:3]) / 1000 / dt_s))
    busy = np.zeros(len(d), bool)
    for i in np.flatnonzero(change):
        busy[max(i - 1, 0): i + settle + 1] = True
    hold = pressed & ~busy

    fig, ax = new_fig("FSR filter: raw vs EMA filtered",
                      f"Force from A0 at {1 / dt_s:.0f} Hz through the firmware table. "
                      f"EMA: y += α(x − y)")
    ax.plot(t, raw, color=SERIES[0], linewidth=1, alpha=0.55, label="Raw")
    lines = []
    for i, a in enumerate(alphas[:3]):
        y = ema_series(raw, a)
        tag = " (firmware)" if i == 0 else ""
        ax.plot(t, y, color=SERIES[1 + i], linewidth=2,
                label=f"EMA α = {a:g}{tag}, τ ≈ {tau_ms(a):.0f} ms")
        if hold.sum() >= 10:
            n_raw = float(np.std(d[hold]))
            n_f = float(np.std(np.diff(y)[hold]))
            lines.append(f"| {a:g}{tag} | {tau_ms(a):.0f} | {n_raw:.3f} | {n_f:.3f} | "
                         f"{100 * (1 - n_f / n_raw):.0f} % |")
        else:
            lines.append(f"| {a:g}{tag} | {tau_ms(a):.0f} | n/a | n/a | hold the force "
                         "steady for a second or two to measure |")
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Force (N)")
    ax.set_ylim(0, max(raw.max(), 1e-3) * 1.35)   # headroom so the legend clears the data
    ax.legend(loc="upper left", ncol=2)
    save(fig, plots / "fsr_filter.png")
    table = ("| α | Time constant (ms) | Raw noise (N) | Filtered noise (N) | Noise reduction |\n"
             "|---|---|---|---|---|\n" + "\n".join(lines))
    return (f"### FSR filter\n\n![FSR filter](plots/fsr_filter.png)\n\n{table}\n\n"
            "Noise = standard deviation of the sample-to-sample change while the force is "
            f"held steady ({int(hold.sum())} samples; releases, press edges and filter settling "
            "excluded). The time constant τ = −Δt / ln(1 − α) is the lag the filter adds: "
            "less noise costs a slower response.\n")


# ----------------------------------------------------------------------------
# Generic time series (Foxglove exports and our own CSVs)
# ----------------------------------------------------------------------------
TIME_HINTS = ("elapsed", "time", "stamp", "t_s", "ms", "sec")


def _to_float(v: str) -> float | None:
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


def load_timeseries(path: Path) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """Returns {series_name: (t_seconds_from_start, values)}.

    Accepts "wide" CSVs (one time column + one column per series, like our own
    files) and "long" CSVs (rows of time, topic/path, value, which is how plot
    exports usually come out). Column names are matched loosely, so check the
    result against a real Foxglove export and adjust TIME_HINTS if needed.
    """
    rows = read_csv(path)
    if not rows:
        sys.exit(f"{path} is empty")
    cols = list(rows[0].keys())
    low = {c: c.lower() for c in cols}
    time_col = next((c for h in TIME_HINTS for c in cols if h in low[c]), cols[0])
    name_col = next((c for c in cols if any(k in low[c] for k in ("topic", "path", "series"))), None)
    value_col = next((c for c in cols if "value" in low[c]), None)

    series: dict[str, list[tuple[float, float]]] = {}
    if name_col and value_col:                       # long format
        for r in rows:
            tv, vv = _to_float(r[time_col]), _to_float(r[value_col])
            if tv is not None and vv is not None:
                series.setdefault(r[name_col], []).append((tv, vv))
    else:                                            # wide format
        for c in cols:
            if c == time_col:
                continue
            pts = [(_to_float(r[time_col]), _to_float(r[c])) for r in rows]
            pts = [(a, b) for a, b in pts if a is not None and b is not None]
            if pts:
                series[c] = pts
    if not series:
        sys.exit(f"No numeric series found in {path} (columns: {cols})")

    t0 = min(p[0][0] for p in series.values())
    span = max(p[-1][0] for p in series.values()) - t0
    # Guess units from the span of a short recording: ns, us, ms or s
    scale = 1e-9 if span > 1e8 else 1e-6 if span > 1e5 else 1e-3 if span > 1e3 else 1.0
    return {k: ((np.array([a for a, _ in v]) - t0) * scale, np.array([b for _, b in v]))
            for k, v in series.items()}


def timeseries(path: Path, wanted: list[str], out: Path, title: str, ylabel: str) -> None:
    series = load_timeseries(path)
    if wanted:
        picked = {}
        for w in wanted:
            hits = [k for k in series if w in k]
            if not hits:
                sys.exit(f"No series matching {w!r}. Available: {list(series)}")
            picked.update({k: series[k] for k in hits})
        series = picked
    if len(series) > len(SERIES):
        sys.exit(f"{len(series)} series on one chart is too many to read; pick up to "
                 f"{len(SERIES)} with --series. Available: {list(series)}")
    fig, ax = new_fig(title or path.stem, f"From {path.name}")
    for i, (name, (t, v)) in enumerate(series.items()):
        ax.plot(t, v, color=SERIES[i], linewidth=1.2 if i == 0 and len(series) > 1 else 2,
                label=name)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    if len(series) > 1:
        ax.legend(loc="best")
    save(fig, out)


# ----------------------------------------------------------------------------
def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--data", type=Path, default=DATA_DIR, help="folder with the CSVs")
    p.add_argument("--plots", type=Path, help="output folder for PNGs")
    p.add_argument("--apply-fsr-table", action="store_true",
                   help="write the fitted FSR table into sensors_analog.cpp")
    p.add_argument("--alphas", type=float, nargs="*", default=[],
                   help="extra EMA alphas to compare on the filter plot")
    p.add_argument("--timeseries", type=Path, help="plot any time series CSV instead")
    p.add_argument("--series", nargs="*", default=[], help="timeseries: names to plot (substring)")
    p.add_argument("--title", default="", help="timeseries: chart title")
    p.add_argument("--ylabel", default="Value", help="timeseries: y axis label")
    p.add_argument("--out", type=Path, help="timeseries: output PNG")
    args = p.parse_args()

    if args.timeseries:
        out = args.out or (CAL_DIR / "plots" / f"{args.timeseries.stem}.png")
        timeseries(args.timeseries, args.series, out, args.title, args.ylabel)
        return

    data = args.data.resolve()
    real = data == DATA_DIR.resolve()
    # Real data -> calibration/plots + calibration/results.md (committed).
    # Anything else (e.g. sim_data) -> outputs inside that folder.
    plots = args.plots or (CAL_DIR / "plots" if real else data / "plots")
    results = (CAL_DIR if real else data) / "results.md"

    print(f"Reading {show(data)}")
    parts = [f"## Calibration Results\n\nGenerated by `tools/plot_calibration.py` from "
             f"`{show(data)}` on {dt.date.today()}. Rerun it after changing any constant in "
             f"`config.h`; every number below is recomputed from the raw counts.\n"]
    found = False
    if (data / "thermistor.csv").exists():
        parts.append(thermistor(data / "thermistor.csv", plots))
        found = True
    fit = None
    if (data / "fsr.csv").exists():
        md, fit = fsr(data / "fsr.csv", plots)
        parts.append(md)
        found = True
    if (data / "fsr_filter.csv").exists():
        parts.append(fsr_filter(data / "fsr_filter.csv", plots, args.alphas))
        found = True
    if not found:
        sys.exit(f"No calibration CSVs in {show(data)}. Record some with tools/log_serial.py "
                 "(or rehearse: --data calibration/sim_data).")

    # Image links in results.md are relative to the results file
    md = "\n".join(parts)
    rel = Path(plots.resolve()).relative_to(results.parent.resolve()) if \
        plots.resolve().is_relative_to(results.parent.resolve()) else plots.resolve()
    md = md.replace("](plots/", f"]({rel.as_posix()}/")
    results.write_text(md)
    print(f"  wrote {show(results)}")

    if args.apply_fsr_table:
        if fit is None:
            sys.exit("--apply-fsr-table needs fsr.csv")
        if not real:
            sys.exit("Refusing to write a table fitted to non-real data into the firmware. "
                     "Use calibration/data.")
        apply_fsr_table(fit, data / "fsr.csv")


if __name__ == "__main__":
    main()
