# Calibration

Measured data lives in `data/`, generated figures in `plots/`. CSVs hold only
measured or reference values; anything that depends on constants in
`firmware/tsp05/config.h` (model temperature, model force, errors) is computed
when the plots are made, so changing a constant never leaves stale numbers here.

## File schemas

### `data/thermistor.csv` (one row per reference temperature)
| Column | Units | Meaning |
|---|---|---|
| ref_C | degC | Thermometer reading (the reference) |
| adc_mean | ADC counts | Mean of `n` samples of A1 (each sample = mean of 4 reads) |
| adc_std | ADC counts | Standard deviation of those samples |
| n | samples | Samples averaged (100 = 2 s at 50 Hz) |

### `data/fsr.csv` (one row per mass)
| Column | Units | Meaning |
|---|---|---|
| trial | # | Increments each time loading starts again after unloading |
| direction | load/unload | Whether mass was being added or removed (shows hysteresis) |
| mass_g | g | Total mass on the FSR, puck included (kitchen scale) |
| force_N | N | Reference force, mass_g / 1000 x 9.81 |
| adc_mean | ADC counts | Mean of `n` samples of A0, taken after a 2 s settle |
| adc_std | ADC counts | Standard deviation of those samples |
| n | samples | Samples averaged (50 = 1 s) |

### `data/fsr_filter.csv` (continuous recording)
| Column | Units | Meaning |
|---|---|---|
| t_s | s | Time since the first sample |
| fsr_adc | ADC counts | Raw A0 at 50 Hz (filtered force is computed from this) |

## Procedure

Flash `firmware/bringup/analog/analog.ino` and close the Arduino Serial Monitor
(only one program can hold the port). Then:

```bash
pip install -r tools/requirements.txt
```

### Before you start
- [ ] Measure both 10k fixed resistors with a multimeter. Put the real values in
      `FSR_FIXED_R_OHMS` and `THERM_FIXED_R_OHMS` in `config.h`.
- [ ] Check B (and R0) on the thermistor datasheet; update `THERM_BETA`.
- [ ] Weigh the puck and every mass on a kitchen scale; write the numbers down.

### Thermistor (A2)
```bash
python3 tools/log_serial.py therm
```
Tape the thermistor next to the thermometer bulb so both read the same water.
Keep the leg joints dry (cling film or heat shrink). For each point, type the
thermometer reading; the script waits for the reading to settle, then captures.
- Ice water: crushed ice **and** water, stirred (~0 degC)
- Room air
- Body: held in a closed hand (~33 to 35 degC; hand surface, not 37)
- Warm water at 2 to 3 temperatures (e.g. ~40, ~50 degC), stirred

### FSR (A4)
```bash
python3 tools/log_serial.py fsr
```
Centre a small rigid puck (a coin) on the FSR's active area so every mass loads
the same area. Type the **total** mass on the FSR each time. Go up in steps
(0, 50, 100, 200, 500, 1000 g...), then back down. Do at least 2 trials;
the script numbers them and tags load/unload for you.

### FSR filter recording (A5)
```bash
python3 tools/log_serial.py stream --seconds 15
```
Press and release the FSR a few times with different force.

### Rehearsal without hardware
Add `--sim` to any command. Simulated data goes to `calibration/sim_data/`
(git-ignored), never to `data/`.

## Figures and error tables (A2 to A6)

```bash
python3 tools/plot_calibration.py
```
Reads every CSV in `data/` and writes `plots/*.png` plus `results.md` (all error
tables, ready for the main README). Rerun it whenever a CSV or a constant in
`config.h` changes. After the FSR calibration, write the fitted table into the
firmware and re-verify:
```bash
python3 tools/plot_calibration.py --apply-fsr-table
firmware/tests/run_tests.sh && python3 tools/check_firmware_vs_model.py
```
Compare filter settings with `--alphas 0.1 0.4`. Rehearse with
`--data calibration/sim_data`.

Any time series CSV, including Foxglove exports (TOF raw vs filtered, DC step
response):
```bash
python3 tools/plot_calibration.py --timeseries export.csv \
    --series raw_mm filt_mm --title "TOF filter" --ylabel "Distance (mm)"
```
