# Calibration

Measured data lives in `data/`, generated figures in `plots/`. CSVs hold only
measured or reference values; anything that depends on constants in
`firmware/tsp05/config.h` (model temperature, model angle, errors) is computed
when the plots are made, so changing a constant never leaves stale numbers here.

## File schemas

### `data/thermistor.csv` (one row per reference temperature)
| Column | Units | Meaning |
|---|---|---|
| ref_C | degC | Thermometer reading (the reference) |
| adc_mean | ADC counts | Mean of `n` samples of A1 (each sample = mean of 4 reads) |
| adc_std | ADC counts | Standard deviation of those samples |
| n | samples | Samples averaged (100 = 2 s at 50 Hz) |

### `data/flex.csv` (one row per angle)
| Column | Units | Meaning |
|---|---|---|
| trial | # | Increments each time bending starts again after unbending |
| direction | bend/unbend | Whether the angle was increasing or decreasing (shows hysteresis) |
| ref_deg | degrees | Bend angle measured with a protractor (the reference) |
| adc_mean | ADC counts | Mean of `n` samples of A0, taken after a 1 s settle |
| adc_std | ADC counts | Standard deviation of those samples |
| n | samples | Samples averaged (50 = 1 s) |

### `data/flex_filter.csv` (continuous recording)
| Column | Units | Meaning |
|---|---|---|
| t_s | s | Time since the first sample |
| flex_adc | ADC counts | Raw A0 at 50 Hz (filtered angle is computed from this) |

## Procedure

Flash `firmware/bringup/analog/analog.ino` and close the Arduino Serial Monitor
(only one program can hold the port). Then:

```bash
pip install -r tools/requirements.txt
```

### Before you start
- [ ] Measure both 10k fixed resistors with a multimeter. Put the real values in
      `FLEX_FIXED_R_OHMS` and `THERM_FIXED_R_OHMS` in `config.h`.
- [ ] Check B (and R0) on the thermistor datasheet; update `THERM_BETA`.
- [ ] Print or draw a protractor guide (0 to 90 degrees in 15 degree steps).

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

### Flex sensor (A4)
```bash
python3 tools/log_serial.py flex
```
Tape the crimped end (where the pins are) flat to the table; that end is
fragile. Bend the free end in the direction that gives the big change in A0,
hold it at each angle against a protractor, type the angle, and keep holding
until it prints `saved`. Go 0, 15, 30, 45, 60, 75, 90 then back down. The
script tags bend/unbend and numbers the trials for you.

### Flex filter recording (A5)
```bash
python3 tools/log_serial.py stream --seconds 15
```
Bend to three different angles, holding each about 2 s and releasing between.

### Rehearsal without hardware
Add `--sim` to any command. Simulated data goes to `calibration/sim_data/`
(git-ignored), never to `data/`.

## Figures and error tables (A2 to A6)

```bash
python3 tools/plot_calibration.py
```
Reads every CSV in `data/` and writes `plots/*.png` plus `results.md` (all error
tables, ready for the main README). Rerun it whenever a CSV or a constant in
`config.h` changes. After the flex calibration, write the fitted table into the
firmware and re-verify:
```bash
python3 tools/plot_calibration.py --apply-flex-table
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
