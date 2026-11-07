#pragma once
// Track 4: flex/bend sensor (A0) and thermistor (A1).
// Call analogSensorsInit() once from setup() and analogSensorsUpdate()
// every loop(). Update self-limits to ANALOG_PERIOD_MS and never blocks.

void  analogSensorsInit();
void  analogSensorsUpdate();

// Getters (latest values, cheap to call)
int   flexRaw();              // ADC counts, 0..1023 (falls as the sensor bends)
float flexDeg();              // unfiltered bend angle, degrees (0 = flat)
float flexDegFiltered();      // EMA-filtered bend angle, degrees
bool  flexFault();            // true if unplugged/shorted; angle holds the last good value
int   thermistorRaw();        // ADC counts (oversampled average, rounded)
float thermistorC();          // degC; holds the last good value during a fault
bool  thermistorFault();      // true if unplugged/shorted. Treat as interlock-active.

// Pure conversions (no hardware access). Exposed so they can be unit tested
// on a laptop and checked against tools/analog_model.py.
namespace analog_math {
  float flexOhmsFromAdc(float adc);      // -1 if adc out of range
  float flexDegFromOhms(float ohms);     // FLEX_DEG_MIN..FLEX_DEG_MAX, never NaN
  float flexDegFromAdc(float adc);
  bool  flexAdcIsFault(float adc);
  float thermOhmsFromAdc(float adc);     // -1 if adc out of range
  float thermCFromOhms(float ohms);      // NaN if ohms <= 0
  bool  thermAdcIsFault(float adc);
  float ema(float prev, float sample, float alpha);
}
