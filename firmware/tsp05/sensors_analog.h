#pragma once
// Track 4: FSR (A0) and thermistor (A1).
// Call analogSensorsInit() once from setup() and analogSensorsUpdate()
// every loop(). Update self-limits to ANALOG_PERIOD_MS and never blocks.

void  analogSensorsInit();
void  analogSensorsUpdate();

// Getters (latest values, cheap to call)
int   fsrRaw();               // ADC counts, 0..1023
float fsrNewtons();           // unfiltered force, N
float fsrNewtonsFiltered();   // EMA-filtered force, N
int   thermistorRaw();        // ADC counts (oversampled average, rounded)
float thermistorC();          // degC; holds the last good value during a fault
bool  thermistorFault();      // true if unplugged/shorted. Treat as interlock-active.

// Pure conversions (no hardware access). Exposed so they can be unit tested
// on a laptop and checked against tools/analog_model.py.
namespace analog_math {
  float fsrOhmsFromAdc(float adc);       // -1 if no press (adc <= 0)
  float fsrNewtonsFromAdc(float adc);    // 0..FSR_MAX_N, never NaN
  float thermOhmsFromAdc(float adc);     // -1 if adc out of range
  float thermCFromOhms(float ohms);      // NaN if ohms <= 0
  bool  thermAdcIsFault(float adc);
  float ema(float prev, float sample, float alpha);
}
