#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "sensors_analog.h"

// ---------------------------------------------------------------------------
// Flex calibration table: sensor resistance (ohms) -> bend angle (degrees).
// PLACEHOLDER from the Adafruit 182 spec (~10k flat, ~20k bent). Replace with
// the fit from calibration/data/flex.csv (tools/plot_calibration.py
// --apply-flex-table). Breakpoints must be ascending.
// Beyond either end the nearest segment's slope is extrapolated, then clamped.
// ---------------------------------------------------------------------------
static const float FLEX_R_OHMS[] = {10000.0f, 20000.0f};
static const float FLEX_DEG[]    = {0.0f, 90.0f};
static const int   FLEX_PTS      = sizeof(FLEX_R_OHMS) / sizeof(FLEX_R_OHMS[0]);

namespace analog_math {

float flexOhmsFromAdc(float adc) {
  // 5V -> flex -> A0 -> R_fixed -> GND
  if (adc <= 0.0f || adc >= ADC_MAX) return -1.0f;
  return FLEX_FIXED_R_OHMS * (ADC_MAX - adc) / adc;
}

float flexDegFromOhms(float ohms) {
  // Piecewise-linear interpolation through the table
  int i = 1;
  while (i < FLEX_PTS - 1 && ohms > FLEX_R_OHMS[i]) i++;
  float r0 = FLEX_R_OHMS[i - 1], r1 = FLEX_R_OHMS[i];
  float d0 = FLEX_DEG[i - 1],    d1 = FLEX_DEG[i];
  float deg = d0 + (d1 - d0) * (ohms - r0) / (r1 - r0);
  if (!(deg >= FLEX_DEG_MIN)) deg = FLEX_DEG_MIN;   // also catches NaN
  if (deg > FLEX_DEG_MAX) deg = FLEX_DEG_MAX;
  return deg;
}

float flexDegFromAdc(float adc) {
  float r = flexOhmsFromAdc(adc);
  if (r < 0.0f) return FLEX_DEG_MIN;
  return flexDegFromOhms(r);
}

bool flexAdcIsFault(float adc) {
  return adc <= FLEX_ADC_OPEN || adc >= FLEX_ADC_SHORT;
}

float thermOhmsFromAdc(float adc) {
  // 5V -> R_fixed -> A1 -> thermistor -> GND
  if (adc <= 0.0f || adc >= ADC_MAX) return -1.0f;
  return THERM_FIXED_R_OHMS * adc / (ADC_MAX - adc);
}

float thermCFromOhms(float ohms) {
  if (ohms <= 0.0f) return NAN;
  float invT = 1.0f / THERM_T0_K + log(ohms / THERM_R0_OHMS) / THERM_BETA;
  return 1.0f / invT - 273.15f;
}

bool thermAdcIsFault(float adc) {
  return adc <= THERM_ADC_SHORT || adc >= THERM_ADC_OPEN;
}

float ema(float prev, float sample, float alpha) {
  return prev + alpha * (sample - prev);
}

}  // namespace analog_math

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------
static uint32_t s_lastMs        = 0;   // uint32_t: same width as millis() on the Uno
static bool  s_flexSeeded       = false;
static int   s_flexRaw          = 0;
static float s_flexDeg          = 0.0f;
static float s_flexDegFilt      = 0.0f;
static bool  s_flexFault        = true;    // until a good reading arrives
static int   s_thRaw            = 0;
static float s_thC              = 25.0f;   // replaced by first good reading
static bool  s_thFault          = true;    // until a good reading arrives

// The ADC's sample capacitor needs a moment after switching channels, so the
// first read after a switch is discarded.
static int readSettled(uint8_t pin) {
  analogRead(pin);
  return analogRead(pin);
}

static void sampleAll() {
  // Flex: one read, so "raw" really is raw for the filter plot
  s_flexRaw = readSettled(PIN_FLEX);
  s_flexFault = analog_math::flexAdcIsFault((float)s_flexRaw);
  if (!s_flexFault) {
    s_flexDeg = analog_math::flexDegFromAdc((float)s_flexRaw);
    s_flexDegFilt = s_flexSeeded ? analog_math::ema(s_flexDegFilt, s_flexDeg, FLEX_EMA_ALPHA)
                                 : s_flexDeg;
    s_flexSeeded = true;
  }

  // Thermistor: average a few reads (temperature changes slowly)
  long sum = readSettled(PIN_THERM);
  for (int k = 1; k < THERM_OVERSAMPLE; k++) sum += analogRead(PIN_THERM);
  float adc = (float)sum / THERM_OVERSAMPLE;
  s_thRaw = (int)(adc + 0.5f);

  s_thFault = analog_math::thermAdcIsFault(adc);
  if (!s_thFault) {
    s_thC = analog_math::thermCFromOhms(analog_math::thermOhmsFromAdc(adc));
  }
}

void analogSensorsInit() {
  pinMode(PIN_FLEX, INPUT);
  pinMode(PIN_THERM, INPUT);
  sampleAll();                 // valid values before the first telemetry line
  s_lastMs = millis();
}

void analogSensorsUpdate() {
  uint32_t now = millis();
  if (now - s_lastMs < ANALOG_PERIOD_MS) return;   // rollover-safe
  s_lastMs = now;
  sampleAll();
}

int   flexRaw()            { return s_flexRaw; }
float flexDeg()            { return s_flexDeg; }
float flexDegFiltered()    { return s_flexDegFilt; }
bool  flexFault()          { return s_flexFault; }
int   thermistorRaw()      { return s_thRaw; }
float thermistorC()        { return s_thC; }
bool  thermistorFault()    { return s_thFault; }
