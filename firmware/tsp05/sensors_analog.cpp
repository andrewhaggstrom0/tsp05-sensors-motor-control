#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "sensors_analog.h"

// ---------------------------------------------------------------------------
// FSR calibration table: conductance (microsiemens = 1e6 / R) -> force (N).
// PLACEHOLDER from the Adafruit FSR guide's approximation. Replace with the
// fit from step A4 (calibration/data/fsr.csv). Breakpoints must be ascending.
// Beyond the last point the last segment's slope is extrapolated, then clamped.
// ---------------------------------------------------------------------------
static const float FSR_G_US[] = {0.0f, 1000.0f, 4000.0f};
static const float FSR_N[]    = {0.0f,   12.5f,  112.5f};
static const int   FSR_PTS    = sizeof(FSR_G_US) / sizeof(FSR_G_US[0]);

namespace analog_math {

float fsrOhmsFromAdc(float adc) {
  // 5V -> FSR -> A0 -> R_fixed -> GND
  if (adc <= 0.0f) return -1.0f;                 // open circuit: no press
  if (adc >= ADC_MAX) return 0.0f;               // FSR fully shorted
  return FSR_FIXED_R_OHMS * (ADC_MAX - adc) / adc;
}

float fsrNewtonsFromAdc(float adc) {
  if (adc <= FSR_ADC_DEADZONE) return 0.0f;
  float r = fsrOhmsFromAdc(adc);
  if (r < 1.0f) r = 1.0f;                        // avoid divide by zero
  float g = 1.0e6f / r;                          // microsiemens

  // Piecewise-linear interpolation through the table
  int i = 1;
  while (i < FSR_PTS - 1 && g > FSR_G_US[i]) i++;
  float g0 = FSR_G_US[i - 1], g1 = FSR_G_US[i];
  float n0 = FSR_N[i - 1],    n1 = FSR_N[i];
  float f = n0 + (n1 - n0) * (g - g0) / (g1 - g0);

  if (f < 0.0f) f = 0.0f;
  if (f > FSR_MAX_N) f = FSR_MAX_N;
  return f;
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
static bool  s_seeded           = false;
static int   s_fsrRaw           = 0;
static float s_fsrN             = 0.0f;
static float s_fsrNFilt         = 0.0f;
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
  // FSR: one read, so "raw" really is raw for the filter plot
  s_fsrRaw = readSettled(PIN_FSR);
  s_fsrN   = analog_math::fsrNewtonsFromAdc((float)s_fsrRaw);
  s_fsrNFilt = s_seeded ? analog_math::ema(s_fsrNFilt, s_fsrN, FSR_EMA_ALPHA)
                        : s_fsrN;

  // Thermistor: average a few reads (temperature changes slowly)
  long sum = readSettled(PIN_THERM);
  for (int k = 1; k < THERM_OVERSAMPLE; k++) sum += analogRead(PIN_THERM);
  float adc = (float)sum / THERM_OVERSAMPLE;
  s_thRaw = (int)(adc + 0.5f);

  s_thFault = analog_math::thermAdcIsFault(adc);
  if (!s_thFault) {
    s_thC = analog_math::thermCFromOhms(analog_math::thermOhmsFromAdc(adc));
  }
  s_seeded = true;
}

void analogSensorsInit() {
  pinMode(PIN_FSR, INPUT);
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

int   fsrRaw()             { return s_fsrRaw; }
float fsrNewtons()         { return s_fsrN; }
float fsrNewtonsFiltered() { return s_fsrNFilt; }
int   thermistorRaw()      { return s_thRaw; }
float thermistorC()        { return s_thC; }
bool  thermistorFault()    { return s_thFault; }
