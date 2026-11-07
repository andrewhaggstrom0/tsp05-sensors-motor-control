// Host-side tests for firmware/tsp05/sensors_analog.cpp
// Run: firmware/tests/run_tests.sh
#include <cstdio>
#include <cmath>
#include "Arduino.h"
#include "config.h"
#include "sensors_analog.h"

int fake_adc[24] = {0};
uint32_t fake_millis = 0;
long fake_analog_reads = 0;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
  printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace analog_math;

// Independent reference (double precision) to check the firmware math
static double refThermC(double adc) {
  double r = 10000.0 * adc / (1023.0 - adc);
  return 1.0 / (1.0 / 298.15 + std::log(r / 10000.0) / 3950.0) - 273.15;
}

static void test_thermistor_math() {
  // Divider midpoint means R_t = R_fixed = R0, so exactly 25 degC
  float c = thermCFromOhms(thermOhmsFromAdc(511.5f));
  CHECK(std::fabs(c - 25.0f) < 0.01f, "midpoint gave %.3f C, want 25", c);

  // Matches double-precision reference across the usable range
  for (int adc = 50; adc <= 970; adc += 20) {
    float got = thermCFromOhms(thermOhmsFromAdc((float)adc));
    double want = refThermC(adc);
    CHECK(std::fabs(got - want) < 0.05, "adc %d: got %.3f want %.3f", adc, got, want);
  }

  // Warmer -> lower thermistor R -> lower A1 counts: temperature must fall as adc rises
  float prev = 1e9f;
  for (int adc = THERM_ADC_SHORT + 1; adc < THERM_ADC_OPEN; adc++) {
    float t = thermCFromOhms(thermOhmsFromAdc((float)adc));
    CHECK(std::isfinite(t), "adc %d gave non-finite temp", adc);
    CHECK(t < prev, "not monotonic at adc %d", adc);
    prev = t;
  }

  // Expected rough operating points (from the team guide discussion)
  float t0 = thermCFromOhms(thermOhmsFromAdc(783.0f));
  float t37 = thermCFromOhms(thermOhmsFromAdc(383.0f));
  CHECK(std::fabs(t0 - 0.0f) < 1.5f, "783 counts gave %.2f C, want ~0", t0);
  CHECK(std::fabs(t37 - 37.0f) < 1.5f, "383 counts gave %.2f C, want ~37", t37);

  // Edge cases
  CHECK(thermOhmsFromAdc(0) < 0 && thermOhmsFromAdc(1023) < 0, "edge adc should be -1");
  CHECK(std::isnan(thermCFromOhms(-1.0f)), "negative ohms should give NaN");
  CHECK(thermAdcIsFault(1023) && thermAdcIsFault(0), "open/short must be faults");
  CHECK(!thermAdcIsFault(511), "midpoint must not be a fault");
}

static void test_fsr_math() {
  CHECK(fsrNewtonsFromAdc(0) == 0.0f, "no press should be 0 N");
  CHECK(fsrNewtonsFromAdc(FSR_ADC_DEADZONE) == 0.0f, "deadzone should be 0 N");
  CHECK(fsrOhmsFromAdc(0) < 0, "adc 0 should report open (-1)");

  float prev = -1.0f;
  for (int adc = 0; adc <= ADC_MAX; adc++) {
    float f = fsrNewtonsFromAdc((float)adc);
    CHECK(std::isfinite(f), "adc %d gave non-finite force", adc);
    CHECK(f >= 0.0f && f <= FSR_MAX_N, "adc %d gave %.2f N out of range", adc, f);
    CHECK(f >= prev, "force not monotonic at adc %d", adc);
    prev = f;
  }
  CHECK(fsrNewtonsFromAdc(1023) == FSR_MAX_N, "full scale should clamp to max");

  // Table must be continuous at breakpoints (no jump at the knee)
  // G = 1000 uS  <=>  R = 1000 ohm  <=>  adc = 1023*10k/(10k+1k)
  float knee = 1023.0f * 10000.0f / 11000.0f;
  float below = fsrNewtonsFromAdc(knee - 0.5f), above = fsrNewtonsFromAdc(knee + 0.5f);
  CHECK(std::fabs(above - below) < 0.5f, "jump at knee: %.2f -> %.2f", below, above);
}

static void test_ema() {
  float y = 0.0f;
  for (int i = 0; i < 100; i++) y = ema(y, 10.0f, 0.2f);
  CHECK(std::fabs(y - 10.0f) < 1e-3f, "EMA should converge to step, got %.4f", y);
  CHECK(std::fabs(ema(0.0f, 10.0f, 0.2f) - 2.0f) < 1e-6f, "one EMA step should be 2.0");
}

static void test_module_update() {
  fake_millis = 1000;
  fake_adc[A0] = 0;      // no press
  fake_adc[A1] = 512;    // ~25 C
  analogSensorsInit();
  CHECK(!thermistorFault(), "good reading should clear fault");
  CHECK(std::fabs(thermistorC() - 25.0f) < 0.2f, "init temp %.2f", thermistorC());
  CHECK(fsrNewtons() == 0.0f && fsrNewtonsFiltered() == 0.0f, "init force should be 0");

  // Rate limiting: nothing happens before the period elapses
  fake_adc[A0] = 900;
  long reads = fake_analog_reads;
  fake_millis += ANALOG_PERIOD_MS - 1;
  analogSensorsUpdate();
  CHECK(fake_analog_reads == reads, "update ran early (no ADC reads expected)");
  CHECK(fsrRaw() == 0, "raw changed before period elapsed");

  // After the period it samples, and the filter lags the raw value
  fake_millis += 1;
  analogSensorsUpdate();
  CHECK(fsrRaw() == 900, "raw should update, got %d", fsrRaw());
  CHECK(fsrNewtons() > 0.0f, "force should be > 0");
  CHECK(fsrNewtonsFiltered() > 0.0f && fsrNewtonsFiltered() < fsrNewtons(),
        "filtered (%.2f) should lag raw (%.2f)", fsrNewtonsFiltered(), fsrNewtons());

  // Unplugged thermistor: fault set, temperature holds last good value
  float lastGood = thermistorC();
  fake_adc[A1] = 1023;
  fake_millis += ANALOG_PERIOD_MS;
  analogSensorsUpdate();
  CHECK(thermistorFault(), "unplugged thermistor should fault");
  CHECK(thermistorC() == lastGood, "temp should hold during fault");
  CHECK(std::isfinite(thermistorC()), "temp must never be NaN");

  // Recovers when plugged back in
  fake_adc[A1] = 400;
  fake_millis += ANALOG_PERIOD_MS;
  analogSensorsUpdate();
  CHECK(!thermistorFault() && thermistorC() > 30.0f, "should recover, got %.2f", thermistorC());

  // millis() rollover does not stall updates
  fake_millis = 0xFFFFFFFFu - 5;
  analogSensorsUpdate();
  fake_adc[A0] = 0;
  fake_millis = 0xFFFFFFFFu - 5 + ANALOG_PERIOD_MS;  // wraps past zero
  analogSensorsUpdate();
  CHECK(fsrRaw() == 0, "update should run across millis() rollover");
}

int main() {
  test_thermistor_math();
  test_fsr_math();
  test_ema();
  test_module_update();
  printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
