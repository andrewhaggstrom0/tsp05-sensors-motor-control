// Host-side tests for firmware/tsp05/sensors_digital.cpp (Track 3)
// Run: firmware/tests/run_tests.sh
// The TOF and the slot interrupt are faked (shim/SparkFun_VL53L1X.h,
// shim/Arduino.h), so the filter, fault and RPM logic run on a laptop.
// Expectations are derived from config.h, so retuning a constant keeps them valid.
#include <cstdio>
#include <cmath>
#include "Arduino.h"
#include "Wire.h"
#include "SparkFun_VL53L1X.h"
#include "config.h"
#include "sensors_digital.h"

// Fakes the shim declares
int        fake_adc[24] = {0};
uint32_t   fake_millis = 0;
long       fake_analog_reads = 0;
uint32_t   fake_micros = 0;
void     (*fake_isr)() = nullptr;
FakeSerial Serial;
TwoWire    Wire;
int        fake_tof_begin = 0;
bool       fake_tof_ready = false;
uint16_t   fake_tof_mm = 0;
uint8_t    fake_tof_status = 0;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
  printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

using namespace digital_math;

// Advance time 1 ms at a time, calling the module's update like loop() would
static void runMs(uint32_t ms) {
  for (uint32_t i = 0; i < ms; i++) {
    fake_millis += 1;
    fake_micros += 1000;
    digitalSensorsUpdate();
  }
}

// The TOF finishes one range: it becomes "ready" and is read on the next poll
static void tofRange(uint16_t mm, uint8_t status = 0) {
  fake_tof_mm = mm;
  fake_tof_status = status;
  fake_tof_ready = true;
  runMs(TOF_PERIOD_MS);
}

static void slotEdge() { fake_isr(); }

// RPM the period method should report for evenly spaced tabs
static float rpmForTabGapMs(float gapMs) { return 60000.0f / (SLOT_TABS * gapMs); }

// ---------------------------------------------------------------------------
static void test_math() {
  int odd[5] = {5, 1, 9, 3, 7};
  CHECK(median(odd, 5) == 5.0f, "median of 5 = %.1f, want 5", median(odd, 5));
  int even[2] = {4, 10};
  CHECK(median(even, 2) == 7.0f, "median of 2 = %.1f, want 7", median(even, 2));
  int one[1] = {42};
  CHECK(median(one, 1) == 42.0f, "median of 1");
  CHECK(median(odd, 0) == 0.0f, "median of 0 must be 0");
  int dup[5] = {300, 300, 900, 300, 300};
  CHECK(median(dup, 5) == 300.0f, "single outlier must not move the median");

  CHECK(std::fabs(ema(100.0f, 200.0f, 0.3f) - 130.0f) < 1e-4f, "ema step");
  CHECK(ema(100.0f, 200.0f, 1.0f) == 200.0f, "alpha 1 = no filtering");

  // 1 rev per second = 60 RPM, whatever the tab count
  CHECK(std::fabs(rpmFromSpan(1000000UL, SLOT_TABS, SLOT_TABS) - 60.0f) < 1e-3f, "60 rpm");
  CHECK(std::fabs(rpmFromSpan(250000UL, 1, 4) - 60.0f) < 1e-3f, "one 250 ms gap, 4 tabs = 60 rpm");
  CHECK(rpmFromSpan(0, 4, 4) == 0.0f, "zero span must give 0, not inf");
  CHECK(rpmFromSpan(1000, 0, 4) == 0.0f, "zero edges must give 0");
}

static void test_tof() {
  CHECK(tofFault(), "fault before the first valid range");

  // Steady target with one outlier: the median removes it from the filtered value
  const uint16_t seq[] = {300, 302, 298, 900, 301, 299, 300};
  for (uint16_t mm : seq) tofRange(mm);
  CHECK(tofRawMm() == 300, "raw = %d, want the latest valid range 300", tofRawMm());
  CHECK(std::fabs(tofFiltMm() - 300.0f) < 3.0f, "filtered = %.1f, outlier 900 should be rejected",
        tofFiltMm());
  CHECK(!tofFault(), "no fault while valid ranges arrive");

  // Invalid ranges (signal/sigma fail, wrapped) are skipped: values hold
  const uint8_t badStatus[] = {1, 2, 7};
  for (uint8_t st : badStatus) {
    tofRange(5, st);
    CHECK(tofRawMm() == 300, "status %d: raw changed to %d", st, tofRawMm());
  }

  // Step from 300 to 500 mm: filter follows without overshoot and settles
  float prev = tofFiltMm();
  bool monotonic = true;
  for (int i = 0; i < 30; i++) {
    tofRange(500);
    if (tofFiltMm() < prev - 1e-3f || tofFiltMm() > 500.0f + 1e-3f) monotonic = false;
    prev = tofFiltMm();
  }
  CHECK(monotonic, "step response must rise monotonically without overshoot");
  CHECK(std::fabs(tofFiltMm() - 500.0f) < 1.0f, "settled at %.1f, want 500", tofFiltMm());

  // Sensor stops producing valid ranges -> fault after TOF_STALE_MS
  runMs(TOF_STALE_MS + 10);
  CHECK(tofFault(), "fault after %d ms without a valid range", TOF_STALE_MS);
  tofRange(400);
  CHECK(!tofFault(), "fault clears when ranges return");
}

static void test_slot() {
  // Let any start-up state time out
  runMs(SLOT_TIMEOUT_MS + SLOT_RPM_PERIOD_MS);
  CHECK(slotRpm() == 0.0f, "idle rpm = %.2f, want 0", slotRpm());
  long base = slotCount();
  slotTakeEvents();

  // Even tabs, 250 ms apart
  const float gap = 250.0f;
  for (int i = 0; i < 3 * SLOT_TABS; i++) { slotEdge(); runMs((uint32_t)gap); }
  CHECK(std::fabs(slotRpm() - rpmForTabGapMs(gap)) < 0.5f, "even tabs: %.2f rpm, want %.2f",
        slotRpm(), rpmForTabGapMs(gap));
  CHECK(slotCount() - base == 3 * SLOT_TABS, "count +%ld, want %d", slotCount() - base, 3 * SLOT_TABS);

  // slotTakeEvents: returns new events once, then 0
  CHECK(slotTakeEvents() == 3 * SLOT_TABS, "take events");
  CHECK(slotTakeEvents() == 0, "events must clear after being taken");

  // Glitch filter: a second edge inside SLOT_GLITCH_US is bounce, not a tab
  long before = slotCount();
  slotEdge();
  fake_micros += SLOT_GLITCH_US / 2;
  slotEdge();
  CHECK(slotCount() - before == 1, "bounce counted: +%ld, want +1", slotCount() - before);

  // Stopped disc -> 0 RPM after SLOT_TIMEOUT_MS
  runMs(SLOT_TIMEOUT_MS + SLOT_RPM_PERIOD_MS);
  CHECK(slotRpm() == 0.0f, "after timeout: %.2f rpm, want 0", slotRpm());
}

static void test_slot_uneven_tabs() {
  // Hand-cut tabs: alternate short/long gaps, 1 rev still takes 1 s (60 rpm).
  // Sampled every 10 ms between edges, the RPM must stay at 60 the whole time.
  runMs(SLOT_TIMEOUT_MS + SLOT_RPM_PERIOD_MS);
  static_assert(SLOT_TABS % 2 == 0, "this test alternates gaps, needs an even tab count");
  const uint32_t shortGap = 2 * 1000 / SLOT_TABS * 2 / 5;     // 40 % of an even pair
  const uint32_t longGap  = 2 * 1000 / SLOT_TABS - shortGap;  // pair sums to 2 even gaps
  float lo = 1e9f, hi = 0.0f;
  for (int rev = 0; rev < 5; rev++) {
    for (int t = 0; t < SLOT_TABS; t++) {
      slotEdge();
      uint32_t g = (t % 2) ? longGap : shortGap;
      for (uint32_t ms = 0; ms < g; ms += 10) {
        runMs(10);
        if (rev >= 2) { lo = std::fmin(lo, slotRpm()); hi = std::fmax(hi, slotRpm()); }
      }
    }
  }
  CHECK(lo > 59.5f && hi < 60.5f, "uneven tabs at 60 rpm read %.2f..%.2f", lo, hi);

  // Spin-down: no more edges. RPM must fall smoothly, never rise, and reach 0.
  float prev = slotRpm();
  bool neverRises = true, fellBeforeTimeout = false;
  for (uint32_t ms = 0; ms < SLOT_TIMEOUT_MS + SLOT_RPM_PERIOD_MS; ms += 10) {
    runMs(10);
    if (slotRpm() > prev + 1e-3f) neverRises = false;
    if (ms < SLOT_TIMEOUT_MS && slotRpm() < 30.0f && slotRpm() > 0.0f) fellBeforeTimeout = true;
    prev = slotRpm();
  }
  CHECK(neverRises, "rpm rose while the disc was slowing");
  CHECK(fellBeforeTimeout, "rpm should decay before the timeout, not hold then drop");
  CHECK(slotRpm() == 0.0f, "rpm = %.2f after spin-down, want 0", slotRpm());
}

static void test_missing_tof() {
  // Re-init with no sensor on the bus: slot sensor still works, TOF reports a fault
  fake_tof_begin = 1;
  digitalSensorsInit();
  tofRange(123);
  CHECK(tofFault(), "missing TOF must report a fault");
  CHECK(tofRawMm() != 123, "missing TOF must not read ranges");
  long before = slotCount();
  slotEdge();
  CHECK(slotCount() == before + 1, "slot sensor must work without the TOF");
}

int main() {
  fake_millis = 10;
  fake_micros = 10000;
  digitalSensorsInit();
  CHECK(fake_isr != nullptr, "init must attach the slot interrupt");

  test_math();
  test_tof();
  test_slot();
  test_slot_uneven_tabs();
  test_missing_tof();

  printf("%d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
