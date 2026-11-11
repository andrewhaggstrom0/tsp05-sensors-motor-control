#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_VL53L1X.h>
#include "config.h"
#include "sensors_digital.h"
#include "tof_gesture.h"

// ---------------------------------------------------------------------------
// TOF: ranging runs continuously inside the sensor. Every TOF_POLL_MS we ask
// whether a new range is ready (one short I2C read) and only then fetch it,
// so the loop never waits. Only status-0 (valid) ranges reach the filter.
//
// Slot sensor: beam blocked -> D3 goes HIGH -> RISING-edge interrupt. The ISR
// timestamps each edge into a ring buffer. RPM uses the time between edges
// (period method), averaged over one full revolution when available, so
// unevenly cut tabs cancel out. Counting events in a short window instead
// would quantise badly: 4 tabs over 100 ms = 150 RPM per count.
// ---------------------------------------------------------------------------

#if TOF_MEDIAN_N > 9 || TOF_MEDIAN_N < 1
#error "TOF_MEDIAN_N must be 1..9"
#endif

namespace digital_math {

float median(const int *v, uint8_t n) {
  if (n == 0) return 0.0f;
  int s[9];
  for (uint8_t i = 0; i < n; i++) {          // insertion sort of a copy
    int x = v[i];
    int8_t j = (int8_t)i - 1;
    while (j >= 0 && s[j] > x) { s[j + 1] = s[j]; j--; }
    s[j + 1] = x;
  }
  if (n & 1) return (float)s[n / 2];
  return 0.5f * (float)(s[n / 2 - 1] + s[n / 2]);
}

float ema(float prev, float sample, float alpha) {
  return prev + alpha * (sample - prev);
}

float rpmFromSpan(uint32_t spanUs, uint8_t edges, uint8_t tabs) {
  if (spanUs == 0 || edges == 0 || tabs == 0) return 0.0f;
  // edges intervals took spanUs; one revolution = tabs intervals
  return 60.0e6f * (float)edges / ((float)tabs * (float)spanUs);
}

}  // namespace digital_math

// ---------------------------------------------------------------------------
// TOF state
// ---------------------------------------------------------------------------
static SFEVL53L1X s_tof;
static bool      s_tofPresent   = false;
static uint32_t  s_lastPollMs   = 0;
static uint32_t  s_lastValidMs  = 0;
static int       s_rawMm        = 0;
static float     s_filtMm       = 0.0f;
static bool      s_filtSeeded   = false;
static int       s_hist[TOF_MEDIAN_N];
static uint8_t   s_histN        = 0;      // samples in the window (< N only at start)
static uint8_t   s_histI        = 0;      // next slot to overwrite

static void tofAddSample(int mm) {
  s_rawMm = mm;
  s_hist[s_histI] = mm;
  s_histI = (uint8_t)((s_histI + 1) % TOF_MEDIAN_N);
  if (s_histN < TOF_MEDIAN_N) s_histN++;
  float med = digital_math::median(s_hist, s_histN);
  s_filtMm = s_filtSeeded ? digital_math::ema(s_filtMm, med, TOF_EMA_ALPHA) : med;
  s_filtSeeded = true;
}

// Ranges alternate between the left and right halves of the view (tof_gesture), so
// every reading also feeds the swipe detector. The filter still gets every valid
// reading: for one target both halves measure about the same distance.
static const uint8_t      ZONE_CENTER[2] = {GESTURE_CENTER_A, GESTURE_CENTER_B};
static tof_gesture::Zone  s_zone  = tof_gesture::LEFT;
static tof_gesture::Detector s_swipeDet;
static int8_t             s_swipe = 0;

static void tofUpdate(uint32_t now) {
  if (!s_tofPresent) return;
  if ((uint32_t)(now - s_lastPollMs) < TOF_POLL_MS) return;
  s_lastPollMs = now;

  if (!s_tof.checkForDataReady()) return;   // nothing new: don't wait
  uint16_t mm     = s_tof.getDistance();
  uint8_t  status = s_tof.getRangeStatus();
  tof_gesture::Zone measured = s_zone;
  s_zone = (s_zone == tof_gesture::LEFT) ? tof_gesture::RIGHT : tof_gesture::LEFT;
  s_tof.setROI(GESTURE_ROI_W, GESTURE_ROI_H, ZONE_CENTER[s_zone]);
  s_tof.clearInterrupt();                   // arm the next range
  tof_gesture::Swipe sw = tof_gesture::update(s_swipeDet, measured, mm, status == 0, now);
  if (sw != tof_gesture::NONE) s_swipe = (int8_t)sw;
  if (status != 0) return;                  // 1 signal fail, 2 sigma fail, 7 wrapped: skip
  s_lastValidMs = now;
  tofAddSample((int)mm);
}

// ---------------------------------------------------------------------------
// Slot state. Everything the ISR writes is volatile; multi-byte values are
// read with interrupts paused so they can't change halfway through a read.
// ---------------------------------------------------------------------------
#define SLOT_RING (SLOT_TABS + 1)            // N+1 timestamps span N intervals = 1 rev

static volatile uint32_t s_edgeUs[SLOT_RING];
static volatile uint8_t  s_head      = 0;    // index of the newest timestamp
static volatile uint8_t  s_run       = 0;    // timestamps since the disc last stopped (<= SLOT_RING)
static volatile uint32_t s_count     = 0;
static volatile uint16_t s_pending   = 0;
static uint32_t          s_lastRpmMs = 0;
static float             s_rpm       = 0.0f;

static void slotIsr() {
  uint32_t now = micros();
  if ((uint32_t)(now - s_edgeUs[s_head]) < SLOT_GLITCH_US) return;   // bounce
  s_head = (uint8_t)((s_head + 1) % SLOT_RING);
  s_edgeUs[s_head] = now;
  if (s_run < SLOT_RING) s_run++;
  s_count++;
  if (s_pending < 0xFFFF) s_pending++;
}

static void slotUpdate(uint32_t now) {
  if ((uint32_t)(now - s_lastRpmMs) < SLOT_RPM_PERIOD_MS) return;
  s_lastRpmMs = now;

  noInterrupts();
  uint32_t nowUs  = micros();
  uint8_t  run    = s_run;
  uint32_t newest = s_edgeUs[s_head];
  uint8_t  edges  = run > 1 ? (uint8_t)(run - 1) : 0;    // intervals available
  if (edges > SLOT_TABS) edges = SLOT_TABS;
  uint32_t oldest = s_edgeUs[(s_head + SLOT_RING - edges) % SLOT_RING];
  uint32_t second = s_edgeUs[(s_head + SLOT_RING - edges + 1) % SLOT_RING];
  uint32_t since  = nowUs - newest;
  if (since > (uint32_t)SLOT_TIMEOUT_MS * 1000UL) s_run = 0;  // stopped: next spin starts fresh
  interrupts();

  if (run == 0 || since > (uint32_t)SLOT_TIMEOUT_MS * 1000UL || edges == 0) {
    s_rpm = 0.0f;
    return;
  }
  float rpm = digital_math::rpmFromSpan(newest - oldest, edges, SLOT_TABS);
  // Slowing down: if the gap in progress is already longer than the same gap
  // one revolution ago, swap it in (oldest interval out, in-progress one in).
  // Comparing like-for-like gaps keeps uneven tabs from faking a slowdown,
  // and the RPM decays smoothly instead of holding until the 2 s timeout.
  float bound = digital_math::rpmFromSpan((newest - second) + since, edges, SLOT_TABS);
  s_rpm = (bound > 0.0f && bound < rpm) ? bound : rpm;
}

// ---------------------------------------------------------------------------
void digitalSensorsInit() {
  pinMode(PIN_SLOT, INPUT);                  // external 10k pull-up
  attachInterrupt(digitalPinToInterrupt(PIN_SLOT), slotIsr, RISING);

  Wire.begin();
  Wire.setClock(TOF_I2C_HZ);
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);           // a glitched bus can't freeze the loop
#endif

  s_tofPresent = false;                      // set again only if the sensor answers
  if (s_tof.begin() != 0) {                  // SparkFun lib: 0 = success
    Serial.println(F("# TOF not found: check 3V3, SDA=A4, SCL=A5, GND"));
    return;
  }
  s_tof.setDistanceModeShort();              // ~1.3 m, better in ambient light
  s_tof.setTimingBudgetInMs(TOF_TIMING_BUDGET_MS);
  s_tof.setIntermeasurementPeriod(TOF_PERIOD_MS);
  s_zone = tof_gesture::LEFT;
  s_tof.setROI(GESTURE_ROI_W, GESTURE_ROI_H, ZONE_CENTER[s_zone]);
  tof_gesture::reset(s_swipeDet);
  s_swipe = 0;
  s_tof.startRanging();
  s_tofPresent = true;
  s_lastPollMs = millis();
}

void digitalSensorsUpdate() {
  uint32_t now = millis();
  tofUpdate(now);
  slotUpdate(now);
}

int   tofRawMm()  { return s_rawMm; }
float tofFiltMm() { return s_filtMm; }
bool  tofFault() {
  return !s_tofPresent || !s_filtSeeded ||
         (uint32_t)(millis() - s_lastValidMs) > TOF_STALE_MS;
}
int   tofTakeSwipe() { int s = s_swipe; s_swipe = 0; return s; }

long slotCount() {
  noInterrupts();
  uint32_t n = s_count;
  interrupts();
  return (long)n;
}

float slotRpm() { return s_rpm; }

int slotTakeEvents() {
  noInterrupts();
  uint16_t n = s_pending;
  s_pending = 0;
  interrupts();
  return n > 32767 ? 32767 : (int)n;
}
