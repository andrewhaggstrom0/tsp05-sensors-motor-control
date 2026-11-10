#pragma once
// Track 3: VL53L1X time-of-flight (I2C on A4/A5) and EE-SX1042 slot sensor (D3, INT1).
// Call digitalSensorsInit() once from setup() and digitalSensorsUpdate() every
// loop(). Update only reads the TOF when a new range is ready; never blocks.

#include <stdint.h>

void  digitalSensorsInit();
void  digitalSensorsUpdate();

// TOF
int   tofRawMm();          // latest valid range, mm (holds the last valid value)
float tofFiltMm();         // filtered range, mm: median of TOF_MEDIAN_N, then EMA
bool  tofFault();          // true if the sensor is missing or no valid range for TOF_STALE_MS

// Slot sensor
long  slotCount();         // total beam-break events since boot
float slotRpm();           // events / slots-per-rev over a window; 0 when idle
int   slotTakeEvents();    // new events since the last call (used to jog the stepper)

// Pure helpers (no hardware access), exposed so they can be unit tested on a laptop.
namespace digital_math {
  float median(const int *v, uint8_t n);                 // n <= 9; 0 if n == 0
  float ema(float prev, float sample, float alpha);
  // RPM from the time spanned by `edges` consecutive tab-to-tab intervals.
  float rpmFromSpan(uint32_t spanUs, uint8_t edges, uint8_t tabs);  // 0 if span or edges is 0
}
