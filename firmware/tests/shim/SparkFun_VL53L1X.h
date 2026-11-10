#pragma once
// Fake VL53L1X: tests set these to script what the sensor returns.
#include <stdint.h>
extern int      fake_tof_begin;    // begin() result: 0 = sensor found
extern bool     fake_tof_ready;    // checkForDataReady(); cleared by clearInterrupt()
extern uint16_t fake_tof_mm;       // getDistance()
extern uint8_t  fake_tof_status;   // getRangeStatus(): 0 = valid
struct SFEVL53L1X {
  int      begin() { return fake_tof_begin; }
  void     setDistanceModeShort() {}
  void     setTimingBudgetInMs(uint16_t) {}
  void     setIntermeasurementPeriod(uint16_t) {}
  void     startRanging() {}
  bool     checkForDataReady() { return fake_tof_ready; }
  uint16_t getDistance() { return fake_tof_mm; }
  uint8_t  getRangeStatus() { return fake_tof_status; }
  void     clearInterrupt() { fake_tof_ready = false; }
};
