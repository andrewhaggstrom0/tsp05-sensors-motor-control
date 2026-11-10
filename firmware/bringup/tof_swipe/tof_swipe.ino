// TOF swipe bring-up: alternates the VL53L1X between its left and right halves,
// detects hand swipes, and sweeps the servo slowly in the swipe's direction.
// Run tools/sync_tof_swipe.sh first: it copies config.h and tof_gesture.* here.
// Needs the "SparkFun VL53L1X 4m Laser Distance Sensor" library.
// Serial (115200): prints "ms,zone,mm,status,servo_deg" per reading and "# swipe ..." per swipe.

#include <Wire.h>
#include <Servo.h>
#include <SparkFun_VL53L1X.h>
#include "config.h"
#include "tof_gesture.h"

using namespace tof_gesture;

const uint8_t ZONE_CENTER[2] = {GESTURE_CENTER_A, GESTURE_CENTER_B};
const unsigned long SERVO_PERIOD_MS = 20;

SFEVL53L1X tof;
Servo      servo;
Detector   det;
Zone       zone = LEFT;
bool       tofOk = false;
float      servoDeg = SERVO_START_DEG;
int8_t     sweepDir = 0;      // +1 toward 180, -1 toward 0, 0 = still

void writeServo() {
  long us = SERVO_MIN_US + (long)((SERVO_MAX_US - SERVO_MIN_US) * servoDeg / 180.0f + 0.5f);
  servo.writeMicroseconds((int)us);
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  servo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  writeServo();

  Wire.begin();
  Wire.setClock(TOF_I2C_HZ);
  tofOk = (tof.begin() == 0);
  if (!tofOk) {
    Serial.println(F("# VL53L1X not found: check SDA/SCL and 3.3 V"));
    return;
  }
  tof.setDistanceModeShort();
  tof.setTimingBudgetInMs(TOF_TIMING_BUDGET_MS);
  tof.setIntermeasurementPeriod(TOF_PERIOD_MS);
  tof.setROI(GESTURE_ROI_W, GESTURE_ROI_H, ZONE_CENTER[zone]);
  tof.startRanging();
  reset(det);
  Serial.println(F("# tof swipe bringup"));
  Serial.println(F("ms,zone,mm,status,servo_deg"));
}

// Reads a finished range, then points the next one at the other zone.
void pollTof() {
  static unsigned long lastPollMs = 0;
  unsigned long now = millis();
  if (now - lastPollMs < TOF_POLL_MS) return;
  lastPollMs = now;
  if (!tof.checkForDataReady()) return;

  uint16_t mm = tof.getDistance();
  uint8_t status = tof.getRangeStatus();
  Zone measured = zone;
  zone = (zone == LEFT) ? RIGHT : LEFT;
  tof.setROI(GESTURE_ROI_W, GESTURE_ROI_H, ZONE_CENTER[zone]);
  tof.clearInterrupt();

  Swipe s = update(det, measured, mm, status == 0, now);
  Serial.print(now);      Serial.print(',');
  Serial.print(measured); Serial.print(',');
  Serial.print(mm);       Serial.print(',');
  Serial.print(status);   Serial.print(',');
  Serial.println(servoDeg, 1);

  if (s == RIGHT_TO_LEFT) { sweepDir = +1; Serial.println(F("# swipe right->left: servo sweeps left")); }
  if (s == LEFT_TO_RIGHT) { sweepDir = -1; Serial.println(F("# swipe left->right: servo sweeps right")); }
}

// Moves the servo GESTURE_SERVO_DEG_S in the sweep direction and stops at the ends.
void sweepServo() {
  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs < SERVO_PERIOD_MS) return;
  float dt = (now - lastMs) / 1000.0f;
  lastMs = now;
  if (sweepDir == 0) return;
  servoDeg += sweepDir * GESTURE_SERVO_DEG_S * dt;
  if (servoDeg >= 180.0f) { servoDeg = 180.0f; sweepDir = 0; }
  if (servoDeg <= 0.0f)   { servoDeg = 0.0f;   sweepDir = 0; }
  writeServo();
}

void loop() {
  if (tofOk) pollTof();
  sweepServo();
}
