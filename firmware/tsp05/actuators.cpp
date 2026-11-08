#include <Arduino.h>
#include <Servo.h>
#include "config.h"
#include "actuators.h"

// ---------------------------------------------------------------------------
// Servo: 0..180 deg mapped linearly onto the measured SERVO_MIN_US..MAX_US.
// Stepper: position and target are counted in steps. actuatorsUpdate() emits
// at most one STEP pulse per call, only when STEPPER_STEP_US has passed, so
// it never blocks the loop.
// ---------------------------------------------------------------------------

static Servo    s_servo;
static float    s_servoDeg   = SERVO_START_DEG;

static long     s_posSteps   = 0;     // steps actually emitted
static long     s_tgtSteps   = 0;     // where the current move ends
static uint32_t s_lastStepUs = 0;

static long roundToLong(float x) { return (long)(x >= 0 ? x + 0.5f : x - 0.5f); }

void actuatorsInit() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_STEP_EN, OUTPUT);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, LOW);
  digitalWrite(PIN_STEP_EN, LOW);     // active LOW: driver enabled

  // Passing the limits to attach() makes the Servo library clamp to them too.
  s_servo.attach(PIN_SERVO, SERVO_MIN_US, SERVO_MAX_US);
  servoSetDeg(SERVO_START_DEG);
}

void actuatorsUpdate() {
  if (s_posSteps == s_tgtSteps) return;                       // nothing to do
  uint32_t now = micros();
  if ((uint32_t)(now - s_lastStepUs) < STEPPER_STEP_US) return; // not due yet
  s_lastStepUs = now;

  bool fwd = s_tgtSteps > s_posSteps;
  digitalWrite(PIN_DIR, fwd ? HIGH : LOW);  // DIR first, then the STEP edge
  digitalWrite(PIN_STEP, HIGH);             // EasyDriver steps on the rising edge
  digitalWrite(PIN_STEP, LOW);              // digitalWrite takes ~4 us: pulse is wide enough
  s_posSteps += fwd ? 1 : -1;
}

// ---------- Servo ----------
void servoSetDeg(float deg) {
  if (deg < 0.0f)   deg = 0.0f;
  if (deg > 180.0f) deg = 180.0f;
  s_servoDeg = deg;
  int us = SERVO_MIN_US + (int)roundToLong((SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0f);
  s_servo.writeMicroseconds(us);
}

float servoDeg() { return s_servoDeg; }

// ---------- Stepper ----------
void stepperMoveRelative(float deg) {
  s_tgtSteps += roundToLong(deg * STEPPER_STEPS_PER_REV / 360.0f);
}

float stepperDeg()       { return s_posSteps * 360.0f / STEPPER_STEPS_PER_REV; }
float stepperTargetDeg() { return s_tgtSteps * 360.0f / STEPPER_STEPS_PER_REV; }