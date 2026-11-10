#include <Arduino.h>
#include "config.h"
#include "dc_control.h"
#include "dc_motor.h"

using namespace dc_control;

static volatile long encCount = 0;
static Ctrl     ctrl;
static int      pwmOut = 0;
static bool     interlock = false;
static uint32_t lastMs = 0;

// Runs on every edge of A; B gives the direction.
static void encoderIsr() {
  if (digitalRead(PIN_ENC_A) != digitalRead(PIN_ENC_B)) encCount++;
  else encCount--;
}

// Copies the count with interrupts paused so the ISR can't change it mid-read.
static long readCount() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c;
}

static void writePwm(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(PIN_DC_IN1, pwm > 0 ? HIGH : LOW);
  digitalWrite(PIN_DC_IN2, pwm < 0 ? HIGH : LOW);
  analogWrite(PIN_DC_EN, abs(pwm));
  pwmOut = pwm;
}

void dcInit() {
  pinMode(PIN_DC_EN, OUTPUT);
  pinMode(PIN_DC_IN1, OUTPUT);
  pinMode(PIN_DC_IN2, OUTPUT);
  writePwm(0);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderIsr, CHANGE);

  ctrlInit(ctrl, readCount());
  lastMs = millis();
}

void dcUpdate() {
  uint32_t now = millis();
  if (now - lastMs < DC_PERIOD_MS) return;
  float dtS = (now - lastMs) / 1000.0f;
  lastMs = now;

  int pwm = ctrlStep(ctrl, readCount(), dtS);
  writePwm(interlock ? 0 : pwm);
}

void dcMoveRelative(float deg) {
  if (interlock) return;
  ctrlSetPosition(ctrl, readCount() + countsFromDeg(deg));
}

void dcSetVelocity(float rpm) {
  if (interlock) return;
  ctrlSetVelocity(ctrl, rpm);
}

void dcStop() {
  ctrlStop(ctrl);
  writePwm(0);
}

void dcSetInterlock(bool active) {
  if (active && !interlock) dcStop();
  interlock = active;
}

float dcDeg() { return degFromCounts(readCount()); }
float dcRpm() { return ctrl.rpm; }
int   dcPwm() { return pwmOut; }
bool  dcArrived() { return ctrl.mode == POSITION && ctrl.arrived; }
