#include "config.h"
#include "dc_control.h"

namespace dc_control {

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float absf(float x) { return x < 0.0f ? -x : x; }
static long roundToLong(float x) { return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f); }

float degFromCounts(long counts) { return counts * 360.0f / DC_COUNTS_PER_REV; }

long countsFromDeg(float deg) { return roundToLong(deg * DC_COUNTS_PER_REV / 360.0f); }

float rpmFromCounts(long dCounts, float dtS) {
  if (dtS <= 0.0f) return 0.0f;
  return dCounts / DC_COUNTS_PER_REV / dtS * 60.0f;
}

int deadbandComp(float u, int deadband) {
  u = clampf(u, -255.0f, 255.0f);
  if (absf(u) < 0.5f) return 0;
  float mag = deadband + absf(u) * (255 - deadband) / 255.0f;
  int pwm = (int)roundToLong(mag);
  return u > 0 ? pwm : -pwm;
}

void pidReset(Pid &p) {
  p.integ = 0.0f;
  p.prevErr = 0.0f;
  p.primed = false;
}

// Integral is clamped so ki * integ stays within +/-iLimit (anti-windup);
// derivative is skipped on the first step after a reset to avoid a kick.
float pidStep(Pid &p, float err, float dtS) {
  if (dtS <= 0.0f) return 0.0f;
  if (p.ki > 0.0f) {
    float lim = p.iLimit / p.ki;
    p.integ = clampf(p.integ + err * dtS, -lim, lim);
  }
  float d = p.primed ? (err - p.prevErr) / dtS : 0.0f;
  p.prevErr = err;
  p.primed = true;
  return p.kp * err + p.ki * p.integ + p.kd * d;
}

void ctrlInit(Ctrl &c, long counts) {
  c.mode = IDLE;
  c.lastCounts = counts;
  c.targetCounts = counts;
  c.targetRpm = 0.0f;
  c.rpm = 0.0f;
  c.arrived = true;
  c.vel = {DC_VEL_KP, DC_VEL_KI, 0.0f, DC_VEL_I_LIMIT, 0.0f, 0.0f, false};
  c.pos = {DC_POS_KP, DC_POS_KI, DC_POS_KD, DC_POS_I_LIMIT, 0.0f, 0.0f, false};
}

void ctrlSetVelocity(Ctrl &c, float rpm) {
  if (c.mode != VELOCITY) pidReset(c.vel);
  c.mode = VELOCITY;
  c.targetRpm = clampf(rpm, -DC_MAX_RPM, DC_MAX_RPM);
}

void ctrlSetPosition(Ctrl &c, long targetCounts) {
  pidReset(c.pos);
  c.mode = POSITION;
  c.targetCounts = targetCounts;
  c.arrived = false;
}

void ctrlStop(Ctrl &c) {
  c.mode = IDLE;
  c.targetRpm = 0.0f;
}

int ctrlStep(Ctrl &c, long counts, float dtS) {
  float raw = rpmFromCounts(counts - c.lastCounts, dtS);
  c.lastCounts = counts;
  c.rpm += DC_RPM_EMA_ALPHA * (raw - c.rpm);

  float u = 0.0f;
  switch (c.mode) {
    case IDLE:
      return 0;

    case VELOCITY:
      if (c.targetRpm == 0.0f) { pidReset(c.vel); return 0; }
      u = c.targetRpm * 255.0f / DC_MAX_RPM + pidStep(c.vel, c.targetRpm - c.rpm, dtS);
      break;

    case POSITION: {
      float errDeg = degFromCounts(c.targetCounts - counts);
      if (absf(errDeg) <= DC_POS_TOL_DEG) {
        pidReset(c.pos);
        c.arrived = true;
        return 0;
      }
      c.arrived = false;
      u = clampf(pidStep(c.pos, errDeg, dtS), -DC_POS_MAX_PWM, DC_POS_MAX_PWM);
      break;
    }
  }
  return deadbandComp(u, DC_DEADBAND_PWM);
}

}
