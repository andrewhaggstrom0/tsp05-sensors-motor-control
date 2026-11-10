#include "config.h"
#include "dc_control.h"

namespace dc_control {

static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static float absf(float x) { return x < 0.0f ? -x : x; }
static long roundToLong(float x) { return (long)(x >= 0.0f ? x + 0.5f : x - 0.5f); }

// Steady speed (counts/s) at each PWM, from calibration/data/dc_speed_map.csv; the
// 0 counts/s point is where a turning motor stops (calibration/data/dc_deadband.csv).
static const float FF_PWM[]     = {58.0f, 63.0f, 68.0f, 75.0f, 85.0f, 95.0f, 110.0f, 130.0f, 155.0f, 180.0f, 205.0f, 230.0f, 255.0f};
static const float FF_CPS_FWD[] = {0.0f, 71.7f, 94.0f, 123.0f, 161.0f, 194.2f, 237.5f, 280.0f, 312.5f, 341.0f, 360.0f, 379.0f, 422.0f};
static const float FF_CPS_REV[] = {0.0f, 55.8f, 73.0f, 99.0f, 136.0f, 167.5f, 211.7f, 256.7f, 304.2f, 339.2f, 361.7f, 385.0f, 445.8f};
static const int   FF_PTS       = sizeof(FF_PWM) / sizeof(FF_PWM[0]);

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

float feedforwardPwm(float rpm) {
  if (rpm == 0.0f) return 0.0f;
  const float *cps = rpm > 0.0f ? FF_CPS_FWD : FF_CPS_REV;
  float want = absf(rpm) * DC_COUNTS_PER_REV / 60.0f;
  float pwm = FF_PWM[FF_PTS - 1];
  for (int i = 1; i < FF_PTS; i++) {
    if (want <= cps[i]) {
      pwm = FF_PWM[i - 1] + (FF_PWM[i] - FF_PWM[i - 1]) * (want - cps[i - 1]) / (cps[i] - cps[i - 1]);
      break;
    }
  }
  return rpm > 0.0f ? pwm : -pwm;
}

void pidReset(Pid &p) {
  p.integ = 0.0f;
  p.prevErr = 0.0f;
  p.primed = false;
}

// Integral is clamped so ki * integ stays within +/-iLimit (anti-windup);
// derivative is skipped on the first step after a reset to avoid a kick.
static void integrate(Pid &p, float err, float dtS) {
  if (p.ki > 0.0f) {
    float lim = p.iLimit / p.ki;
    p.integ = clampf(p.integ + err * dtS, -lim, lim);
  }
}

float pidStep(Pid &p, float err, float dtS) {
  if (dtS <= 0.0f) return 0.0f;
  integrate(p, err, dtS);
  float d = p.primed ? (err - p.prevErr) / dtS : 0.0f;
  p.prevErr = err;
  p.primed = true;
  return p.kp * err + p.ki * p.integ + p.kd * d;
}

float pidStepRate(Pid &p, float err, float errRate, float dtS) {
  if (dtS <= 0.0f) return 0.0f;
  integrate(p, err, dtS);
  p.prevErr = err;
  p.primed = true;
  return p.kp * err + p.ki * p.integ + p.kd * errRate;
}

void ctrlInit(Ctrl &c, long counts) {
  c.mode = IDLE;
  c.lastCounts = counts;
  c.targetCounts = counts;
  c.targetRpm = 0.0f;
  c.rpm = 0.0f;
  c.arrived = true;
  c.breakaway = DC_DEADBAND_START_PWM;
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

static int coast(Ctrl &c) {
  c.breakaway = DC_DEADBAND_START_PWM;
  return 0;
}

// Stiction varies with shaft angle, so while commanded but stopped the breakaway
// deadband ramps up until the motor starts, then resets.
// Velocity mode: pwm already includes the deadband (feedforward), so only the
// breakaway minimum applies while stopped.
static int applyBreakaway(Ctrl &c, float pwm, float dtS) {
  pwm = clampf(pwm, -255.0f, 255.0f);
  if (absf(c.rpm) > DC_MOVING_RPM) {
    c.breakaway = DC_DEADBAND_START_PWM;
    return (int)roundToLong(pwm);
  }
  if (absf(pwm) < 0.5f) return 0;
  float mag = absf(pwm) > c.breakaway ? absf(pwm) : c.breakaway;
  c.breakaway = clampf(c.breakaway + DC_BREAKAWAY_RAMP * dtS, DC_DEADBAND_START_PWM, DC_BREAKAWAY_MAX_PWM);
  return (int)roundToLong(pwm > 0.0f ? mag : -mag);
}

static int applyDeadband(Ctrl &c, float u, float dtS) {
  if (absf(c.rpm) > DC_MOVING_RPM) {
    c.breakaway = DC_DEADBAND_START_PWM;
    return deadbandComp(u, DC_DEADBAND_RUN_PWM);
  }
  int pwm = deadbandComp(u, (int)c.breakaway);
  if (pwm != 0)
    c.breakaway = clampf(c.breakaway + DC_BREAKAWAY_RAMP * dtS, DC_DEADBAND_START_PWM, DC_BREAKAWAY_MAX_PWM);
  return pwm;
}

int ctrlStep(Ctrl &c, long counts, float dtS) {
  float raw = rpmFromCounts(counts - c.lastCounts, dtS);
  c.lastCounts = counts;
  c.rpm += DC_RPM_EMA_ALPHA * (raw - c.rpm);

  float u = 0.0f;
  switch (c.mode) {
    case IDLE:
      return coast(c);

    case VELOCITY:
      if (c.targetRpm == 0.0f) { pidReset(c.vel); return coast(c); }
      return applyBreakaway(c, feedforwardPwm(c.targetRpm) + pidStep(c.vel, c.targetRpm - c.rpm, dtS), dtS);

    case POSITION: {
      float errDeg = degFromCounts(c.targetCounts - counts);
      if (absf(errDeg) <= DC_POS_TOL_DEG) {
        pidReset(c.pos);
        c.arrived = true;
        return coast(c);
      }
      c.arrived = false;
      u = clampf(pidStepRate(c.pos, errDeg, -c.rpm * 6.0f, dtS), -DC_POS_MAX_PWM, DC_POS_MAX_PWM);
      break;
    }
  }
  return applyDeadband(c, u, dtS);
}

}
