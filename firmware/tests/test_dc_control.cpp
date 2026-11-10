// Host-side tests for firmware/tsp05/dc_control.cpp against a simulated motor.
// Run: firmware/tests/run_tests.sh
#include <cstdio>
#include <cmath>
#include <cstdlib>
#include <initializer_list>
#include "config.h"
#include "dc_control.h"

using namespace dc_control;

static int g_fail = 0, g_pass = 0;
#define CHECK(cond, ...) do { if (cond) { g_pass++; } else { g_fail++; \
  printf("FAIL %s:%d  ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

// First-order motor matched to the bench measurements: steady speed from
// calibration/data/dc_speed_map.csv scaled by `gain` (0.92 = a weaker motor than
// the controller's table, to exercise the PI correction), breakaway 115-140
// forward / 110-125 reverse depending on shaft angle, keeps turning down to 58.
// tau (driven) and tauCoast (unpowered) were fitted to the hardware p90/p30/p10
// steps in calibration/data: ~1.2 deg RMS, under one encoder count.
static const double M_PWM[]     = {58, 63, 68, 75, 85, 95, 110, 130, 155, 180, 205, 230, 255};
static const double M_CPS_FWD[] = {0, 71.7, 94.0, 123.0, 161.0, 194.2, 237.5, 280.0, 312.5, 341.0, 360.0, 379.0, 422.0};
static const double M_CPS_REV[] = {0, 55.8, 73.0, 99.0, 136.0, 167.5, 211.7, 256.7, 304.2, 339.2, 361.7, 385.0, 445.8};

struct Motor {
  double rpm = 0, deg = 0;
  int    stopDb = 58;
  double gain = 0.92, tau = 0.03, tauCoast = 0.02;
  double loadRpm = 0;

  static double steadyRpm(int pwm) {
    const double *cps = pwm > 0 ? M_CPS_FWD : M_CPS_REV;
    double a = std::abs(pwm), c = cps[12];
    for (int i = 1; i < 13; i++)
      if (a <= M_PWM[i]) { c = cps[i - 1] + (cps[i] - cps[i - 1]) * (a - M_PWM[i - 1]) / (M_PWM[i] - M_PWM[i - 1]); break; }
    return (pwm > 0 ? 1 : -1) * c * 60.0 / DC_COUNTS_PER_REV;
  }

  void step(int pwm, double dt) {
    int a = std::abs(pwm);
    bool moving = std::fabs(rpm) > 1.0;
    double swing = 1.0 + std::sin(deg * 0.07);
    int startDb = pwm > 0 ? 115 + (int)(12.5 * swing) : 110 + (int)(7.5 * swing);
    double target = 0;
    if (a > (moving ? stopDb : startDb)) target = gain * steadyRpm(pwm) - (pwm > 0 ? loadRpm : -loadRpm);
    rpm += (target - rpm) * dt / (target == 0 ? tauCoast : tau);
    deg += rpm * 6.0 * dt;
  }
  long counts() const { return (long)std::floor(deg * DC_COUNTS_PER_REV / 360.0); }
};

struct Run {
  Motor m;
  Ctrl  c;
  int   pwm = 0;
  Run() { ctrlInit(c, m.counts()); }

  void sim(double seconds) {
    const double dt = DC_PERIOD_MS / 1000.0;
    for (double t = 0; t < seconds; t += dt) {
      for (int i = 0; i < 10; i++) m.step(pwm, dt / 10);
      pwm = ctrlStep(c, m.counts(), (float)dt);
    }
  }

  double meanRpm(double seconds, int *maxAbsPwm = nullptr) {
    const double dt = DC_PERIOD_MS / 1000.0;
    double sum = 0; int n = 0, mx = 0;
    for (double t = 0; t < seconds; t += dt) {
      sim(dt);
      sum += m.rpm; n++;
      if (std::abs(pwm) > mx) mx = std::abs(pwm);
    }
    if (maxAbsPwm) *maxAbsPwm = mx;
    return sum / n;
  }
};

static void test_math() {
  CHECK(std::fabs(degFromCounts(countsFromDeg(90.0f)) - 90.0f) <= 360.0f / DC_COUNTS_PER_REV,
        "deg->counts->deg round trip off by more than one count");
  CHECK(countsFromDeg(-90.0f) == -countsFromDeg(90.0f), "countsFromDeg not symmetric");
  CHECK(std::fabs(rpmFromCounts((long)DC_COUNTS_PER_REV, 1.0f) - 60.0f) < 1e-3, "1 rev/s should be 60 rpm");
  CHECK(rpmFromCounts(10, 0.0f) == 0.0f, "dt = 0 must not divide by zero");

  CHECK(deadbandComp(0.0f, 60) == 0, "zero command must stay zero");
  CHECK(deadbandComp(1.0f, 60) >= 60, "small command must clear the deadband");
  CHECK(deadbandComp(255.0f, 60) == 255 && deadbandComp(-255.0f, 60) == -255, "full scale maps to 255");
  CHECK(deadbandComp(999.0f, 60) == 255, "out-of-range command must clamp");
  int prev = 0;
  bool mono = true;
  for (int u = 1; u <= 255; u++) {
    int p = deadbandComp((float)u, 60);
    if (p < prev) mono = false;
    if (deadbandComp((float)-u, 60) != -p) mono = false;
    prev = p;
  }
  CHECK(mono, "deadbandComp must be monotonic and odd-symmetric");

  CHECK(feedforwardPwm(0.0f) == 0.0f, "zero speed needs zero feedforward");
  CHECK(std::fabs(feedforwardPwm(194.2f * 60.0f / DC_COUNTS_PER_REV) - 95.0f) < 0.1f, "table point fwd 194.2 cps -> 95");
  CHECK(std::fabs(feedforwardPwm(-167.5f * 60.0f / DC_COUNTS_PER_REV) + 95.0f) < 0.1f, "table point rev 167.5 cps -> -95");
  CHECK(feedforwardPwm(500.0f) == 255.0f, "beyond the table saturates at 255");
  bool ffMono = true;
  for (float r = 1.0f; r < 150.0f; r += 1.0f) {
    if (feedforwardPwm(r + 1.0f) < feedforwardPwm(r)) ffMono = false;
    if (feedforwardPwm(-r - 1.0f) > feedforwardPwm(-r)) ffMono = false;
  }
  CHECK(ffMono, "feedforward must rise with speed in both directions");
}

static void test_pid() {
  Pid p = {1.0f, 10.0f, 1.0f, 50.0f, 0, 0, false};
  float first = pidStep(p, 5.0f, 0.01f);
  CHECK(std::fabs(first - (5.0f + 10.0f * 0.05f)) < 1e-4, "first step must have no derivative kick (got %.3f)", first);
  for (int i = 0; i < 10000; i++) pidStep(p, 100.0f, 0.01f);
  CHECK(std::fabs(p.ki * p.integ) <= 50.0f + 1e-3, "integral must be clamped to iLimit (got %.1f)", p.ki * p.integ);
  pidReset(p);
  CHECK(p.integ == 0.0f && !p.primed, "reset must clear state");
}

static void test_velocity(float target) {
  Run r;
  ctrlSetVelocity(r.c, target);
  r.sim(2.0);
  double mean = r.meanRpm(0.5);
  CHECK(std::fabs(mean - target) <= 0.05 * std::fabs(target) + 1.0,
        "velocity %.0f rpm: settled at %.1f rpm", target, mean);
}

static void test_velocity_load() {
  Run r;
  ctrlSetVelocity(r.c, 60.0f);
  r.sim(1.5);
  r.m.loadRpm = 25;
  r.sim(2.0);
  double mean = r.meanRpm(0.5);
  CHECK(std::fabs(mean - 60.0) <= 4.0, "velocity under load: settled at %.1f rpm, want 60", mean);
}

static void test_position(float moveDeg) {
  Run r;
  double start = r.m.deg;
  ctrlSetPosition(r.c, r.m.counts() + countsFromDeg(moveDeg));
  r.sim(2.5);
  double moved = r.m.deg - start;
  CHECK(std::fabs(moved - moveDeg) <= DC_POS_TOL_DEG + 360.0 / DC_COUNTS_PER_REV + 1.0,
        "position %+.0f deg: moved %.2f", moveDeg, moved);
  int maxPwm = 0;
  r.meanRpm(0.5, &maxPwm);
  CHECK(maxPwm == 0 && r.c.arrived, "position %+.0f deg: still driving after settling (max pwm %d)", moveDeg, maxPwm);
}

static void test_stop() {
  Run r;
  ctrlSetVelocity(r.c, 80.0f);
  r.sim(1.0);
  ctrlStop(r.c);
  r.sim(0.05);
  CHECK(r.pwm == 0, "stop must output 0 (got %d)", r.pwm);
  r.sim(1.0);
  CHECK(std::fabs(r.m.rpm) < 1.0, "motor should coast to rest after stop (rpm %.1f)", r.m.rpm);
}

int main() {
  test_math();
  test_pid();
  for (float v : {30.0f, 60.0f, 100.0f, -60.0f}) test_velocity(v);
  test_velocity_load();
  for (float d : {10.0f, 90.0f, -90.0f, 360.0f, -720.0f}) test_position(d);
  test_stop();
  printf("dc_control: %d passed, %d failed\n", g_pass, g_fail);
  return g_fail ? 1 : 0;
}
