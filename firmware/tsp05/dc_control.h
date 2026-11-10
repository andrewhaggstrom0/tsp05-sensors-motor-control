#pragma once
#include <stdint.h>

namespace dc_control {

enum Mode : uint8_t { IDLE, VELOCITY, POSITION };

struct Pid {
  float kp, ki, kd, iLimit;
  float integ;
  float prevErr;
  bool  primed;
};

struct Ctrl {
  Mode  mode;
  long  lastCounts;
  long  targetCounts;
  float targetRpm;
  float rpm;
  bool  arrived;
  float breakaway;
  Pid   vel, pos;
};

float degFromCounts(long counts);
long  countsFromDeg(float deg);
float rpmFromCounts(long dCounts, float dtS);

// PWM that holds this signed speed in steady state, from the measured speed map.
float feedforwardPwm(float rpm);

// Maps a command in -255..255 onto the range the motor actually moves in: 0 stays 0,
// anything else becomes +/-(deadband..255). ctrlStep uses the running deadband once
// moving; while stopped it ramps the breakaway deadband up until the motor starts.
int   deadbandComp(float u, int deadband);

void  pidReset(Pid &p);
float pidStep(Pid &p, float err, float dtS);

// Same, but with the error's rate of change supplied (e.g. -filtered speed) instead of
// differencing the error: with 2 deg encoder steps, differencing makes the D term chatter.
float pidStepRate(Pid &p, float err, float errRate, float dtS);

void  ctrlInit(Ctrl &c, long counts);
void  ctrlSetVelocity(Ctrl &c, float rpm);
void  ctrlSetPosition(Ctrl &c, long targetCounts);
void  ctrlStop(Ctrl &c);

// One control period: updates the speed estimate from the new count, returns the PWM to apply.
int   ctrlStep(Ctrl &c, long counts, float dtS);

}
