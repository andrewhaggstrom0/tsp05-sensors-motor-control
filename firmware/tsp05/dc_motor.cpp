#include <Arduino.h>
#include "config.h"
#include "dc_motor.h"

// STUB: compiles and does nothing. Track 1 replaces the bodies.

void  dcInit() {}
void  dcUpdate() {}

void  dcMoveRelative(float deg) { (void)deg; }
void  dcSetVelocity(float rpm) { (void)rpm; }
void  dcStop() {}
void  dcSetInterlock(bool active) { (void)active; }

float dcDeg() { return 0.0f; }
float dcRpm() { return 0.0f; }
int   dcPwm() { return 0; }
