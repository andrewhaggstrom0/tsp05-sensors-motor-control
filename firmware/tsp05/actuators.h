#pragma once
// Track 2: servo (D9) and stepper via EasyDriver (D10 STEP, D11 DIR, D12 EN).
// Call actuatorsInit() once from setup() and actuatorsUpdate() every loop().
// Stepping is non-blocking: update emits step pulses as they come due.

void  actuatorsInit();
void  actuatorsUpdate();

// Servo
void  servoSetDeg(float deg);        // absolute angle, clamped to the measured limits
float servoDeg();                    // last commanded angle, deg

// Stepper
void  stepperMoveRelative(float deg); // signed, rounded to the nearest step
float stepperDeg();                   // actual position (whole steps), deg
float stepperTargetDeg();             // where the current move ends, deg
