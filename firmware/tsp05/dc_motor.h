#pragma once
// Track 1: DC motor (L298 on D5/D7/D8) + Hall encoder (D2/D4) + PID.
// Call dcInit() once from setup() and dcUpdate() every loop(). Update
// self-limits to its control period (10 ms) and never blocks.

void  dcInit();
void  dcUpdate();

// Commands
void  dcMoveRelative(float deg);   // position mode: move deg from the current position
void  dcSetVelocity(float rpm);    // velocity mode: hold signed rpm
void  dcStop();
void  dcSetInterlock(bool active); // true = thermal interlock: stop and ignore commands

// Getters (latest values, cheap to call)
float dcDeg();                     // output-shaft position, deg
float dcRpm();                     // measured output-shaft speed, rpm
int   dcPwm();                     // last command sent to the L298, -255..255
