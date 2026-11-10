#pragma once

void  dcInit();
void  dcUpdate();                  // call every loop(); runs the controller every DC_PERIOD_MS

void  dcMoveRelative(float deg);   // position mode: move deg from current position
void  dcSetVelocity(float rpm);    // velocity mode: hold signed rpm
void  dcStop();
void  dcSetInterlock(bool active); // true: stop and ignore commands

float dcDeg();
float dcRpm();
int   dcPwm();                     // -255..255
bool  dcArrived();                 // position mode: within DC_POS_TOL_DEG of target
