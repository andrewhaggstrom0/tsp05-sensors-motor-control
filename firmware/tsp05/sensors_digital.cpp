#include <Arduino.h>
#include "config.h"
#include "sensors_digital.h"

// STUB: compiles and does nothing. Track 3 replaces the bodies.

void  digitalSensorsInit() {}
void  digitalSensorsUpdate() {}

int   tofRawMm() { return 0; }
float tofFiltMm() { return 0.0f; }

long  slotCount() { return 0; }
float slotRpm() { return 0.0f; }
int   slotTakeEvents() { return 0; }
