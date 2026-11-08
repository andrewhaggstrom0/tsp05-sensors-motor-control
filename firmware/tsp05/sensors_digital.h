#pragma once
// Track 3: VL53L1X time-of-flight (I2C on A4/A5) and EE-SX1042 slot sensor (D3, INT1).
// Call digitalSensorsInit() once from setup() and digitalSensorsUpdate() every
// loop(). Update only reads the TOF when a new range is ready; never blocks.

void  digitalSensorsInit();
void  digitalSensorsUpdate();

// TOF
int   tofRawMm();          // latest range, mm
float tofFiltMm();         // filtered range, mm

// Slot sensor
long  slotCount();         // total beam-break events since boot
float slotRpm();           // events / slots-per-rev over a window; 0 when idle
int   slotTakeEvents();    // new events since the last call (used to jog the stepper)
