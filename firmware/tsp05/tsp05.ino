// TSP05 main sketch (Track 2 owns this file).
// Every module follows the same pattern: xxxInit() once in setup(), xxxUpdate()
// every loop(). Each update self-limits to its own rate and never blocks, so
// loop() just calls them all as fast as it can.
//
// Track 2 adds here: serial command parser, GUI/SENSOR mode, SENSOR-mode
// mappings, thermal interlock, and the 20 Hz telemetry line (docs/protocol.md).
// SENSOR mode: TOF -> servo, flex bend -> DC rpm, slot event -> stepper jog.

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "config.h"
#include "sensors_analog.h"    // Track 4
#include "sensors_digital.h"   // Track 3
#include "dc_motor.h"          // Track 1
#include "actuators.h"         // Track 2
#include "button.h"            // Track 2

// Plain uint8_t (not an enum type) so the IDE's auto-generated prototypes compile.
const uint8_t MODE_GUI    = 0;
const uint8_t MODE_SENSOR = 1;

static uint8_t  g_mode      = MODE_GUI;
static bool     g_interlock = false;

// ===========================================================================
// Mode
// ===========================================================================
static float g_lastMapRpm = 0.0f;
static bool  g_mapRpmSent = false;

static void setMode(uint8_t m) {
  if (m == g_mode) return;
  g_mode = m;
  dcStop();                 // either way, start the new mode from a stopped DC motor
  g_mapRpmSent = false;     // SENSOR mode re-sends its first RPM
  slotTakeEvents();         // drop slot events from before the switch
}

// ===========================================================================
// Serial commands (docs/protocol.md). Read without waiting; act on '\n'.
// ===========================================================================
static char    g_rx[32];
static uint8_t g_rxLen      = 0;
static bool    g_rxOverflow = false;

static void handleCommand(char *line) {
  // Mode works in both modes.
  if (line[0] == 'M' && line[1] == ',') {
    int m = atoi(line + 2);
    if (m == 0) setMode(MODE_GUI);
    else if (m == 1) setMode(MODE_SENSOR);
    return;
  }

  // Motor commands are ignored in SENSOR mode.
  if (g_mode != MODE_GUI) return;

  if (line[0] == 'S' && line[1] == ',') {
    servoSetDeg(atof(line + 2));
  } else if (line[0] == 'P' && line[1] == ',') {
    stepperMoveRelative(atof(line + 2));
  } else if (line[0] == 'D' && line[1] == ',') {
    char sub = line[2];
    if (sub == 'X')                         dcStop();
    else if (sub == 'P' && line[3] == ',')  dcMoveRelative(atof(line + 4));
    else if (sub == 'V' && line[3] == ',')  dcSetVelocity(atof(line + 4));
  }
  // Anything else is silently ignored.
}

static void readCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (!g_rxOverflow && g_rxLen > 0) {
        g_rx[g_rxLen] = '\0';
        handleCommand(g_rx);
      }
      g_rxLen = 0;
      g_rxOverflow = false;
    } else if (g_rxLen < sizeof(g_rx) - 1) {
      g_rx[g_rxLen++] = c;
    } else {
      g_rxOverflow = true;  // line too long: throw it away at the next '\n'
    }
  }
}

// ===========================================================================
// SENSOR-mode mappings (ranges in config.h)
// ===========================================================================
static float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

static void runSensorMappings() {
  // TOF distance -> servo angle
  float mm = clampf(tofFiltMm(), MAP_TOF_MIN_MM, MAP_TOF_MAX_MM);
  float t  = (mm - MAP_TOF_MIN_MM) / (float)(MAP_TOF_MAX_MM - MAP_TOF_MIN_MM);
  servoSetDeg(MAP_SERVO_MIN_DEG + t * (MAP_SERVO_MAX_DEG - MAP_SERVO_MIN_DEG));

  // Flex bend angle -> DC RPM, with a dead zone. A flex fault means 0 rpm.
  // Only send when it actually changes.
  float bend = flexDegFiltered();
  float rpm  = 0.0f;
  if (!flexFault() && bend > MAP_FLEX_DEADZONE_DEG) {
    float f = clampf((bend - MAP_FLEX_DEADZONE_DEG) / (MAP_FLEX_FULL_DEG - MAP_FLEX_DEADZONE_DEG), 0.0f, 1.0f);
    rpm = f * MAP_DC_MAX_RPM;
  }
  if (!g_mapRpmSent || fabs(rpm - g_lastMapRpm) >= 0.5f) {
    dcSetVelocity(rpm);
    g_lastMapRpm = rpm;
    g_mapRpmSent = true;
  }

  // Each new slot event -> one fixed stepper jog (each event consumed once)
  int ev = slotTakeEvents();
  if (ev > 0) stepperMoveRelative(ev * MAP_SLOT_JOG_DEG);
}

// ===========================================================================
// Thermal interlock: active in both modes
// ===========================================================================
static void updateInterlock() {
  bool want = thermistorFault() || thermistorC() > INTERLOCK_TEMP_C;
  if (want != g_interlock) {
    g_interlock = want;
    dcSetInterlock(want);
  }
}

// ===========================================================================
// Telemetry. The line is built into a buffer and sent only as fast as the
// 64-byte Serial TX buffer has room, so printing never stalls the loop.
// ===========================================================================
static char     g_tx[160];      // longest possible line is ~141 chars
static uint8_t  g_txLen = 0;
static uint8_t  g_txPos = 0;

static void addStr(const char *s) {
  while (*s && g_txLen < sizeof(g_tx) - 1) g_tx[g_txLen++] = *s++;
}

static void addLong(long v) {
  char tmp[12];
  ltoa(v, tmp, 10);
  addStr(",");
  addStr(tmp);
}

static void addFloat(float v, uint8_t decimals) {   // fixed point, no printf %f on AVR
  long scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= 10;
  long x = (long)(v * scale + (v >= 0 ? 0.5f : -0.5f));
  addStr(",");
  if (x < 0) { addStr("-"); x = -x; }
  char tmp[12];
  ltoa(x / scale, tmp, 10);
  addStr(tmp);
  if (decimals) {
    addStr(".");
    long frac = x % scale;
    for (long d = scale / 10; d > 0; d /= 10) {     // keep leading zeros (e.g. 1.05)
      char c[2] = { (char)('0' + (frac / d) % 10), '\0' };
      addStr(c);
    }
  }
}

static void buildTelemetry() {
  g_txLen = 0;
  g_txPos = 0;
  addStr("T");
  addLong((long)millis());
  addLong(g_mode);
  addLong(flexRaw());
  addFloat(flexDeg(), 1);
  addFloat(flexDegFiltered(), 1);
  addLong(tofRawMm());
  addFloat(tofFiltMm(), 1);
  addLong(thermistorRaw());
  addFloat(thermistorC(), 2);
  addLong(slotCount());
  addFloat(slotRpm(), 1);
  addFloat(servoDeg(), 1);
  addFloat(dcDeg(), 1);
  addFloat(dcRpm(), 1);
  addLong(dcPwm());
  addFloat(stepperDeg(), 2);
  addFloat(stepperTargetDeg(), 2);
  addLong(g_interlock ? 1 : 0);
  addStr("\n");
}

static void sendTelemetry() {
  int room = Serial.availableForWrite();
  while (room > 0 && g_txPos < g_txLen) {
    Serial.write(g_tx[g_txPos++]);
    room--;
  }
}

// ===========================================================================
static uint32_t g_lastMapMs  = 0;
static uint32_t g_lastTeleMs = 0;

void setup() {
  Serial.begin(SERIAL_BAUD);
  analogSensorsInit();
  digitalSensorsInit();
  dcInit();
  actuatorsInit();
  buttonInit();
  Serial.println(F("# TSP05 starting"));   // '#' = comment line for the bridge
}

void loop() {
  uint32_t now = millis();

  // Module updates (each self-limits to its own rate)
  analogSensorsUpdate();
  digitalSensorsUpdate();
  dcUpdate();
  actuatorsUpdate();
  buttonUpdate();

  // Every pass
  readCommands();
  if (buttonTakePress()) setMode(g_mode == MODE_GUI ? MODE_SENSOR : MODE_GUI);
  updateInterlock();

  // SENSOR-mode mappings at 50 Hz
  if ((uint32_t)(now - g_lastMapMs) >= MAPPING_PERIOD_MS) {
    g_lastMapMs = now;
    if (g_mode == MODE_SENSOR) runSensorMappings();
    else slotTakeEvents();   // GUI mode: discard slot events so none pile up
  }

  // Telemetry at 20 Hz; a new line starts only when the last one has gone out
  if ((uint32_t)(now - g_lastTeleMs) >= TELEMETRY_PERIOD_MS && g_txPos >= g_txLen) {
    g_lastTeleMs = now;
    buildTelemetry();
  }
  sendTelemetry();
}