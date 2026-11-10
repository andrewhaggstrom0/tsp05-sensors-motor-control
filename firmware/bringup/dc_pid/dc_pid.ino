// DC motor closed-loop bring-up using the real dc_motor module.
// Run tools/sync_dc_bringup.sh first: it copies the module files into this folder.
// Serial (115200, Newline): p<deg> move relative, v<rpm> hold speed, x stop, i1/i0 interlock on/off.
// Streams "ms,mode,target,deg,rpm,pwm" at 50 Hz (mode: 0 idle, 1 velocity, 2 position).

#include "config.h"
#include "dc_motor.h"

const unsigned long PRINT_MS = 20;

int   mode = 0;
float target = 0.0f;

// Non-blocking line reader. Returns true when a full line is in buf.
bool readLine(char *buf, byte size) {
  static byte len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len == 0) continue;
      buf[len] = '\0';
      len = 0;
      return true;
    }
    if (len < size - 1) buf[len++] = c;
  }
  return false;
}

void handleCommand(const char *line) {
  float value = atof(line + 1);
  switch (line[0]) {
    case 'p': target = dcDeg() + value; mode = 2; dcMoveRelative(value); break;
    case 'v': target = value;           mode = 1; dcSetVelocity(value);  break;
    case 'x': target = 0.0f;            mode = 0; dcStop();              break;
    case 'i': dcSetInterlock(value != 0.0f); if (value != 0.0f) mode = 0; break;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD);
  dcInit();
  Serial.println(F("# tsp05 dc bringup: p<deg> v<rpm> x i1/i0"));
  Serial.println(F("ms,mode,target,deg,rpm,pwm"));
}

void loop() {
  dcUpdate();

  char line[12];
  if (readLine(line, sizeof(line))) handleCommand(line);

  static unsigned long lastMs = 0;
  unsigned long now = millis();
  if (now - lastMs >= PRINT_MS) {
    lastMs = now;
    Serial.print(now);         Serial.print(',');
    Serial.print(mode);        Serial.print(',');
    Serial.print(target, 1);   Serial.print(',');
    Serial.print(dcDeg(), 1);  Serial.print(',');
    Serial.print(dcRpm(), 1);  Serial.print(',');
    Serial.println(dcPwm());
  }
}
