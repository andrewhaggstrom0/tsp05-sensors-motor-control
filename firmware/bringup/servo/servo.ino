// Servo test: type a pulse width in microseconds, or + / - to nudge by 10 us
#include <Servo.h>

const int PIN_SERVO = 9;
const int HARD_MIN_US = 500;    // absolute safety clamp
const int HARD_MAX_US = 2500;

Servo servo;
int us = 1500;                  // start at the center, which is safe

void setup() {
  Serial.begin(115200);
  servo.attach(PIN_SERVO);
  servo.writeMicroseconds(us);
  Serial.println(F("Type a pulse in us (e.g. 1500) or + / - to nudge 10 us"));
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.peek();
  if (c == '+' || c == '-') { Serial.read(); us += (c == '+') ? 10 : -10; }
  else if (isDigit(c))       { us = Serial.parseInt(); }
  else                       { Serial.read(); return; }   // skip newlines and spaces

  us = constrain(us, HARD_MIN_US, HARD_MAX_US);
  servo.writeMicroseconds(us);
  Serial.print(F("pulse = ")); Serial.print(us); Serial.println(F(" us"));
}