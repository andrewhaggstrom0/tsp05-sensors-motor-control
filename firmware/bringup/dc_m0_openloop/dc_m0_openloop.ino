// M0: type a PWM value (-255..255) in the Serial Monitor (115200, Newline) to drive the motor.

const int PIN_DC_EN  = 5;
const int PIN_DC_IN1 = 7;
const int PIN_DC_IN2 = 8;

int currentPwm = 0;

// Positive = forward, negative = reverse, 0 = coast.
void dcSetPwm(int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(PIN_DC_IN1, HIGH);
    digitalWrite(PIN_DC_IN2, LOW);
  } else if (pwm < 0) {
    digitalWrite(PIN_DC_IN1, LOW);
    digitalWrite(PIN_DC_IN2, HIGH);
  } else {
    digitalWrite(PIN_DC_IN1, LOW);
    digitalWrite(PIN_DC_IN2, LOW);
  }
  analogWrite(PIN_DC_EN, abs(pwm));
  currentPwm = pwm;
}

void setup() {
  pinMode(PIN_DC_EN, OUTPUT);
  pinMode(PIN_DC_IN1, OUTPUT);
  pinMode(PIN_DC_IN2, OUTPUT);
  dcSetPwm(0);

  Serial.begin(115200);
  Serial.println(F("M0 open-loop DC test. Type -255..255 then Enter. 0 = stop."));
}

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

void loop() {
  char line[8];
  if (readLine(line, sizeof(line))) {
    dcSetPwm(atoi(line));
    Serial.print(F("PWM = "));
    Serial.println(currentPwm);
  }
}
