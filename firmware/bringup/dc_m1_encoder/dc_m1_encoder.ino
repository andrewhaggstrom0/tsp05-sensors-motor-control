// M1: encoder count + open-loop drive. Serial (115200, Newline): -255..255 = PWM, z = zero count.
// Prints ms,pwm,counts,counts_per_s every 200 ms.

const int PIN_ENC_A  = 2;
const int PIN_ENC_B  = 4;
const int PIN_DC_EN  = 5;
const int PIN_DC_IN1 = 7;
const int PIN_DC_IN2 = 8;

const unsigned long PRINT_MS = 200;

volatile long encCount = 0;
int currentPwm = 0;

// Runs on every edge of A; B gives the direction.
void encoderIsr() {
  if (digitalRead(PIN_ENC_A) != digitalRead(PIN_ENC_B)) encCount++;
  else encCount--;
}

// Copies the count with interrupts paused so the ISR can't change it mid-read.
long readCount() {
  noInterrupts();
  long c = encCount;
  interrupts();
  return c;
}

// Positive = forward, negative = reverse, 0 = coast.
void dcSetPwm(int pwm) {
  pwm = constrain(pwm, -255, 255);
  digitalWrite(PIN_DC_IN1, pwm > 0 ? HIGH : LOW);
  digitalWrite(PIN_DC_IN2, pwm < 0 ? HIGH : LOW);
  analogWrite(PIN_DC_EN, abs(pwm));
  currentPwm = pwm;
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

void setup() {
  pinMode(PIN_DC_EN, OUTPUT);
  pinMode(PIN_DC_IN1, OUTPUT);
  pinMode(PIN_DC_IN2, OUTPUT);
  dcSetPwm(0);

  pinMode(PIN_ENC_A, INPUT);
  pinMode(PIN_ENC_B, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderIsr, CHANGE);

  Serial.begin(115200);
  Serial.println(F("M1 encoder test. -255..255 = PWM, z = zero count"));
  Serial.println(F("ms,pwm,counts,counts_per_s"));
}

void loop() {
  char line[8];
  if (readLine(line, sizeof(line))) {
    if (line[0] == 'z') {
      noInterrupts();
      encCount = 0;
      interrupts();
    } else {
      dcSetPwm(atoi(line));
    }
  }

  static unsigned long lastMs = 0;
  static long lastCount = 0;
  unsigned long now = millis();
  if (now - lastMs >= PRINT_MS) {
    long c = readCount();
    float cps = (c - lastCount) * 1000.0f / (now - lastMs);
    lastMs = now;
    lastCount = c;
    Serial.print(now);        Serial.print(',');
    Serial.print(currentPwm); Serial.print(',');
    Serial.print(c);          Serial.print(',');
    Serial.println(cps, 1);
  }
}
