// Minimal stepper test: 1 rev forward, pause, 1 rev back. Repeats.
const int STEP = 10, DIR = 11, EN = 12;

void setup() {
  pinMode(STEP, OUTPUT); pinMode(DIR, OUTPUT); pinMode(EN, OUTPUT);
  digitalWrite(EN, LOW);                 // driver on
}

void loop() {
  for (int dir = 0; dir < 2; dir++) {
    digitalWrite(DIR, dir);
    for (int i = 0; i < 1600; i++) {     // 1600 microsteps = 1 rev at 1/8
      digitalWrite(STEP, HIGH); delayMicroseconds(5);
      digitalWrite(STEP, LOW);  delayMicroseconds(2000);   // slow: ~0.4 rev/s
    }
    delay(1000);
  }
}