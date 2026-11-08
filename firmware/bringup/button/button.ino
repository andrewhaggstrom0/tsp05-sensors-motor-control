// Button test (Track 2): A2 -> button -> GND, internal pull-up.
// Prints the raw pin once a second, and a counter on every debounced press.
// Target: 50 deliberate presses = count 50.

const int PIN_BUTTON = A2;
const unsigned long DEBOUNCE_MS = 30;

bool raw = false, down = false;
unsigned long changedMs = 0, lastRawPrint = 0;
long presses = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  raw = down = (digitalRead(PIN_BUTTON) == LOW);
  Serial.println(F("Button test. Released should read HIGH, pressed LOW."));
}

void loop() {
  unsigned long now = millis();
  bool r = (digitalRead(PIN_BUTTON) == LOW);

  if (r != raw) { raw = r; changedMs = now; }                 // bouncing: restart timer
  else if (r != down && now - changedMs >= DEBOUNCE_MS) {     // stable for 30 ms
    down = r;
    if (down) { presses++; Serial.print(F("press #")); Serial.println(presses); }
  }

  if (now - lastRawPrint >= 1000) {
    lastRawPrint = now;
    Serial.print(F("raw pin = ")); Serial.println(digitalRead(PIN_BUTTON) ? F("HIGH (released)") : F("LOW (pressed)"));
  }
}