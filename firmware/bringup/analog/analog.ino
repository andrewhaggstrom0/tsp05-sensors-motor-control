// Track 4 bring-up: FSR (A0) + thermistor (A1), standalone.
// Streams "ms,fsr_adc,th_adc" at 50 Hz for tools/log_serial.py.
// Open the Serial Monitor at 115200 to eyeball it: press the FSR and
// fsr_adc rises; pinch the thermistor and th_adc falls.
//
// Samples exactly like firmware/tsp05/sensors_analog.cpp (discard the first
// read after a channel switch; thermistor = mean of 4), so counts recorded
// here mean the same thing to the real module. Keep the pins and constants
// below in sync with firmware/tsp05/config.h.

const uint8_t  PIN_FSR = A0;          // 5V -> FSR -> A0 -> 10k -> GND
const uint8_t  PIN_THERM = A1;        // 5V -> 10k -> A1 -> thermistor -> GND
const uint32_t PERIOD_MS = 20;        // 50 Hz, same as ANALOG_PERIOD_MS
const uint8_t  THERM_OVERSAMPLE = 4;  // same as THERM_OVERSAMPLE

uint32_t lastMs = 0;

int readSettled(uint8_t pin) {
  analogRead(pin);                    // first read after a switch is unreliable
  return analogRead(pin);
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_FSR, INPUT);
  pinMode(PIN_THERM, INPUT);
  Serial.println(F("# tsp05 analog bringup v1"));
  Serial.println(F("# ms,fsr_adc,th_adc"));
  lastMs = millis();
}

void loop() {
  uint32_t now = millis();
  if (now - lastMs < PERIOD_MS) return;
  lastMs += PERIOD_MS;                // fixed rate, no drift

  int fsr = readSettled(PIN_FSR);
  long sum = readSettled(PIN_THERM);
  for (uint8_t k = 1; k < THERM_OVERSAMPLE; k++) sum += analogRead(PIN_THERM);

  Serial.print(now);
  Serial.print(',');
  Serial.print(fsr);
  Serial.print(',');
  Serial.println((float)sum / THERM_OVERSAMPLE, 2);
}
