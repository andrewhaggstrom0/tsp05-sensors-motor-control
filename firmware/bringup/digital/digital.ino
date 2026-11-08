// Track 3 bring-up: VL53L1X time-of-flight (I2C, A4/A5) + EE-SX1042 slot
// sensor (D3, INT1), standalone. Proves both sensors before they go into
// firmware/tsp05/sensors_digital.cpp.
//
// Prints "ms,tof_mm,tof_status,slot_count" each time a new TOF range arrives
// (~25 Hz). Open the Serial Monitor at 115200.
//   tof_status: 0 = valid, 1 = signal fail, 2 = sigma fail, 7 = wrapped target,
//               255 = TOF not detected (slot count still printed every 100 ms)
//
// Needs the "SparkFun VL53L1X 4m Laser Distance Sensor" library
// (Arduino IDE: Library Manager).
//
// Non-blocking like the main sketch: no delay(), the TOF is polled every
// TOF_POLL_MS and read only when a range is ready.

#include <Wire.h>
#include <SparkFun_VL53L1X.h>

const uint8_t  PIN_SLOT        = 3;      // INT1, 10k pull-up to 5V; beam blocked -> HIGH
const uint16_t TOF_BUDGET_MS   = 33;     // time per range (allowed: 15,20,33,50,100,200,500)
const uint16_t TOF_PERIOD_MS   = 40;     // time between ranges; must be >= budget (default is 100!)
const uint32_t TOF_POLL_MS     = 5;      // how often to ask "data ready?" over I2C
const uint32_t SLOT_GLITCH_US  = 1000;   // ignore edges closer than this (bounce / slow edge)

SFEVL53L1X tof;
bool       tofOk       = false;
uint32_t   lastPollMs  = 0;
uint32_t   lastPrintMs = 0;

// Changed inside the ISR -> volatile
volatile uint32_t slotCount  = 0;
volatile uint32_t slotLastUs = 0;

void slotIsr() {
  uint32_t now = micros();
  if (now - slotLastUs < SLOT_GLITCH_US) return;
  slotLastUs = now;
  slotCount++;
}

uint32_t readSlotCount() {
  noInterrupts();                // 32-bit value: read it in one piece
  uint32_t n = slotCount;
  interrupts();
  return n;
}

void printLine(uint32_t ms, uint16_t mm, uint8_t status) {
  Serial.print(ms);
  Serial.print(',');
  Serial.print(mm);
  Serial.print(',');
  Serial.print(status);
  Serial.print(',');
  Serial.println(readSlotCount());
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("# tsp05 digital bringup v1"));

  pinMode(PIN_SLOT, INPUT);      // external 10k pull-up, not INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(PIN_SLOT), slotIsr, RISING);

  Wire.begin();
  Wire.setClock(400000);         // 400 kHz: shorter I2C transactions
#if defined(WIRE_HAS_TIMEOUT)
  Wire.setWireTimeout(3000, true);  // a glitched bus can't freeze the loop
#endif

  if (tof.begin() != 0) {        // SparkFun lib: 0 = success
    Serial.println(F("# TOF not found: check 3V3, SDA=A4, SCL=A5, GND"));
  } else {
    Serial.print(F("# TOF sensor id 0x"));
    Serial.println(tof.getSensorID(), HEX);   // expect EEAC
    tof.setDistanceModeShort();               // ~1.3 m max, better in ambient light
    tof.setTimingBudgetInMs(TOF_BUDGET_MS);
    tof.setIntermeasurementPeriod(TOF_PERIOD_MS);
    tof.startRanging();                       // runs continuously from here on
    tofOk = true;
  }
  Serial.println(F("# ms,tof_mm,tof_status,slot_count"));
}

void loop() {
  uint32_t now = millis();
  if (now - lastPollMs < TOF_POLL_MS) return;
  lastPollMs = now;

  if (tofOk) {
    if (!tof.checkForDataReady()) return;     // nothing new yet: don't wait
    uint16_t mm     = tof.getDistance();
    uint8_t  status = tof.getRangeStatus();
    tof.clearInterrupt();                     // arm the sensor for the next range
    printLine(now, mm, status);
  } else if (now - lastPrintMs >= 100) {      // no TOF: still show the slot count
    lastPrintMs = now;
    printLine(now, 0, 255);
  }
}
