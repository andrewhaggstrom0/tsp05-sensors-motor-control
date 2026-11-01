#pragma once
// TSP05 pin map and constants. Change pins here only.

// ---------- DC motor (L298 + Hall encoder) ----------
#define PIN_ENC_A        2    // INT0
#define PIN_ENC_B        4    // read inside ENC_A ISR
#define PIN_DC_EN        5    // PWM (Timer0, unaffected by Servo lib)
#define PIN_DC_IN1       7
#define PIN_DC_IN2       8

// ---------- Slot sensor (EE-SX1042) ----------
#define PIN_SLOT         3    // INT1, 10k pull-up to 5V

// ---------- Servo (HS-311) ----------
#define PIN_SERVO        9    // Servo lib uses Timer1: no PWM on 9/10

// ---------- Stepper (EasyDriver) ----------
#define PIN_STEP         10
#define PIN_DIR          11
#define PIN_STEP_EN      12   // active LOW

// ---------- Analog sensors ----------
#define PIN_FSR          A0   // 5V -> FSR -> A0 -> 10k -> GND
#define PIN_THERM        A1   // 5V -> 10k -> A1 -> thermistor -> GND

// ---------- User input ----------
#define PIN_BUTTON       A2   // INPUT_PULLUP, pressed = LOW

// ---------- I2C (VL53L1X) ----------
// SDA = A4, SCL = A5 (fixed on Uno)

// ---------- Serial ----------
#define SERIAL_BAUD      115200
