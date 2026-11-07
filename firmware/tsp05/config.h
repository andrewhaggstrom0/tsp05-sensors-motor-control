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

// ---------- Analog sensors: conversion + filtering (Track 4) ----------
// Update rate
#define ANALOG_PERIOD_MS      20       // 50 Hz
#define ADC_MAX               1023

// Fixed divider resistors. Nominal 10k; measure yours with a multimeter
// and put the real values here (a 5% resistor is up to ~1 degC of error).
#define FSR_FIXED_R_OHMS      10000.0f // A0 -> GND
#define THERM_FIXED_R_OHMS    10000.0f // 5V -> A1

// Thermistor (Beta model). Verify B on the SparkFun datasheet.
#define THERM_R0_OHMS         10000.0f // resistance at T0
#define THERM_T0_K            298.15f  // 25 degC
#define THERM_BETA            3950.0f
#define THERM_OVERSAMPLE      4        // ADC reads averaged per update
#define THERM_ADC_SHORT       5        // at/below: shorted thermistor -> fault
#define THERM_ADC_OPEN        1018     // at/above: unplugged thermistor -> fault

// FSR
#define FSR_ADC_DEADZONE      3        // at/below: no press, report 0 N
#define FSR_MAX_N             100.0f   // clamp (Adafruit FSR rated to ~10 kg)
#define FSR_EMA_ALPHA         0.2f     // 0..1, higher = less smoothing, less lag
