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
#define PIN_FLEX         A0   // 5V -> flex sensor -> A0 -> 10k -> GND
#define PIN_THERM        A1   // 5V -> 10k -> A1 -> thermistor -> GND

// ---------- User input ----------
#define PIN_BUTTON       A2   // INPUT_PULLUP, pressed = LOW

// ---------- I2C (VL53L1X) ----------
// SDA = A4, SCL = A5 (fixed on Uno)

// ---------- Digital sensors: TOF + slot (Track 3) ----------
#define TOF_I2C_HZ            400000UL // 400 kHz; drop to 100000UL if the bus is flaky
#define TOF_TIMING_BUDGET_MS  33       // time per range. Allowed: 15, 20, 33, 50, 100, 200, 500
#define TOF_PERIOD_MS         40       // time between ranges, >= budget (sensor default 100 = only 10 Hz)
#define TOF_POLL_MS           5        // how often to ask the sensor "range ready?" over I2C
#define TOF_MEDIAN_N          5        // median window: removes single-sample outliers (odd, <= 9)
#define TOF_EMA_ALPHA         0.3f     // 0..1 after the median; higher = less smoothing, less lag
#define TOF_STALE_MS          500      // no valid range for this long -> tofFault()
#define SLOT_TABS             4        // tabs on the cardboard disc (events per revolution)
#define SLOT_GLITCH_US        1000UL   // edges closer than this are bounce, not a new tab
#define SLOT_RPM_PERIOD_MS    100      // RPM recomputed at 10 Hz
#define SLOT_TIMEOUT_MS       2000     // no event for this long -> 0 RPM

// ---------- Serial ----------
#define SERIAL_BAUD      115200

// ---------- Analog sensors: conversion + filtering (Track 4) ----------
// Update rate
#define ANALOG_PERIOD_MS      20       // 50 Hz
#define ADC_MAX               1023

// Fixed divider resistors. Nominal 10k; measure yours with a multimeter
// and put the real values here (a 5% resistor is up to ~1 degC of error).
#define FLEX_FIXED_R_OHMS     9790.0f  // measured 2026-11-07
#define THERM_FIXED_R_OHMS    10030.0f // measured 2026-11-07

// Thermistor (Beta model). Verify B on the SparkFun datasheet.
#define THERM_R0_OHMS         10000.0f // resistance at T0
#define THERM_T0_K            298.15f  // 25 degC
#define THERM_BETA            3950.0f
#define THERM_OVERSAMPLE      4        // ADC reads averaged per update
#define THERM_ADC_SHORT       5        // at/below: shorted thermistor -> fault
#define THERM_ADC_OPEN        1018     // at/above: unplugged thermistor -> fault

// Flex sensor (Adafruit 182): ~10k flat, ~20k bent; resistance rises with bend
#define FLEX_ADC_OPEN         5        // at/below: unplugged flex sensor -> fault
#define FLEX_ADC_SHORT        1018     // at/above: shorted flex sensor -> fault
#define FLEX_DEG_MIN          -30.0f   // clamp (bending backwards reads slightly negative)
#define FLEX_DEG_MAX          180.0f   // clamp
#define FLEX_EMA_ALPHA        0.2f     // 0..1, higher = less smoothing, less lag

// ---------- Servo (Track 2) ----------
// Measured with firmware/bringup/servo: strains at 620 us, stalls at 2400 us.
// Full safe travel 640-2380 us = 183 deg (~9.5 us/deg), so 0-180 deg = 640-2351 us.
#define SERVO_MIN_US          640
#define SERVO_MAX_US          2351
#define SERVO_START_DEG       90.0f

// ---------- Stepper (Track 2) ----------
// EasyDriver default 1/8 microstep x 200 full steps = 1600 steps/rev (verify).
#define STEPPER_STEPS_PER_REV 1600
#define STEPPER_STEP_US       1250     // time between steps: 800 steps/s = 0.5 rev/s. Lower = faster

// ---------- Button (Track 2) ----------
#define BUTTON_DEBOUNCE_MS    30

// ---------- Scheduler (Track 2) ----------
#define TELEMETRY_PERIOD_MS   50      // 20 Hz
#define MAPPING_PERIOD_MS     20       // SENSOR-mode mappings, 50 Hz

// ---------- SENSOR-mode mappings (Track 2; agree with team before changing) ----------
#define MAP_TOF_MIN_MM        50       // at/below -> MAP_SERVO_MIN_DEG
#define MAP_TOF_MAX_MM        400      // at/above -> MAP_SERVO_MAX_DEG
#define MAP_SERVO_MIN_DEG     0.0f
#define MAP_SERVO_MAX_DEG     180.0f
#define MAP_FLEX_DEADZONE_DEG 10.0f    // bend below this -> 0 rpm
#define MAP_FLEX_FULL_DEG     90.0f    // bend at/above this -> MAP_DC_MAX_RPM
#define MAP_DC_MAX_RPM        60.0f
#define MAP_SLOT_JOG_DEG      15.0f    // stepper jog per slot event

// ---------- Thermal interlock (Track 2) ----------
#define INTERLOCK_TEMP_C      30.0f    // above this (or thermistor fault) -> DC stops
