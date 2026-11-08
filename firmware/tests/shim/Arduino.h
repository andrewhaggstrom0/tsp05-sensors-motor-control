#pragma once
// Minimal stand-in for the Arduino core so firmware modules compile on a laptop.
// Tests set fake_adc[] and fake_millis to drive the module.
#include <stdint.h>
#include <math.h>

#define A0 14
#define A1 15
#define A2 16
#define INPUT 0
#define INPUT_PULLUP 2

extern int fake_adc[24];           // indexed by pin number
extern uint32_t fake_millis;       // 32-bit, like millis() on the Uno
extern long fake_analog_reads;     // counts analogRead() calls

inline int analogRead(uint8_t pin) { fake_analog_reads++; return fake_adc[pin]; }
inline uint32_t millis() { return fake_millis; }
inline void pinMode(uint8_t, uint8_t) {}

// ---- Added for Track 3 (sensors_digital): micros, interrupts, Serial ----
// Tests set fake_micros alongside fake_millis and call fake_isr() to simulate
// an edge on an interrupt pin.
#define RISING 3
#define F(x) (x)
extern uint32_t fake_micros;       // 32-bit, like micros() on the Uno
extern void (*fake_isr)();         // last function given to attachInterrupt()
inline uint32_t micros() { return fake_micros; }
inline void noInterrupts() {}
inline void interrupts() {}
inline int digitalPinToInterrupt(uint8_t pin) { return pin == 2 ? 0 : (pin == 3 ? 1 : -1); }
inline void attachInterrupt(int, void (*isr)(), int) { fake_isr = isr; }
struct FakeSerial { void println(const char *) {} };
extern FakeSerial Serial;
