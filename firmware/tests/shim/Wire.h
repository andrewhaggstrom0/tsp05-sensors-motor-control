#pragma once
// Minimal stand-in for the Arduino Wire (I2C) library: calls do nothing.
#include <stdint.h>
struct TwoWire {
  void begin() {}
  void setClock(uint32_t) {}
};
extern TwoWire Wire;
