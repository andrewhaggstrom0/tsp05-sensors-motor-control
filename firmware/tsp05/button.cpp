#include <Arduino.h>
#include "config.h"
#include "button.h"

// Debounce: a raw change is accepted only after it has stayed the same for
// BUTTON_DEBOUNCE_MS. A press is reported once, on the debounced press edge.

static bool     s_raw       = false;  // last raw reading (true = pressed)
static bool     s_down      = false;  // debounced state
static uint32_t s_changedMs = 0;      // when the raw reading last changed
static bool     s_pressed   = false;  // latched press, cleared by buttonTakePress()

static bool readPressed() { return digitalRead(PIN_BUTTON) == LOW; }  // pull-up: pressed = LOW

void buttonInit() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  s_raw = s_down = readPressed();     // a button held at boot is not a press
  s_changedMs = millis();
}

void buttonUpdate() {
  bool raw = readPressed();
  uint32_t now = millis();

  if (raw != s_raw) {                 // still bouncing: restart the timer
    s_raw = raw;
    s_changedMs = now;
    return;
  }
  if (raw != s_down && (uint32_t)(now - s_changedMs) >= BUTTON_DEBOUNCE_MS) {
    s_down = raw;
    if (s_down) s_pressed = true;     // press edge only; holding does nothing more
  }
}

bool buttonTakePress() {
  bool p = s_pressed;
  s_pressed = false;
  return p;
}

bool buttonIsDown() { return s_down; }