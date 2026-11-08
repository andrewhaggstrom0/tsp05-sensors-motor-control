#pragma once
// Track 2: debounced pushbutton on A2 (INPUT_PULLUP, pressed = LOW).
// Call buttonInit() once from setup() and buttonUpdate() every loop().

void  buttonInit();
void  buttonUpdate();

bool  buttonTakePress();   // true once per debounced press, then clears
bool  buttonIsDown();      // current debounced state
