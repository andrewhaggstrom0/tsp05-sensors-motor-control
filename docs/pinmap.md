# Pin Map

| Pin | Function | Notes |
|---|---|---|
| D2 | Encoder A | INT0, 4.7k pull-up to 5V |
| D3 | Slot sensor | INT1, 10k pull-up to 5V |
| D4 | Encoder B | Read in encoder ISR, 4.7k pull-up to 5V |
| D5 | L298 enable (PWM) | Timer0 PWM |
| D7 | L298 IN1 | Direction |
| D8 | L298 IN2 | Direction |
| D9 | Servo signal | Servo lib disables PWM on D9/D10 |
| D10 | EasyDriver STEP | |
| D11 | EasyDriver DIR | |
| D12 | EasyDriver ENABLE | Active LOW |
| A0 | FSR divider | 10k to GND |
| A1 | Thermistor divider | 10k to 5V |
| A2 | Pushbutton | INPUT_PULLUP |
| A4 | VL53L1X SDA | |
| A5 | VL53L1X SCL | |

## Power
- DC motor: 12 V bench supply into L298
- Stepper: 12 V bench supply, current limited, into EasyDriver (set pot before connecting motor)
- Servo: separate 5 to 6 V supply
- All supply grounds tied to Arduino GND
- Never wire or unwire while powered

## Stepper wiring (corrected colors)
A = red, C = blue, B = black, D = green
