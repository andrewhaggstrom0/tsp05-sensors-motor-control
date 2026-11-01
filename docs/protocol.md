# Serial Protocol (115200 baud, ASCII, newline terminated)

## Telemetry (Uno to laptop, ~20 Hz)

    T,ms,mode,fsr_raw,fsr_N,fsr_filt,tof_raw_mm,tof_filt_mm,th_raw,temp_C,slot_count,slot_rpm,servo_deg,dc_deg,dc_rpm,dc_pwm,step_deg,step_target_deg,interlock

| Field | Units | Notes |
|---|---|---|
| ms | ms | millis() |
| mode | 0/1 | 0 = GUI, 1 = SENSOR |
| fsr_raw | ADC counts | 0 to 1023 |
| fsr_N | N | Unfiltered force |
| fsr_filt | N | Filtered force |
| tof_raw_mm | mm | |
| tof_filt_mm | mm | |
| th_raw | ADC counts | |
| temp_C | degC | |
| slot_count | events | |
| slot_rpm | RPM | |
| servo_deg | deg | Commanded |
| dc_deg | deg | Encoder position |
| dc_rpm | RPM | Measured |
| dc_pwm | -255 to 255 | |
| step_deg | deg | Actual position (rounded to steps) |
| step_target_deg | deg | |
| interlock | 0/1 | 1 = thermal interlock active |

## Commands (laptop to Uno)

| Command | Meaning |
|---|---|
| `M,0` / `M,1` | Mode: GUI / SENSOR |
| `S,<deg>` | Servo absolute angle |
| `D,P,<deg>` | DC move relative to current position |
| `D,V,<rpm>` | DC velocity (signed) |
| `D,X` | DC stop |
| `P,<deg>` | Stepper relative move (signed) |

Motor commands are ignored in SENSOR mode. Mode is echoed in every telemetry line.
