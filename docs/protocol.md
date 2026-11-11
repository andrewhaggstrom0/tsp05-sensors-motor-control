# Serial Protocol (115200 baud, ASCII, newline terminated)

## Telemetry (Uno to laptop, ~20 Hz)

    T,ms,mode,flex_raw,flex_deg,flex_filt_deg,tof_raw_mm,tof_filt_mm,th_raw,temp_C,slot_count,slot_rpm,servo_deg,dc_deg,dc_rpm,dc_pwm,step_deg,step_target_deg,interlock,dc_mode,dc_target,dc_arrived,tof_fault

The last four fields (`dc_mode` to `tof_fault`) were added after the first
release. Older firmware sends the first 19 fields only, so receivers must accept
both lengths (19 or 23 fields) and tell them apart by the field count. New fields
are only ever appended at the end; existing positions never move.

| Field | Units | Notes |
|---|---|---|
| ms | ms | millis() |
| mode | 0/1 | 0 = GUI, 1 = SENSOR |
| flex_raw | ADC counts | 0 to 1023, falls as the sensor bends |
| flex_deg | deg | Unfiltered bend angle, 1 decimal |
| flex_filt_deg | deg | Filtered bend angle (EMA), 1 decimal |
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
| dc_mode | 0/1/2 | DC controller mode: 0 = idle (stopped: after `D,X`, a mode switch, the interlock, or at boot), 1 = velocity (`D,V`, or the flex mapping in SENSOR mode), 2 = position (`D,P`) |
| dc_target | deg or RPM | Depends on `dc_mode`: 2 = target position in deg, same frame as `dc_deg`; 1 = target speed in RPM (signed, clamped to `DC_MAX_RPM`); 0 = 0.0. 1 decimal |
| dc_arrived | 0/1 | 1 only when `dc_mode` = 2 and `dc_deg` is within `DC_POS_TOL_DEG` (2 deg) of `dc_target`. The controller stops driving there, so the motor may still coast briefly |
| tof_fault | 0/1 | 1 = TOF missing, no valid range since boot, or none for `TOF_STALE_MS` (500 ms). While 1, `tof_raw_mm` and `tof_filt_mm` hold their last value |

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
