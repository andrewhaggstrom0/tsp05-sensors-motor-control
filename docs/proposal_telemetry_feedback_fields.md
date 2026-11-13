# Proposal: DC and TOF feedback fields in the telemetry line (prototype, parked)

Status: **in PR #15 (with the bridge and GUI feedback), tested on the Uno
2026-11-11, not merged.** Firmware files touched here belong to Track 1 (`dc_motor`),
Track 2 (`tsp05.ino`, `docs/protocol.md`) and Track 3 (`tofFault()`); nothing merges
without their agreement. Track 5 (Raoul Bouri) wrote it to show the change end to
end.

## Why
The firmware already computes feedback it does not send: the DC controller's mode
and target (`dc_control.h` `Ctrl`), `dcArrived()` and `tofFault()`. Without them the
GUI cannot draw DC target vs actual, cannot say when a move has arrived, and cannot
tell a frozen TOF (no valid range) from a steady one. On the bench (2026-11-10) a
TOF reading sat at exactly 312 / 332.9 mm for seconds with nothing in view, which
looked like a valid reading.

## The change
Four fields appended to the end of the `T` line (existing positions unchanged):

| Field | Values | Meaning |
|---|---|---|
| `dc_mode` | 0 / 1 / 2 | idle (after `D,X`, a mode switch, the interlock, or at boot) / velocity (`D,V` or the SENSOR flex mapping) / position (`D,P`) |
| `dc_target` | deg or rpm, 1 decimal | position mode: target in deg, same frame as `dc_deg`; velocity mode: target rpm (clamped to `DC_MAX_RPM`); idle: 0.0 |
| `dc_arrived` | 0 / 1 | position mode and within `DC_POS_TOL_DEG` (2 deg) of the target |
| `tof_fault` | 0 / 1 | TOF missing, no valid range since boot, or none for `TOF_STALE_MS` (500 ms) |

`dc_mode` is needed because `dc_target`'s unit depends on the mode; without it a
receiver cannot tell 60 deg from 60 rpm.

Per file:
- `docs/protocol.md`: the four fields; receivers must accept the 19-field line
  (older firmware) and the 23-field line, told apart by field count.
- `firmware/tsp05/dc_motor.{h,cpp}`: `int dcMode()` and `float dcTarget()`, read-only
  getters on the controller state; a `static_assert` ties the mode numbers to the doc.
- `firmware/tsp05/tsp05.ino`: `buildTelemetry()` appends the four values.
- `ros2_ws/src/tsp05_bridge`: `protocol.py` accepts both lengths;
  `bridge_node.py` creates `/tsp05/dc/mode` (Int8), `/tsp05/dc/target` (Float32),
  `/tsp05/dc/arrived` (Bool), `/tsp05/tof/fault` (Bool) on the first line that
  carries them, so with older firmware those topics never appear.
- `tools/fake_arduino.py`: sends the fields; `--legacy-telemetry` sends the old line.
- Tests for all of the above.

## Evidence
- Uno build (`arduino-cli` 1.5.1, `arduino:avr` 1.8.8, `arduino:avr:uno`):

  | | Flash | RAM |
  |---|---|---|
  | `main` at `76a46a5` | 21,766 B (67%) | 1,238 B (60%) |
  | prototype | 22,006 B (68%) | 1,242 B (60%) |

  +240 B flash, +4 B RAM; RAM stays under the 75% budget with ~294 B to spare.
  The Arduino IDE 1.8.19 build of the same sketch gave the same numbers.
- Longest possible line: 145 chars incl. `\n` (every field at its widest), inside
  `g_tx[160]` (159 usable).
- `docker/test.sh`: 81/81 on Jazzy and on Humble (3 runs each).
- `firmware/tests/run_tests.sh`: all suites pass (analog 5217, digital 41,
  dc_control 34, tof_gesture 13).
- Fake Arduino with the new fields: `D,P,90` gives mode 2, target 90.0, arrived 0
  while moving and 1 inside the band; `D,V,-45.3` gives mode 1, target -45.3;
  `tof_fault` goes 1 0.51 s after the last valid range.

## Issues found (read before taking this further)
1. **`dc_arrived` is instantaneous, so it flickers on real hardware.** The controller
   marks "arrived" as soon as the position enters the +-2 deg band and stops driving;
   the motor can coast through and be driven back. Deevi's
   `calibration/data/dc_step_p90_filtD01.csv` (final gains) enters the band at
   0.22 s, overshoots to 94 deg and is out of the band for ~0.3 s before settling at
   92 deg, so `dc_arrived` reads 1 -> 0 -> 1. Either the GUI shows "arrived" only
   after it has been steady (with rpm near 0), or the firmware latches it. Open
   question for Track 1.
2. **A truncated new line can be a valid old line.** A 23-field line cut exactly
   after `interlock` parses as a 19-field line. Its values are that line's own correct
   base values, so only the four new fields of that one line are lost; any other cut
   is still rejected. With the fake's corruption mode this happened for ~1% of
   truncations (50 of 5015). Inherent to a variable-length line; acceptable, but
   worth knowing.
3. **Fake Arduino slot-rpm divide-by-zero (also on `main`).** When a slot edge and
   an rpm update fall on the same instant the fake divided by zero; the firmware's
   `rpmFromSpan()` returns 0 for an empty span. Fixed here and to be fixed on `main`
   separately. Not reachable at the fake's normal 10 ms step.
4. **The memory baseline in the original brief was stale.** `main` is now 60% RAM /
   67% flash (after the TOF swipe PR #10), not 58% / 64%.
5. **Uploading from the Mac through its USB hub fails** (the bootloader syncs, then
   the first 128-byte page write is never acknowledged; two Uno R3 boards, avrdude
   8.0 and 6.3). The prototype was flashed another way on 2026-11-11.

## Hardware results (2026-11-11, prototype firmware on the Uno)
- Raw serial: `# TSP05 starting`, then 23-field lines every 50 ms.
- `D,P,90` from 0 deg: `dc_mode` 2, `dc_target` 90.0; `dc_arrived` 1 at 0.26 s at
  94 deg (overshoot), then 1 -> 0 -> 1, ending at 90.0 deg; `D,X`: `dc_mode` 0,
  `dc_target` 0.0. Matches the prediction from Deevi's p90 recording.
- Through the bridge (PR #15) and Foxglove: 20.0 Hz, 0 malformed after the first
  window (one line cut by the reset when the port opens); `dc/settled` after
  0.75 s at 90.0 deg; every GUI command (DC +-90/+360, 30 rpm, STOP, stepper +45,
  servo 0/90/180, GUI/SENSOR) reported `applied` in 14-90 ms.
- `tof_fault`: 1 with `tof_raw_mm` 0 while the TOF produced no valid ranges, and 0
  once it did after the sensors were powered. The 0 -> 1 timing after a hand leaves
  is still to be measured.
- Not yet checked: the thermistor interlock (pinch above 30 C) and the flex sensor
  (reads -30 deg, Track 4).

## To finish (if the team agrees)
1. Agree field names, order and units with Track 2 (protocol), Track 1 (DC getters,
   and whether to latch `dc_arrived`) and Track 3 (`tof_fault`).
2. Measure the `tof_fault` 0 -> 1 timing with a hand, and the interlock.
3. Review PR #15. Its bridge takes these fields from the line when present and works
   them out otherwise, so firmware and bridge can be updated in either order.

## Draft message to Yashasvi, Harish and Deevi
> Hi all, I prototyped sending four values the firmware already computes, so the GUI
> can show DC target vs actual, "arrived", and a frozen TOF. Branch
> `proto/telemetry-feedback-fields`, notes in `docs/proposal_telemetry_feedback_fields.md`.
>
> Fields appended after `interlock`: `dc_mode` (0 idle, 1 velocity, 2 position),
> `dc_target` (deg in position mode, rpm in velocity mode, 0 idle), `dc_arrived`
> (0/1), `tof_fault` (0/1). Existing positions don't move; the bridge accepts both
> the old 19-field and the new 23-field line, so firmware and bridge can be updated
> in either order.
>
> Cost: +240 B flash, +4 B RAM (68% / 60%); worst-case line 145 of 159 chars.
> Deevi: two read-only getters in dc_motor (`dcMode()`, `dcTarget()`). Yashasvi: four
> `add*` calls in `buildTelemetry()` and the protocol.md rows. Harish: uses your
> existing `tofFault()`.
>
> Questions: (1) are the names/order OK? (2) Deevi, should `dc_arrived` latch? On
> your p90 run the motor overshoots to 94 and leaves the band for ~0.3 s, so the flag
> flickers. (3) Could someone with a Windows/Linux machine flash it once to test?
> My Mac can't upload through its USB hub.
