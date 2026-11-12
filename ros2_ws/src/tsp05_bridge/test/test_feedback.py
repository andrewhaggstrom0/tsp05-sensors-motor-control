"""tsp05_bridge.feedback: DC mode/target/arrival, command status, TOF fault and
rate, worked out from commands and 19-field telemetry. Plain pytest, no ROS.

Two kinds of test: hand-made telemetry for each rule, and a scripted session
against tools/fake_arduino.py (which mirrors the firmware), where the inferred
values are compared with the fake's own internal state at every line.
"""
import re
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
REPO = PKG.parent.parent.parent
CONFIG_H = REPO / "firmware" / "tsp05" / "config.h"
TOOLS = REPO / "tools"
sys.path.insert(0, str(PKG))
from tsp05_bridge import protocol as p  # noqa: E402
from tsp05_bridge.feedback import (DC_IDLE, DC_POSITION, DC_VELOCITY,  # noqa: E402
                                   Feedback, FeedbackConfig)

BASE = "T,{ms},{mode},470,{flex},{flex},{tof},{tof},583,{temp},0,0.0,{servo},{dc_deg},{rpm},{pwm},0.00,{step_t},{il}"


def line(**kw):
    v = dict(ms=0, mode=0, flex="0.0", tof=300, temp="22.00", servo="90.0", dc_deg="0.0",
             rpm="0.0", pwm=0, step_t="0.00", il=0)
    v.update(kw)
    return p.parse_telemetry(BASE.format(**v))


# ---------------------------------------------------------------- config
@pytest.mark.skipif(not CONFIG_H.exists(), reason="firmware/ not in this checkout")
@pytest.mark.parametrize("attr,define", [
    ("dc_pos_tol_deg", "DC_POS_TOL_DEG"), ("dc_moving_rpm", "DC_MOVING_RPM"),
    ("dc_max_rpm", "DC_MAX_RPM"), ("dc_counts_per_rev", "DC_COUNTS_PER_REV"),
    ("map_flex_deadzone_deg", "MAP_FLEX_DEADZONE_DEG"), ("map_flex_full_deg", "MAP_FLEX_FULL_DEG"),
    ("map_dc_max_rpm", "MAP_DC_MAX_RPM"), ("flex_adc_open", "FLEX_ADC_OPEN"),
    ("flex_adc_short", "FLEX_ADC_SHORT"), ("stepper_steps_per_rev", "STEPPER_STEPS_PER_REV"),
    ("tof_stale_ms", "TOF_STALE_MS"), ("interlock_temp_c", "INTERLOCK_TEMP_C"),
])
def test_defaults_match_config_h(attr, define):
    m = re.search(rf"^\s*#define\s+{define}\s+(-?[\d.]+)", CONFIG_H.read_text(), re.MULTILINE)
    assert m, define
    assert getattr(FeedbackConfig(), attr) == pytest.approx(float(m.group(1)))


# ---------------------------------------------------------------- DC rules
def test_position_move_target_arrived_settled():
    fb = Feedback()
    fb.on_telemetry(line(dc_deg="10.0"), 0.0)
    fb.on_command("D,P,91.0", 0.01)           # 91 deg = 45.5 counts -> 46 counts = 92 deg
    assert (fb.dc_mode, fb.dc_target) == (DC_POSITION, pytest.approx(102.0))
    fb.on_telemetry(line(dc_deg="50.0", rpm="80.0", pwm=200), 0.05)
    assert not fb.dc_arrived
    fb.on_telemetry(line(dc_deg="101.0", rpm="40.0", pwm=0), 0.10)
    assert fb.dc_arrived and not fb.dc_settled          # in the band but still moving
    fb.on_telemetry(line(dc_deg="106.0", rpm="10.0", pwm=0), 0.15)
    assert not fb.dc_arrived                            # coasted through the band
    for k in range(5):
        fb.on_telemetry(line(dc_deg="103.0", rpm="0.0", pwm=0), 0.2 + 0.05 * k)
    assert fb.dc_arrived and fb.dc_settled


def test_velocity_is_clamped_and_stop_goes_idle():
    fb = Feedback()
    fb.on_telemetry(line(), 0.0)
    fb.on_command("D,V,200.0", 0.01)
    assert (fb.dc_mode, fb.dc_target) == (DC_VELOCITY, 140.0)
    fb.on_command("D,X", 0.02)
    assert (fb.dc_mode, fb.dc_target) == (DC_IDLE, 0.0)


def test_mode_change_and_interlock_stop_the_motor():
    fb = Feedback()
    fb.on_telemetry(line(), 0.0)
    fb.on_command("D,V,30.0", 0.01)
    fb.on_telemetry(line(il=1, temp="31.00"), 0.05)     # interlock trips
    assert fb.dc_mode == DC_IDLE
    fb.on_command("D,V,30.0", 0.06)                     # firmware ignores it during the interlock
    assert fb.dc_mode == DC_IDLE and fb.take_status()[-1] == "D,V,30.0: ignored (interlock)"
    fb.on_telemetry(line(), 0.10)                       # interlock clears: stays stopped
    assert fb.dc_mode == DC_IDLE
    fb.on_command("D,P,90.0", 0.11)
    fb.on_telemetry(line(mode=1), 0.15)                 # button -> SENSOR: setMode() stops it,
    assert fb.dc_mode == DC_VELOCITY and fb.dc_target == 0.0   # then the flex mapping (flat: 0 rpm)


def test_sensor_mode_flex_mapping_and_interlock_quirk():
    fb = Feedback()
    fb.on_telemetry(line(mode=1, flex="50.0"), 0.0)     # (50-10)/(90-10) * 60 = 30 rpm
    assert (fb.dc_mode, fb.dc_target) == (DC_VELOCITY, pytest.approx(30.0))
    fb.on_telemetry(line(mode=1, flex="50.2"), 0.05)    # change < 0.5 rpm: not re-sent
    assert fb.dc_target == pytest.approx(30.0)
    fb.on_telemetry(line(mode=1, flex="70.0", il=1), 0.10)   # interlock: stopped, rpm noted
    assert fb.dc_mode == DC_IDLE
    fb.on_telemetry(line(mode=1, flex="70.0"), 0.15)    # cleared, flex unchanged: stays stopped
    assert fb.dc_mode == DC_IDLE
    fb.on_telemetry(line(mode=1, flex="80.0"), 0.20)    # flex changes: driven again
    assert (fb.dc_mode, fb.dc_target) == (DC_VELOCITY, pytest.approx(52.5))
    fb.on_telemetry(line(mode=1, flex="80.0") | {"flex_raw": 1020.0}, 0.25)   # flex fault -> 0 rpm
    assert fb.dc_target == 0.0


# ---------------------------------------------------------------- command status
def test_command_status_applied_ignored_and_timeout():
    fb = Feedback()
    fb.on_telemetry(line(), 0.0)
    fb.on_command("S,45.0", 1.000)
    fb.on_telemetry(line(servo="45.0"), 1.031)
    assert fb.take_status() == ["S,45.0: applied in 31 ms"]
    fb.on_command("P,10.0", 2.0)                        # 44 steps = 9.90 deg
    fb.on_command("P,10.0", 2.01)                       # chained: 19.80 deg
    fb.on_telemetry(line(step_t="9.90"), 2.05)
    fb.on_telemetry(line(step_t="19.80"), 2.10)
    assert fb.take_status() == ["P,10.0: applied in 50 ms", "P,10.0: applied in 90 ms"]
    fb.on_command("M,1", 3.0)
    fb.on_telemetry(line(mode=1), 3.05)
    fb.on_command("S,10.0", 3.06)
    assert fb.take_status() == ["M,1: applied in 50 ms", "S,10.0: ignored (SENSOR mode)"]
    fb.on_telemetry(line(), 4.0)
    fb.on_command("S,120.0", 4.01)
    fb.tick(5.2)
    assert fb.take_status() == ["S,120.0: no effect seen within 1 s"]


# ---------------------------------------------------------------- TOF and rate
def test_tof_fault_from_frozen_raw():
    fb = Feedback()
    fb.on_telemetry(line(tof=0), 0.0)
    assert fb.tof_fault                                  # no valid range since boot
    fb.on_telemetry(line(tof=301), 0.05)
    assert not fb.tof_fault
    fb.on_telemetry(line(tof=301), 0.50)
    assert not fb.tof_fault                              # unchanged for 0.45 s
    fb.on_telemetry(line(tof=301), 0.56)
    assert fb.tof_fault                                  # unchanged for 0.51 s
    fb.on_telemetry(line(tof=299), 0.60)
    assert not fb.tof_fault


def test_rate_and_line_age():
    fb = Feedback()
    for k in range(60):
        fb.on_telemetry(line(), k * 0.05)
    assert fb.rate_hz(2.95) == pytest.approx(20.0, abs=0.6)
    assert fb.line_age_s(3.20) == pytest.approx(0.25)


# ---------------------------------------------------------------- against the fake Arduino
@pytest.fixture
def fa():
    if not (TOOLS / "fake_arduino.py").exists():
        pytest.skip("tools/fake_arduino.py not in this checkout")
    sys.path.insert(0, str(TOOLS))
    import fake_arduino
    return fake_arduino


class Session:
    """The fake Arduino and Feedback side by side, one telemetry line every 50 ms."""

    MODES = {"idle": DC_IDLE, "velocity": DC_VELOCITY, "position": DC_POSITION}

    def __init__(self, fa, temp="room", seed=1):
        # legacy: the 19-field line, so the feedback is worked out, not read from the line
        self.uno = fa.FakeUno(temp, 0.0, seed=seed, legacy=True)
        self.fb = Feedback()
        self.lines = 0
        self.dc_mismatch = []     # (t, inferred, truth)
        self.tof_mismatch = []

    def run(self, seconds):
        for _ in range(int(round(seconds / 0.05))):
            for _ in range(5):
                self.uno.step()
            t = p.parse_telemetry(self.uno.telemetry())
            self.fb.on_telemetry(t, self.uno.t)
            self.lines += 1
            u, fb = self.uno, self.fb
            truth_mode = self.MODES[u.dc_mode]
            truth_target = (u.dc_target_counts * 360.0 / 180.0 if u.dc_mode == "position"
                            else u.dc_target_rpm if u.dc_mode == "velocity" else 0.0)
            if fb.dc_mode != truth_mode or abs(fb.dc_target - truth_target) > 1.0:
                self.dc_mismatch.append((round(u.t, 2), fb.dc_mode, fb.dc_target, truth_mode, truth_target))
            if fb.tof_fault != u.tof_fault():
                self.tof_mismatch.append(round(u.t, 2))

    def cmd(self, c):
        self.uno.handle(c)
        self.fb.on_command(c, self.uno.t)


def test_matches_fake_arduino_through_a_session(fa):
    s = Session(fa)
    s.run(0.5)
    for c, wait in [("S,30.0", 0.3), ("D,P,90.0", 1.2), ("D,P,-45.0", 1.0), ("D,V,-60.0", 1.0),
                    ("D,X", 0.5), ("P,45.0", 0.6), ("M,1", 4.0), ("S,170.0", 0.3), ("M,0", 0.5),
                    ("D,P,360.0", 2.0), ("D,V,30.0", 1.0), ("D,X", 0.5)]:
        s.cmd(c)
        s.run(wait)
    # The bridge sees the Uno 50 ms at a time, so a transition can show one line late.
    assert len(s.dc_mismatch) <= 6, s.dc_mismatch
    status = s.fb.take_status()
    assert "S,170.0: ignored (SENSOR mode)" in status
    for c in ("S,30.0", "D,P,90.0", "D,V,-60.0", "D,X", "P,45.0", "M,1", "M,0", "D,P,360.0"):
        assert any(x.startswith(f"{c}: applied in ") for x in status), (c, status)
    assert s.fb.dc_mode == DC_IDLE


def test_position_arrival_matches_fake(fa):
    s = Session(fa)
    s.run(0.5)
    s.cmd("D,P,90.0")
    arrived_lines, settled_at = 0, None
    for _ in range(40):
        s.run(0.05)
        u = s.uno
        assert s.fb.dc_arrived == (u.dc_mode == "position"
                                   and abs(u.dc_deg() - s.fb.dc_target) <= 2.0)
        arrived_lines += s.fb.dc_arrived
        if s.fb.dc_settled and settled_at is None:
            settled_at = u.t
    assert arrived_lines > 0 and settled_at is not None
    assert abs(s.uno.dc_deg() - s.fb.dc_target) <= 2.0 and abs(s.uno.dc_rpm) < 2.0


def test_interlock_and_sensor_mode_match_fake(fa):
    s = Session(fa, temp="cycle", seed=4)     # thermistor pinched above 30 C at ~27-33 s
    s.run(1.0)
    s.cmd("D,V,40.0")
    s.run(5.0)
    s.cmd("M,1")
    s.run(30.0)                               # swipes, slot jogs, flex mapping, interlock
    assert len(s.dc_mismatch) <= 0.03 * s.lines, s.dc_mismatch[:10]


def test_tof_fault_matches_fake(fa):
    s = Session(fa)
    s.run(25.0)                               # hand passes at 2.0-2.4 s of every 10 s
    assert s.fb.tof_fault is True
    # Disagreements only at transitions (the bridge needs one line to see a change).
    assert len(s.tof_mismatch) <= 2 * 3 * 2, s.tof_mismatch


def test_firmware_fields_are_used_when_present():
    fb = Feedback()
    t = line(dc_deg="10.0") | {"dc_mode": 2.0, "dc_target": 100.0, "dc_arrived": 0.0, "tof_fault": 0.0}
    fb.on_telemetry(t, 0.0)
    assert (fb.source, fb.dc_mode, fb.dc_target, fb.dc_arrived, fb.tof_fault) == ("firmware", 2, 100.0, False, False)
    for k in range(5):
        fb.on_telemetry(t | {"dc_deg": 99.0, "dc_arrived": 1.0}, 0.05 * (k + 1))
    assert fb.dc_arrived and fb.dc_settled                # settled is still worked out here
    fb.on_telemetry(line(tof=0), 1.0)                      # a 19-field line: back to inference
    assert fb.source == "inferred" and fb.tof_fault


def test_firmware_fields_agree_with_inference_on_the_fake(fa):
    """The fake sends the fields like the new firmware; inference from the same
    session (without them) must agree with them."""
    uno = fa.FakeUno("cycle", 0.0, seed=4)
    fw, inf = Feedback(), Feedback()
    mismatches = 0
    script = {20: "D,P,90.0", 60: "D,V,-60.0", 90: "D,X", 110: "M,1", 500: "M,0"}
    for i in range(800):                                   # 40 s
        if i in script:
            uno.handle(script[i]); fw.on_command(script[i], uno.t); inf.on_command(script[i], uno.t)
        for _ in range(5):
            uno.step()
        full = p.parse_telemetry(uno.telemetry())
        base = {k: v for k, v in full.items() if k in p.TELEMETRY_FIELDS_BASE}
        fw.on_telemetry(full, uno.t); inf.on_telemetry(base, uno.t)
        mismatches += (fw.dc_mode != inf.dc_mode or abs(fw.dc_target - inf.dc_target) > 1.0)
    assert fw.source == "firmware" and inf.source == "inferred"
    assert mismatches <= 0.02 * 800, mismatches


def test_commands_right_after_a_mode_change_use_the_new_mode():
    """The panel's STOP in SENSOR mode sends M,0 then D,X: the Uno switches mode
    first, so D,X is obeyed even though telemetry still says SENSOR."""
    fb = Feedback()
    fb.on_telemetry(line(mode=1), 0.0)
    fb.on_command("M,0", 0.01)
    fb.on_command("D,X", 0.011)
    assert fb.take_status() == []                       # not "ignored (SENSOR mode)"
    fb.on_telemetry(line(mode=0), 0.05)
    assert fb.take_status() == ["M,0: applied in 40 ms", "D,X: applied in 39 ms"]
    # and the other way: M,1 then a motor command is ignored by the Uno
    fb.on_command("M,1", 1.0)
    fb.on_command("S,10.0", 1.001)
    assert fb.take_status() == ["S,10.0: ignored (SENSOR mode)"]
    # a sent mode that never shows up stops counting after the command timeout
    fb.on_telemetry(line(mode=0), 1.05)
    fb.on_command("S,20.0", 2.5)
    fb.on_telemetry(line(mode=0, servo="20.0"), 2.55)
    assert "S,20.0: applied in 50 ms" in fb.take_status()


def test_dc_command_right_after_a_mode_change_survives_the_switch_line():
    """M,0 then D,P before telemetry shows GUI mode: the Uno stops the DC motor on
    M,0 and then starts the move, so the line showing the switch must not reset it."""
    fb = Feedback()
    fb.on_telemetry(line(mode=1, flex="50.0"), 0.0)     # SENSOR mode: flex drives the DC
    assert fb.dc_mode == DC_VELOCITY
    fb.on_command("M,0", 0.01)
    assert fb.dc_mode == DC_IDLE                        # setMode() stops it on M,0
    fb.on_command("D,P,90.0", 0.011)
    fb.on_telemetry(line(mode=0, pwm=150), 0.05)        # first line in GUI mode
    assert (fb.dc_mode, fb.dc_target) == (DC_POSITION, pytest.approx(90.0))
    # a switch the bridge didn't send (the button) still resets on telemetry
    fb.on_telemetry(line(mode=1, flex="0.0"), 0.10)
    assert (fb.dc_mode, fb.dc_target) == (DC_VELOCITY, 0.0)
