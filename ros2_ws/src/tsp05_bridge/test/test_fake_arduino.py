"""tools/fake_arduino.py must behave like firmware/tsp05 on main.

Plain pytest, no ROS. The fake runs in-process with its own clock (step()), so
every check is deterministic. When the firmware changes, these tests say which
fake behaviour no longer matches.
"""
import csv
import re
import statistics
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
REPO = PKG.parent.parent.parent
TOOLS = REPO / "tools"
CONFIG_H = REPO / "firmware" / "tsp05" / "config.h"
DC_STEP_CSV = REPO / "calibration" / "data" / "dc_step_p90_filtD01.csv"
pytestmark = pytest.mark.skipif(not (TOOLS / "fake_arduino.py").exists() or not CONFIG_H.exists(),
                                reason="tools/ or firmware/ not in this checkout")

sys.path.insert(0, str(PKG))
from tsp05_bridge import protocol as p  # noqa: E402


@pytest.fixture(scope="module")
def fa():
    sys.path.insert(0, str(TOOLS))
    import fake_arduino
    return fake_arduino


def run(uno, seconds):
    for _ in range(int(round(seconds / 0.01))):
        uno.step()


def define(name):
    m = re.search(rf"^\s*#define\s+{name}\s+(-?[\d.]+)", CONFIG_H.read_text(), re.MULTILINE)
    assert m, f"{name} not in config.h"
    return float(m.group(1))


# ---------------------------------------------------------------- constants
@pytest.mark.parametrize("const,define_name,scale", [
    ("SLOT_TABS", "SLOT_TABS", 1), ("SLOT_TIMEOUT_S", "SLOT_TIMEOUT_MS", 1e-3),
    ("STEP_PERIOD_S", "STEPPER_STEP_US", 1e-6), ("STEPS_PER_REV", "STEPPER_STEPS_PER_REV", 1),
    ("MAP_FLEX_DEADZONE_DEG", "MAP_FLEX_DEADZONE_DEG", 1), ("MAP_DC_MAX_RPM", "MAP_DC_MAX_RPM", 1),
    ("MAP_SLOT_JOG_DEG", "MAP_SLOT_JOG_DEG", 1), ("INTERLOCK_TEMP_C", "INTERLOCK_TEMP_C", 1),
    ("DC_COUNTS_PER_REV", "DC_COUNTS_PER_REV", 1), ("DC_POS_TOL_DEG", "DC_POS_TOL_DEG", 1),
    ("GESTURE_SERVO_DEG_S", "GESTURE_SERVO_DEG_S", 1), ("TELEMETRY_PERIOD_S", "TELEMETRY_PERIOD_MS", 1e-3),
])
def test_constants_come_from_config_h(fa, const, define_name, scale):
    assert getattr(fa, const) == pytest.approx(define(define_name) * scale)


# ---------------------------------------------------------------- telemetry
def test_telemetry_matches_protocol_and_firmware_formats(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    run(uno, 3.0)
    line = uno.telemetry()
    t = p.parse_telemetry(line)
    f = line.split(",")
    assert t["ms"] == 3000
    assert re.fullmatch(r"-?\d+\.\d\d", f[9]), "temp_C has 2 decimals"
    assert re.fullmatch(r"-?\d+\.\d\d", f[16]) and re.fullmatch(r"-?\d+\.\d\d", f[17]), "stepper 2 decimals"
    assert re.fullmatch(r"-?\d+\.\d", f[7]), "tof_filt_mm has 1 decimal"


# ---------------------------------------------------------------- commands
def test_motor_commands_ignored_silently_in_sensor_mode(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("M,1")
    assert uno.handle("S,10") is None and uno.servo_deg != 10.0
    target = uno.step_target
    uno.handle("P,90")
    assert uno.step_target == target
    uno.handle("M,0")
    uno.handle("S,10")
    assert uno.servo_deg == 10.0


def test_parser_has_c_semantics(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("S,abc")                    # atof("abc") = 0
    assert uno.servo_deg == 0.0
    uno.handle("S,45.5junk")
    assert uno.servo_deg == 45.5
    uno.handle("S," + "1" * 40)            # longer than the 31-char buffer: dropped
    assert uno.servo_deg == 45.5
    uno.handle("M,7")                      # only 0 and 1 change the mode
    assert uno.mode == 0


def test_mode_switch_stops_dc_and_keeps_servo(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("S,30")
    uno.handle("D,V,40")
    run(uno, 1.0)
    assert uno.dc_rpm > 30
    uno.handle("M,1")
    assert uno.dc_mode == "idle" and uno.dc_pwm == 0
    assert uno.servo_deg == 30.0


# ---------------------------------------------------------------- actuators
def test_servo_reports_the_command(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    assert uno.servo_deg == define("SERVO_START_DEG")
    uno.handle("S,180")
    assert uno.servo_deg == 180.0          # servoDeg() is the commanded angle
    uno.handle("S,200")
    assert uno.servo_deg == 180.0


def test_stepper_speed_and_relative_target(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("P,90")
    uno.handle("P,-45")                    # adds to the target, not the position
    assert uno.step_target == 200
    uno.handle("P,45")
    steps = 400
    expected_s = steps * define("STEPPER_STEP_US") / 1e6
    t0 = uno.t
    while uno.step_pos != uno.step_target:
        uno.step()
    assert uno.t - t0 == pytest.approx(expected_s, abs=0.02)


def test_dc_step_matches_hardware_recording(fa):
    """90 deg move vs Deevi's hardware run with the final gains."""
    rows = list(csv.DictReader(DC_STEP_CSV.open()))
    t = [float(r["t_s"]) for r in rows]
    deg = [float(r["deg"]) for r in rows]
    rpm = [float(r["rpm"]) for r in rows]
    pwm = [float(r["pwm"]) for r in rows]
    i0 = next(i for i, x in enumerate(pwm) if x != 0)
    t0, d0 = t[i0 - 1], deg[i0 - 1]
    hw_inband = next(t[i] - t0 for i in range(i0, len(t)) if abs(deg[i] - d0 - 90) <= 2)
    hw_stopped = next(t[i] - t0 for i in range(i0, len(t))
                      if abs(deg[i] - d0 - 90) <= 2 and abs(rpm[i]) < 2 and pwm[i] == 0)
    hw_peak = max(abs(r) for r in rpm)

    uno = fa.FakeUno("room", 0.0, seed=1)
    run(uno, 0.5)
    start = uno.dc_deg()
    uno.handle("D,P,90")
    t_start, inband, stopped, peak = uno.t, None, None, 0.0
    while uno.t - t_start < 2.0:
        uno.step()
        dt = uno.t - t_start
        peak = max(peak, abs(uno.dc_rpm))
        near = abs(uno.dc_deg() - start - 90) <= 2
        inband = inband if inband is not None or not near else dt
        if stopped is None and near and abs(uno.dc_rpm) < 2 and uno.dc_pwm == 0:
            stopped = dt
    assert uno.dc_arrived
    assert inband == pytest.approx(hw_inband, abs=0.1)
    assert stopped == pytest.approx(hw_stopped, abs=0.15)
    assert peak == pytest.approx(hw_peak, rel=0.15)


def test_dc_velocity_noise_like_hardware(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("D,V,30")
    run(uno, 1.5)
    samples = []
    for _ in range(200):
        uno.step()
        samples.append(uno.dc_rpm)
    assert statistics.mean(samples) == pytest.approx(30.0, abs=1.0)
    assert 0.5 < statistics.pstdev(samples) < 4.0      # 2 deg encoder steps through the EMA
    assert uno.dc_pwm == pytest.approx(67, abs=3)        # measured speed map at 30 rpm


def test_interlock_without_hysteresis_and_dc_stays_stopped(fa):
    uno = fa.FakeUno("cycle", 0.0, seed=4)
    uno.handle("D,V,40")
    tripped = cleared = None
    while uno.t < 40 and cleared is None:
        uno.step()
        if uno.interlock and tripped is None:
            tripped = uno.temp_c
            uno.handle("D,V,40")                 # ignored while the interlock is on
            assert uno.dc_mode == "idle"
        if tripped is not None and not uno.interlock:
            cleared = uno.temp_c
    assert tripped > define("INTERLOCK_TEMP_C") and cleared <= define("INTERLOCK_TEMP_C")
    run(uno, 1.0)
    assert uno.dc_mode == "idle" and uno.dc_rpm == pytest.approx(0, abs=0.5)


# ---------------------------------------------------------------- sensors
def test_tof_freezes_without_a_target(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    run(uno, 1.0)
    assert (uno.tof_raw_mm, uno.tof_filt_mm) == (0, 0.0)     # no valid range since boot
    run(uno, 2.5)                                          # hand passes at 2.0-2.4 s
    frozen = (uno.tof_raw_mm, uno.tof_filt_mm)
    assert 200 < frozen[0] < 300
    run(uno, 1.0)
    assert (uno.tof_raw_mm, uno.tof_filt_mm) == frozen


def test_slot_one_tab_rpm_and_timeout(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    run(uno, 8.0)
    assert uno.slot_rpm == pytest.approx(30.0, abs=0.5)
    assert uno.slot_count == 4                              # 30 rpm, 1 tab, 8 s
    run(uno, 2.0 + define("SLOT_TIMEOUT_MS") / 1000 + 0.2)  # disc stops at 10 s
    assert uno.slot_rpm == 0.0


# ---------------------------------------------------------------- SENSOR mode
def test_sensor_mode_swipe_sweeps_servo(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("M,1")
    run(uno, 2.6)                       # right-to-left swipe at 2.0-2.4 s: sweep toward 180
    assert uno._sweep_dir == +1
    start = uno.servo_deg
    run(uno, 1.0)
    assert uno.servo_deg - start == pytest.approx(define("GESTURE_SERVO_DEG_S"), abs=1.0)
    run(uno, 1.9)                       # t = 5.5 s: 90 -> 180 takes 2 s at 45 deg/s
    assert uno.servo_deg == define("MAP_SERVO_MAX_DEG")   # stops at the end of travel
    assert uno._sweep_dir == 0
    run(uno, 1.1)                       # left-to-right swipe at 6.0-6.4 s: toward 0
    assert uno._sweep_dir == -1


def test_sensor_mode_flex_mapping_and_slot_jog(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    uno.handle("M,1")
    while uno.t < 20:
        uno.step()
        bend = uno.flex_filt_deg
        if uno.dc_mode == "velocity":
            dz, full, top = define("MAP_FLEX_DEADZONE_DEG"), define("MAP_FLEX_FULL_DEG"), define("MAP_DC_MAX_RPM")
            want = 0.0 if bend <= dz else min(1.0, (bend - dz) / (full - dz)) * top
            assert abs(uno.dc_target_rpm - want) <= 0.5 + 1e-6    # re-sent on 0.5 rpm changes
    jogs = uno.step_target * 360.0 / uno_steps(fa) / define("MAP_SLOT_JOG_DEG")
    assert jogs == pytest.approx(uno.slot_count, abs=1)


def uno_steps(fa):
    return fa.STEPS_PER_REV


# ---------------------------------------------------------------- appended fields
def _extras(uno):
    t = p.parse_telemetry(uno.telemetry())
    return {k: t[k] for k in p.TELEMETRY_FIELDS_EXTRA}


def test_line_lengths_new_and_legacy(fa):
    assert len(fa.FakeUno("room", seed=1).telemetry().split(",")) == 1 + len(p.TELEMETRY_FIELDS)
    legacy = fa.FakeUno("room", seed=1, legacy=True)
    assert len(legacy.telemetry().split(",")) == 1 + len(p.TELEMETRY_FIELDS_BASE)
    assert set(p.parse_telemetry(legacy.telemetry())) == set(p.TELEMETRY_FIELDS_BASE)


def test_dc_mode_target_arrived_follow_commands(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    run(uno, 0.2)
    assert _extras(uno)["dc_mode"] == 0 and _extras(uno)["dc_target"] == 0.0
    start = uno.dc_deg()
    uno.handle("D,P,90")
    e = _extras(uno)
    assert (e["dc_mode"], e["dc_arrived"]) == (2, 0)
    assert e["dc_target"] == pytest.approx(start + 90.0)          # same frame as dc_deg
    run(uno, 1.0)
    e = _extras(uno)
    assert e["dc_arrived"] == 1 and abs(uno.dc_deg() - e["dc_target"]) <= define("DC_POS_TOL_DEG")
    uno.handle("D,V,200")                                          # clamped like ctrlSetVelocity
    e = _extras(uno)
    assert (e["dc_mode"], e["dc_target"], e["dc_arrived"]) == (1, define("DC_MAX_RPM"), 0)
    uno.handle("M,1")
    assert _extras(uno)["dc_mode"] == 0                            # setMode() stops the motor ...
    run(uno, 0.05)
    assert _extras(uno)["dc_mode"] == 1                            # ... then the flex mapping drives it
    uno.handle("M,0")
    assert (_extras(uno)["dc_mode"], _extras(uno)["dc_target"]) == (0, 0.0)


def test_interlock_sets_dc_idle(fa):
    uno = fa.FakeUno("cycle", 0.0, seed=4)
    uno.handle("D,P,3600")
    while not uno.interlock:
        uno.step()
    assert (_extras(uno)["dc_mode"], _extras(uno)["dc_target"]) == (0, 0.0)


def test_tof_fault_timing(fa):
    uno = fa.FakeUno("room", 0.0, seed=1)
    run(uno, 1.0)
    assert _extras(uno)["tof_fault"] == 1                          # no valid range since boot
    run(uno, 1.2)                                                  # hand in view at 2.0-2.4 s
    assert _extras(uno)["tof_fault"] == 0
    while uno._hand():
        uno.step()
    last_valid = uno._tof_last_valid
    while not uno.tof_fault():
        uno.step()
    assert uno.t - last_valid == pytest.approx(define("TOF_STALE_MS") / 1000, abs=0.03)
