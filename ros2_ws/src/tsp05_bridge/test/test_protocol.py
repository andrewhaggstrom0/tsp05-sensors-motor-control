"""Unit tests for tsp05_bridge.protocol. Plain pytest, no ROS needed:

    python3 -m pytest ros2_ws/src/tsp05_bridge/test
"""
import math
import random
import re
import sys
from pathlib import Path

import pytest

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
from tsp05_bridge import protocol as p  # noqa: E402

REPO = PKG.parent.parent.parent          # ros2_ws/src/tsp05_bridge -> repo root
PROTOCOL_MD = REPO / "docs" / "protocol.md"
TOOLS = REPO / "tools"

GOOD = "T,2004,0,425,22.0,19.7,723,731,523,23.9,4,30.0,90.0,0.0,0.0,0,0.000,0.000,0"


# ---------------------------------------------------------------- contract
@pytest.mark.skipif(not PROTOCOL_MD.exists(), reason="docs/protocol.md not in this checkout")
def test_fields_match_protocol_md():
    """The field order must match the contract document exactly."""
    text = PROTOCOL_MD.read_text()
    m = re.search(r"^\s*T,([a-zA-Z_,]+)\s*$", text, re.MULTILINE)
    assert m, "telemetry line not found in docs/protocol.md"
    assert tuple(m.group(1).split(",")) == p.TELEMETRY_FIELDS


# ---------------------------------------------------------------- telemetry
def test_parse_good_line():
    t = p.parse_telemetry(GOOD)
    assert list(t) == list(p.TELEMETRY_FIELDS_BASE)        # older firmware: base fields only
    assert t["ms"] == 2004 and t["mode"] == 0 and t["flex_raw"] == 425
    assert t["temp_C"] == pytest.approx(23.9)
    assert t["servo_deg"] == 90.0 and t["interlock"] == 0


@pytest.mark.parametrize("ending", ["\n", "\r\n", "  \r\n"])
def test_parse_line_endings(ending):
    assert p.parse_telemetry(GOOD + ending)["ms"] == 2004


def test_negative_values():
    line = GOOD.replace(",0.0,0.0,0,0.000", ",-12.5,-50.0,-130,-9.900")
    t = p.parse_telemetry(line)
    assert t["dc_deg"] == -12.5 and t["dc_rpm"] == -50.0 and t["dc_pwm"] == -130
    assert t["step_deg"] == pytest.approx(-9.9)


@pytest.mark.parametrize("bad", [
    GOOD[:30],                              # cut short (e.g. Uno reset mid-line)
    GOOD + ",99",                           # extra field
    GOOD.replace(",425,", ",4x5,"),         # non-numeric
    GOOD.replace(",425,", ",,"),            # empty field
    GOOD.replace(",23.9,", ",nan,"),        # not finite
    GOOD.replace(",23.9,", ",inf,"),
    GOOD.replace("T,2004,0,", "T,2004,2,"),  # mode not 0/1
    GOOD[:-1] + "7",                        # interlock not 0/1
    "T,",
    "T",
])
def test_malformed_raises_parse_error(bad):
    with pytest.raises(p.ParseError):
        p.parse_telemetry(bad)


@pytest.mark.parametrize("line", ["# tsp05 fake arduino", "TSP05 firmware stub", "", "S,45"])
def test_non_telemetry_lines(line):
    assert not p.is_telemetry(line)


def test_fuzz_only_parse_error():
    """Random garbage never raises anything but ParseError."""
    rng = random.Random(0)
    alphabet = "T,0123456789.-+eEnaifx# \r"
    for _ in range(5000):
        line = "T," + "".join(rng.choice(alphabet) for _ in range(rng.randrange(0, 120)))
        try:
            t = p.parse_telemetry(line)
        except p.ParseError:
            continue
        assert all(math.isfinite(v) for v in t.values())


# ---------------------------------------------------------------- commands
def test_command_strings():
    assert p.cmd_mode(0) == "M,0" and p.cmd_mode(1) == "M,1"
    assert p.cmd_servo_deg(45) == "S,45.0"
    assert p.cmd_dc_move_deg(-90) == "D,P,-90.0"
    assert p.cmd_dc_rpm(30.25) == "D,V,30.2"
    assert p.cmd_dc_stop() == "D,X"
    assert p.cmd_stepper_deg(10) == "P,10.0"
    assert p.encode("D,X") == b"D,X\n"


@pytest.mark.parametrize("fn", [p.cmd_servo_deg, p.cmd_dc_move_deg, p.cmd_dc_rpm, p.cmd_stepper_deg])
@pytest.mark.parametrize("value", [float("nan"), float("inf"), float("-inf")])
def test_commands_reject_non_finite(fn, value):
    with pytest.raises(ValueError):
        fn(value)


def test_mode_rejects_other_values():
    with pytest.raises(ValueError):
        p.cmd_mode(2)


# ---------------------------------------------------------------- fake Arduino
@pytest.fixture
def fake():
    if not (TOOLS / "fake_arduino.py").exists():
        pytest.skip("tools/fake_arduino.py not in this checkout")
    sys.path.insert(0, str(TOOLS))
    import fake_arduino
    return fake_arduino


def test_fake_arduino_lines_parse(fake):
    uno = fake.FakeUno("cycle", 0.0, seed=1)
    for _ in range(200):
        uno.step(0.05)
        p.parse_telemetry(uno.telemetry())


def test_fake_arduino_corrupt_lines_rejected(fake):
    uno = fake.FakeUno("room", 1.0, seed=2)      # corrupt every line
    rejected = 0
    for _ in range(400):
        uno.step(0.05)
        line = uno.telemetry()
        if not p.is_telemetry(line):
            rejected += 1
            continue
        with pytest.raises(p.ParseError):
            p.parse_telemetry(line)
        rejected += 1
    assert rejected == 400


def test_fake_arduino_accepts_every_command(fake):
    uno = fake.FakeUno("room", 0.0, seed=3)
    for cmd in (p.cmd_mode(0), p.cmd_servo_deg(45), p.cmd_dc_move_deg(90),
                p.cmd_dc_rpm(-30), p.cmd_dc_stop(), p.cmd_stepper_deg(-10), p.cmd_mode(1)):
        assert uno.handle(cmd) is None, cmd
