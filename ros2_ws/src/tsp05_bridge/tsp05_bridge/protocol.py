"""TSP05 serial protocol: parse telemetry lines and format commands.

This module is the laptop side of docs/protocol.md, the contract with the
firmware. It has no ROS dependency, so it can be unit-tested (and reused by
tools) on any machine with plain Python.

Telemetry (Uno -> laptop, ~20 Hz, one line):
    T,ms,mode,flex_raw,flex_deg,flex_filt_deg,tof_raw_mm,tof_filt_mm,th_raw,
    temp_C,slot_count,slot_rpm,servo_deg,dc_deg,dc_rpm,dc_pwm,step_deg,
    step_target_deg,interlock

Any other line is not telemetry: lines starting with '#' are firmware
comments (banners, "ignored" replies) and everything else is noise, e.g.
the half line seen right after the Uno resets.
"""
from __future__ import annotations

import math
from typing import Dict

# Field names after the leading "T", in the exact order the firmware sends them.
# Changing this needs a PR that updates docs/protocol.md first.
TELEMETRY_FIELDS_BASE = (
    "ms", "mode",
    "flex_raw", "flex_deg", "flex_filt_deg",
    "tof_raw_mm", "tof_filt_mm",
    "th_raw", "temp_C",
    "slot_count", "slot_rpm",
    "servo_deg", "dc_deg", "dc_rpm", "dc_pwm",
    "step_deg", "step_target_deg",
    "interlock",
)
# Appended later (docs/protocol.md). Older firmware stops after "interlock", so a
# line may carry the base fields only or the base fields plus all of these.
TELEMETRY_FIELDS_EXTRA = ("dc_mode", "dc_target", "dc_arrived", "tof_fault")
TELEMETRY_FIELDS = TELEMETRY_FIELDS_BASE + TELEMETRY_FIELDS_EXTRA
ACCEPTED_LENGTHS = (len(TELEMETRY_FIELDS_BASE), len(TELEMETRY_FIELDS))
TELEMETRY_PREFIX = "T"
FLAG_FIELDS = ("mode", "interlock", "dc_arrived", "tof_fault")   # exactly 0 or 1 when present
DC_MODES = (0.0, 1.0, 2.0)                                       # idle, velocity, position

MODE_GUI, MODE_SENSOR = 0, 1

Telemetry = Dict[str, float]


class ParseError(ValueError):
    """A line that looks like telemetry but is malformed."""


def is_telemetry(line: str) -> bool:
    return line.startswith(TELEMETRY_PREFIX + ",")


def parse_telemetry(line: str) -> Telemetry:
    """Parse one telemetry line into {field: value}.

    Accepts the base line (older firmware) or the base line plus the appended
    fields; the result has only the fields that were sent. Accepts trailing
    whitespace / CRLF. Raises ParseError (never anything else) for any other
    field count, a non-numeric or non-finite value, a 0/1 flag that is neither,
    or a dc_mode outside 0..2.
    """
    parts = line.strip().split(",")
    if parts[0] != TELEMETRY_PREFIX:
        raise ParseError(f"not a telemetry line: {line[:40]!r}")
    values = parts[1:]
    if len(values) not in ACCEPTED_LENGTHS:
        raise ParseError(f"expected {' or '.join(map(str, ACCEPTED_LENGTHS))} fields, got {len(values)}")
    out: Telemetry = {}
    for name, text in zip(TELEMETRY_FIELDS, values):
        try:
            value = float(text)
        except ValueError:
            raise ParseError(f"{name}: not a number: {text[:20]!r}") from None
        if not math.isfinite(value):
            raise ParseError(f"{name}: not finite: {text!r}")
        out[name] = value
    for name in FLAG_FIELDS:
        if name in out and out[name] not in (0.0, 1.0):
            raise ParseError(f"{name}: expected 0 or 1, got {out[name]:g}")
    if "dc_mode" in out and out["dc_mode"] not in DC_MODES:
        raise ParseError(f"dc_mode: expected 0, 1 or 2, got {out['dc_mode']:g}")
    return out


# ---------------------------------------------------------------- commands
def _num(value: float, what: str) -> str:
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{what} must be a finite number, got {value}")
    # Short, fixed format: the Uno parses it with atof() into a small buffer.
    return f"{value:.1f}"


def cmd_mode(mode: int) -> str:
    if mode not in (MODE_GUI, MODE_SENSOR):
        raise ValueError(f"mode must be 0 (GUI) or 1 (SENSOR), got {mode}")
    return f"M,{mode}"


def cmd_servo_deg(deg: float) -> str:
    """Servo absolute angle. The firmware clamps to its calibrated range."""
    return f"S,{_num(deg, 'servo angle')}"


def cmd_dc_move_deg(deg: float) -> str:
    """DC motor: move relative to the current position."""
    return f"D,P,{_num(deg, 'DC move')}"


def cmd_dc_rpm(rpm: float) -> str:
    """DC motor: hold a signed velocity."""
    return f"D,V,{_num(rpm, 'DC rpm')}"


def cmd_dc_stop() -> str:
    return "D,X"


def cmd_stepper_deg(deg: float) -> str:
    """Stepper: move relative (signed)."""
    return f"P,{_num(deg, 'stepper move')}"


def encode(command: str) -> bytes:
    """Bytes to write to the serial port for one command."""
    return (command + "\n").encode("ascii")
