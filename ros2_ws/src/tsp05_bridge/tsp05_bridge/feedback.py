"""Feedback the firmware does not send, worked out from the commands the bridge
sends and the 19-field telemetry it receives.

No ROS imports: the bridge node feeds it, the tests drive it directly.

What it reports, and how:
  dc_mode    0 idle, 1 velocity, 2 position: the DC controller's mode, followed
             from the commands (D,P / D,V / D,X) and the events that stop the
             motor in the firmware (a GUI/SENSOR mode change, the interlock),
             plus the SENSOR-mode flex mapping, which sets velocity mode.
  dc_target  position mode: target in deg, in the dc_deg frame (dc_deg when the
             command was sent + the requested move, in whole encoder counts like
             dcMoveRelative()). Exact when the motor was still; while it turns,
             off by up to one telemetry period of travel. Velocity mode: the
             commanded rpm (clamped like the firmware), or the flex-mapped rpm in
             SENSOR mode. Idle: 0.
  dc_arrived position mode and |dc_deg - dc_target| <= DC_POS_TOL_DEG (what the
             firmware's dcArrived() reports). It can flicker: the motor coasts
             through the band and is driven back (the hardware 90 deg step reaches
             94 deg).
  dc_settled dc_arrived and slower than DC_MOVING_RPM for settle_lines lines in a
             row: the state a GUI should call "arrived".
  tof_fault  tof_raw_mm still 0 (no valid range since boot) or unchanged for
             TOF_STALE_MS. Valid ranges jitter by ~2 mm, so an exactly constant
             value means the firmware is holding its last range.
  status     one line per command: applied (with the time until telemetry showed
             it), ignored (SENSOR mode / interlock, which the firmware does
             silently), or no effect seen within cmd_timeout_s.
  rate_hz    telemetry lines per second over rate_window_s.

Newer firmware sends dc_mode, dc_target, dc_arrived and tof_fault at the end of
the line (docs/protocol.md). When a line carries them they are used as sent and
only dc_settled, the command status and the rate are worked out here; with older
firmware (19-field line) everything above is worked out.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, List, Optional

DC_IDLE, DC_VELOCITY, DC_POSITION = 0, 1, 2
MODE_GUI, MODE_SENSOR = 0, 1


@dataclass
class FeedbackConfig:
    """Defaults mirror firmware/tsp05/config.h (test_feedback.py checks them)."""
    dc_pos_tol_deg: float = 2.0        # DC_POS_TOL_DEG
    dc_moving_rpm: float = 2.0         # DC_MOVING_RPM
    dc_max_rpm: float = 140.0          # DC_MAX_RPM
    dc_counts_per_rev: float = 180.0   # DC_COUNTS_PER_REV
    map_flex_deadzone_deg: float = 10.0  # MAP_FLEX_DEADZONE_DEG
    map_flex_full_deg: float = 90.0    # MAP_FLEX_FULL_DEG
    map_dc_max_rpm: float = 60.0       # MAP_DC_MAX_RPM
    flex_adc_open: float = 5.0         # FLEX_ADC_OPEN: at/below = flex fault
    flex_adc_short: float = 1018.0     # FLEX_ADC_SHORT: at/above = flex fault
    stepper_steps_per_rev: float = 1600.0  # STEPPER_STEPS_PER_REV
    tof_stale_ms: float = 500.0        # TOF_STALE_MS
    interlock_temp_c: float = 30.0     # INTERLOCK_TEMP_C
    # bridge-only
    settle_lines: int = 5              # 5 lines = 250 ms at 20 Hz
    cmd_timeout_s: float = 1.0
    rate_window_s: float = 2.0


def _atof(text: str) -> float:
    """C atof(): leading number or 0 (the firmware parses commands this way)."""
    num = ""
    for ch in text.strip():
        if ch.isdigit() or ch in "+-." or (ch in "eE" and num):
            num += ch
        else:
            break
    try:
        return float(num)
    except ValueError:
        return 0.0


class _Pending:
    def __init__(self, cmd: str, sent: float, check):
        self.cmd, self.sent, self.check = cmd, sent, check


class Feedback:
    def __init__(self, cfg: Optional[FeedbackConfig] = None):
        self.cfg = cfg or FeedbackConfig()
        self.dc_mode = DC_IDLE
        self.dc_target = 0.0
        self.dc_arrived = False
        self.dc_settled = False
        self.tof_fault = True
        self.source = "inferred"     # "firmware" once lines carry dc_mode..tof_fault
        self._settle_run = 0
        self._last: Optional[Dict[str, float]] = None
        self._prev_mode: Optional[int] = None
        self._prev_interlock = False
        self._map_sent = False
        self._last_map_rpm = 0.0
        self._tof_raw: Optional[float] = None
        self._tof_changed_at: Optional[float] = None
        self._line_times: Deque[float] = deque()
        self.last_line_at: Optional[float] = None
        self._pending: List[_Pending] = []
        self._step_expected: Optional[float] = None
        self._events: List[str] = []

    # ------------------------------------------------------------ outputs
    def take_status(self) -> List[str]:
        out, self._events = self._events, []
        return out

    def rate_hz(self, now: float) -> float:
        self._trim(now)
        return len(self._line_times) / self.cfg.rate_window_s

    def line_age_s(self, now: float) -> Optional[float]:
        return None if self.last_line_at is None else now - self.last_line_at

    # ------------------------------------------------------------ DC model
    def _dc_idle(self) -> None:
        self.dc_mode, self.dc_target = DC_IDLE, 0.0

    def _set_velocity(self, rpm: float) -> None:
        self.dc_mode = DC_VELOCITY
        self.dc_target = max(-self.cfg.dc_max_rpm, min(self.cfg.dc_max_rpm, rpm))

    def _flex_rpm(self, t: Dict[str, float]) -> float:
        c = self.cfg
        if t["flex_raw"] <= c.flex_adc_open or t["flex_raw"] >= c.flex_adc_short:
            return 0.0                           # flex fault -> 0 rpm
        bend = t["flex_filt_deg"]
        if bend <= c.map_flex_deadzone_deg:
            return 0.0
        f = (bend - c.map_flex_deadzone_deg) / (c.map_flex_full_deg - c.map_flex_deadzone_deg)
        return max(0.0, min(1.0, f)) * c.map_dc_max_rpm

    # ------------------------------------------------------------ inputs
    def on_command(self, cmd: str, now: float) -> None:
        """A command the bridge has just written to the Uno."""
        t = self._last
        parts = cmd.split(",")
        if parts[0] == "M" and len(parts) == 2:
            want = int(_atof(parts[1]))
            self._watch(cmd, now, lambda x: x["mode"] == want)
            return
        if t is not None and int(t["mode"]) == MODE_SENSOR:
            self._events.append(f"{cmd}: ignored (SENSOR mode)")
            return
        interlock = t is not None and bool(t["interlock"])
        if parts[0] == "S" and len(parts) == 2:
            target = max(0.0, min(180.0, _atof(parts[1])))
            self._watch(cmd, now, lambda x: abs(x["servo_deg"] - target) < 0.06)
        elif parts[0] == "P" and len(parts) == 2:
            spr = self.cfg.stepper_steps_per_rev
            steps = round(_atof(parts[1]) * spr / 360.0)
            base = self._step_expected if self._step_expected is not None else (
                t["step_target_deg"] if t is not None else 0.0)
            expected = base + steps * 360.0 / spr
            self._step_expected = expected
            self._watch(cmd, now, lambda x: abs(x["step_target_deg"] - expected) < 0.006)
        elif parts[:2] == ["D", "X"]:
            self._dc_idle()
            self._watch(cmd, now, lambda x: x["dc_pwm"] == 0)
        elif parts[:2] == ["D", "P"] and len(parts) == 3:
            if interlock:
                self._events.append(f"{cmd}: ignored (interlock)")
                return
            cpr = self.cfg.dc_counts_per_rev
            counts = round(_atof(parts[2]) * cpr / 360.0)
            start = t["dc_deg"] if t is not None else 0.0
            self.dc_mode = DC_POSITION
            self.dc_target = start + counts * 360.0 / cpr
            self._settle_run = 0
            target, tol = self.dc_target, self.cfg.dc_pos_tol_deg
            self._watch(cmd, now, lambda x: x["dc_pwm"] != 0 or abs(x["dc_deg"] - target) <= tol)
        elif parts[:2] == ["D", "V"] and len(parts) == 3:
            if interlock:
                self._events.append(f"{cmd}: ignored (interlock)")
                return
            self._set_velocity(_atof(parts[2]))
            rpm = self.dc_target
            if rpm == 0:
                self._watch(cmd, now, lambda x: x["dc_pwm"] == 0)
            else:
                self._watch(cmd, now, lambda x: x["dc_pwm"] * rpm > 0)

    def on_telemetry(self, t: Dict[str, float], now: float) -> None:
        """One parsed telemetry line (tsp05_bridge.protocol.parse_telemetry)."""
        self.last_line_at = now
        self._line_times.append(now)
        self._trim(now)

        mode, interlock = int(t["mode"]), bool(t["interlock"])
        if self._prev_mode is not None and mode != self._prev_mode:
            self._dc_idle()                      # setMode() stops the DC motor
            self._map_sent = False               # SENSOR mode re-sends its first rpm
        if interlock and not self._prev_interlock:
            self._dc_idle()                      # dcSetInterlock(true) stops it
        if mode == MODE_SENSOR:
            # runSensorMappings(): send rpm on entry and on 0.5 rpm changes; during the
            # interlock dcSetVelocity() ignores it but the "last sent" value updates,
            # so the motor stays stopped after the interlock clears until flex changes.
            rpm = self._flex_rpm(t)
            if not self._map_sent or abs(rpm - self._last_map_rpm) >= 0.5:
                if not interlock:
                    self._set_velocity(rpm)
                self._last_map_rpm, self._map_sent = rpm, True

        if "dc_mode" in t:
            # Newer firmware sends these itself (docs/protocol.md): use them as they are.
            self.source = "firmware"
            self.dc_mode, self.dc_target = int(t["dc_mode"]), t["dc_target"]
            self.dc_arrived = bool(t["dc_arrived"])
        else:
            self.source = "inferred"
            self.dc_arrived = (self.dc_mode == DC_POSITION
                               and abs(t["dc_deg"] - self.dc_target) <= self.cfg.dc_pos_tol_deg)
        slow = abs(t["dc_rpm"]) < self.cfg.dc_moving_rpm
        self._settle_run = self._settle_run + 1 if (self.dc_arrived and slow) else 0
        self.dc_settled = self._settle_run >= self.cfg.settle_lines

        raw = t["tof_raw_mm"]
        if raw != self._tof_raw:
            self._tof_raw, self._tof_changed_at = raw, now
        if "tof_fault" in t:
            self.tof_fault = bool(t["tof_fault"])
        else:
            self.tof_fault = raw == 0 or (now - self._tof_changed_at) * 1000.0 >= self.cfg.tof_stale_ms

        self._prev_mode, self._prev_interlock, self._last = mode, interlock, t
        self._resolve(t, now)

    def tick(self, now: float) -> None:
        """Call regularly: reports commands whose effect never showed up."""
        for pend in list(self._pending):
            if now - pend.sent > self.cfg.cmd_timeout_s:
                self._pending.remove(pend)
                self._events.append(f"{pend.cmd}: no effect seen within {self.cfg.cmd_timeout_s:g} s")
        if not any(p.cmd.startswith("P,") for p in self._pending):
            self._step_expected = None

    # ------------------------------------------------------------ helpers
    def _watch(self, cmd: str, now: float, check) -> None:
        self._pending.append(_Pending(cmd, now, check))

    def _resolve(self, t: Dict[str, float], now: float) -> None:
        for pend in list(self._pending):
            if pend.check(t):
                self._pending.remove(pend)
                self._events.append(f"{pend.cmd}: applied in {1000.0 * (now - pend.sent):.0f} ms")
        if not any(p.cmd.startswith("P,") for p in self._pending):
            self._step_expected = None

    def _trim(self, now: float) -> None:
        while self._line_times and now - self._line_times[0] > self.cfg.rate_window_s:
            self._line_times.popleft()

