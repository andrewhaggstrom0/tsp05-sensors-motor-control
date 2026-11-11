#!/usr/bin/env python3
"""Fake TSP05 Uno for Track 5: a virtual serial port that behaves like firmware/tsp05.

Lets the ROS 2 bridge and the Foxglove GUI be built and tested without hardware,
and predicts what the real board will do. Other tracks can use it too.

It mirrors the firmware on main, not an idealised device:
  * every constant comes from firmware/tsp05/config.h (and the DC speed map from
    dc_control.cpp), so changing a value there changes the fake too
  * the telemetry line has the exact protocol.md fields and the firmware's
    number formats (temp_C and stepper angles with 2 decimals, the rest with 1)
  * commands are parsed like tsp05.ino: atof/atoi semantics, 31-char lines,
    and motor commands are ignored *silently* in SENSOR mode
  * servo: telemetry reports the commanded angle (servoDeg() is the command)
  * stepper: P adds to the target; the motor steps there at STEPPER_STEP_US per step
  * DC: D,P moves relative to the current measured position; the encoder has
    DC_COUNTS_PER_REV counts (2 deg steps); speed is an EMA of count differences;
    inside DC_POS_TOL_DEG it coasts and counts as arrived; PWM comes from the
    measured speed map; D,P / D,V are ignored while the interlock is active
  * interlock: thermistor fault or temp_C > INTERLOCK_TEMP_C, no hysteresis; when
    it trips the DC motor stops and stays stopped after it clears
  * mode switch (M or the button) stops the DC motor and drops pending slot events
    and swipes; the servo stays where it is
  * SENSOR mode: a TOF swipe sweeps the servo at GESTURE_SERVO_DEG_S to the end of
    travel (right-to-left toward MAP_SERVO_MAX_DEG); flex bend past the dead zone
    sets the DC rpm, re-sent only when it changes by 0.5 rpm; each slot event jogs
    the stepper MAP_SLOT_JOG_DEG
  * TOF: raw and filtered (median of TOF_MEDIAN_N, then EMA) update only on valid
    ranges, so with no hand in view both freeze at their last value, like hardware
  * slot: SLOT_TABS tabs per revolution, rpm from the time across one revolution,
    recomputed every SLOT_RPM_PERIOD_MS, 0 after SLOT_TIMEOUT_MS without an event

Simulated world (the only parts not taken from the firmware): a hand swipes past
the TOF twice every 10 s (right-to-left at 2 s, left-to-right at 6 s), the flex
is bent slowly 0-90 deg, the slot disc spins at 30 rpm for 10 s out of every 20,
and with --temp cycle the thermistor is pinched above the interlock every 40 s.

Examples:
  python3 tools/fake_arduino.py                       # prints the pty path
  python3 tools/fake_arduino.py --link /tmp/tsp05     # also a stable symlink
  python3 tools/fake_arduino.py --print               # echo lines to stdout
  python3 tools/fake_arduino.py --bad-rate 0.05       # 5 % malformed lines
  python3 tools/fake_arduino.py --temp room           # never trips the interlock

Standard library only. Works on Linux, macOS, and in the docker/ container.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import re
import select
import signal
import sys
import time
import tty
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analog_model as am  # noqa: E402  (parses firmware/tsp05/config.h)

FIRMWARE = am.REPO / "firmware" / "tsp05"


def cfg(name: str) -> float:
    """A numeric #define from firmware/tsp05/config.h; fail loudly if it was renamed."""
    if name not in am.C:
        raise KeyError(f"{name} is not defined in firmware/tsp05/config.h; update tools/fake_arduino.py")
    return am.C[name]


# ---------------------------------------------------------------- firmware constants
TELEMETRY_PERIOD_S = cfg("TELEMETRY_PERIOD_MS") / 1000
MAPPING_PERIOD_S = cfg("MAPPING_PERIOD_MS") / 1000
ANALOG_PERIOD_S = cfg("ANALOG_PERIOD_MS") / 1000
SERVO_START_DEG = cfg("SERVO_START_DEG")
STEPS_PER_REV = int(cfg("STEPPER_STEPS_PER_REV"))
STEP_PERIOD_S = cfg("STEPPER_STEP_US") / 1e6
MAP_SERVO_MIN_DEG = cfg("MAP_SERVO_MIN_DEG")
MAP_SERVO_MAX_DEG = cfg("MAP_SERVO_MAX_DEG")
MAP_FLEX_DEADZONE_DEG = cfg("MAP_FLEX_DEADZONE_DEG")
MAP_FLEX_FULL_DEG = cfg("MAP_FLEX_FULL_DEG")
MAP_DC_MAX_RPM = cfg("MAP_DC_MAX_RPM")
MAP_SLOT_JOG_DEG = cfg("MAP_SLOT_JOG_DEG")
INTERLOCK_TEMP_C = cfg("INTERLOCK_TEMP_C")
DC_PERIOD_S = cfg("DC_PERIOD_MS") / 1000
DC_COUNTS_PER_REV = cfg("DC_COUNTS_PER_REV")
DC_MAX_RPM = cfg("DC_MAX_RPM")
DC_RPM_EMA_ALPHA = cfg("DC_RPM_EMA_ALPHA")
DC_POS_TOL_DEG = cfg("DC_POS_TOL_DEG")
DC_POS_MAX_PWM = cfg("DC_POS_MAX_PWM")
DC_POS_KP = cfg("DC_POS_KP")                 # PWM per degree of error
DC_DEADBAND_RUN_PWM = cfg("DC_DEADBAND_RUN_PWM")
TOF_PERIOD_S = cfg("TOF_PERIOD_MS") / 1000
TOF_MEDIAN_N = int(cfg("TOF_MEDIAN_N"))
TOF_EMA_ALPHA = cfg("TOF_EMA_ALPHA")
TOF_STALE_S = cfg("TOF_STALE_MS") / 1000
GESTURE_NEAR_MM = cfg("GESTURE_NEAR_MM")
GESTURE_CLEAR_S = cfg("GESTURE_CLEAR_MS") / 1000
GESTURE_SERVO_DEG_S = cfg("GESTURE_SERVO_DEG_S")
SLOT_TABS = int(cfg("SLOT_TABS"))
SLOT_RPM_PERIOD_S = cfg("SLOT_RPM_PERIOD_MS") / 1000
SLOT_TIMEOUT_S = cfg("SLOT_TIMEOUT_MS") / 1000

# Measured DC speed map (PWM -> counts/s), the firmware's feedforward table.
_DC_SRC = FIRMWARE / "dc_control.cpp"
FF_PWM = am._parse_float_array(_DC_SRC, "FF_PWM")
FF_CPS_FWD = am._parse_float_array(_DC_SRC, "FF_CPS_FWD")
FF_CPS_REV = am._parse_float_array(_DC_SRC, "FF_CPS_REV")

RX_MAX = 31          # tsp05.ino: char g_rx[32]; longer lines are discarded
SIM_DT = 0.01        # simulation step (s); matches the DC control period

# ---------------------------------------------------------------- plant assumptions
# Not in the firmware: how the physical parts respond. Chosen to match bench results
# calibration/data/dc_step_p90_filtD01.csv (90 deg step: in band 0.22 s, stopped
# 0.60 s, peak 84 rpm, peak PWM 224) and 30 rpm holding at about 30 +- 2 rpm.
DC_TAU_S = 0.12                     # motor speed time constant while driven
DC_COAST_TAU_S = 0.05               # unpowered: gearbox friction stops it quickly
TOF_HAND_MM = 250                   # distance of the swiping hand
TOF_NOISE_MM = 2.0                  # Harish measured 2.1 mm raw noise
DISC_RPM = 30.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def rpm_from_span(span_s: float, edges: int) -> float:
    """digital_math::rpmFromSpan(): edges intervals took span_s; 0 for an empty span.
    (An edge and an rpm update can land on the same instant: span 0.)"""
    if span_s <= 0 or edges <= 0:
        return 0.0
    return 60.0 * edges / (SLOT_TABS * span_s)


def feedforward_pwm(rpm: float) -> float:
    """dc_control.cpp feedforwardPwm(): PWM that holds this speed (measured map)."""
    if rpm == 0.0:
        return 0.0
    cps_table = FF_CPS_FWD if rpm > 0 else FF_CPS_REV
    want = abs(rpm) * DC_COUNTS_PER_REV / 60.0
    pwm = FF_PWM[-1]
    for i in range(1, len(FF_PWM)):
        if want <= cps_table[i]:
            pwm = FF_PWM[i - 1] + (FF_PWM[i] - FF_PWM[i - 1]) * (want - cps_table[i - 1]) / (cps_table[i] - cps_table[i - 1])
            break
    return pwm if rpm > 0 else -pwm


def deadband_comp(u: float, deadband: float) -> int:
    """dc_control.cpp deadbandComp(): any non-zero command becomes +/-(deadband..255)."""
    u = clamp(u, -255.0, 255.0)
    if abs(u) < 0.5:
        return 0
    mag = int(round(deadband + abs(u) * (255 - deadband) / 255.0))
    return mag if u > 0 else -mag


def rpm_from_pwm(pwm: float) -> float:
    """Steady speed at this PWM: the measured speed map, inverted (plant model)."""
    if pwm == 0:
        return 0.0
    cps_table = FF_CPS_FWD if pwm > 0 else FF_CPS_REV
    p = abs(pwm)
    if p <= FF_PWM[0]:
        return 0.0
    cps = cps_table[-1]
    for i in range(1, len(FF_PWM)):
        if p <= FF_PWM[i]:
            cps = cps_table[i - 1] + (cps_table[i] - cps_table[i - 1]) * (p - FF_PWM[i - 1]) / (FF_PWM[i] - FF_PWM[i - 1])
            break
    rpm = cps * 60.0 / DC_COUNTS_PER_REV
    return rpm if pwm > 0 else -rpm


_NUM = re.compile(r"\s*[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")


def c_atof(text: str) -> float:
    """C atof(): the longest numeric prefix, 0.0 if there is none."""
    m = _NUM.match(text)
    return float(m.group(0)) if m else 0.0


def c_atoi(text: str) -> int:
    m = re.match(r"\s*[+-]?\d+", text)
    return int(m.group(0)) if m else 0


class FakeUno:
    """The firmware's behaviour plus a simulated bench. Time advances only in step()."""

    MODE_GUI, MODE_SENSOR = 0, 1

    def __init__(self, temp_profile: str = "cycle", bad_rate: float = 0.0, seed: int | None = None):
        self.rng = random.Random(seed)
        self.temp_profile = temp_profile
        self.bad_rate = bad_rate
        self.t = 0.0
        self.cmd_count = 0
        self.ignored_count = 0
        self._due: dict[str, float] = {}

        self.mode = self.MODE_GUI
        self.interlock = False
        # analog
        self.flex_raw, self.flex_deg, self.flex_filt_deg, self.flex_fault = 0, 0.0, 0.0, False
        self._flex_seeded = False
        self.th_raw, self.temp_c, self.th_fault = 0, 0.0, False
        # TOF
        self.tof_raw_mm, self.tof_filt_mm = 0, 0.0
        self._tof_hist: list[int] = []
        self._tof_seeded = False
        self._tof_last_valid = None   # time of the last valid range (for tof_fault())
        self._pass_dir = 0            # swipe direction of the hand pass in progress
        self._pass_last_near = None   # time the hand was last seen
        self._swipe = 0               # pending swipe for tofTakeSwipe()
        # slot
        self.slot_count = 0
        self.slot_rpm = 0.0
        self._slot_edges: list[float] = []
        self._slot_pending = 0
        self._disc_phase = 0.0
        # servo / stepper
        self.servo_deg = SERVO_START_DEG
        self.step_pos = 0
        self.step_target = 0
        self._step_acc = 0.0
        # DC
        self.dc_mode = "idle"         # idle | velocity | position
        self.dc_target_rpm = 0.0
        self.dc_target_counts = 0
        self.dc_arrived = True
        self._dc_shaft_deg = 0.0      # true shaft angle
        self._dc_speed = 0.0          # true speed, rpm
        self._dc_counts = 0
        self.dc_rpm = 0.0             # EMA estimate, as reported
        self.dc_pwm = 0
        # SENSOR-mode mapping state (tsp05.ino)
        self._sweep_dir = 0
        self._map_rpm_sent = False
        self._last_map_rpm = 0.0

    # ------------------------------------------------------------ scheduling
    def _every(self, name: str, period: float) -> bool:
        due = self._due.get(name, 0.0)
        if self.t + 1e-9 >= due:
            self._due[name] = (due if due else self.t) + period
            return True
        return False

    def step(self, dt: float = SIM_DT) -> None:
        self.t += dt
        if self._every("analog", ANALOG_PERIOD_S):
            self._analog()
        if self._every("tof", TOF_PERIOD_S):
            self._tof()
        self._slot_disc(dt)
        if self._every("slot_rpm", SLOT_RPM_PERIOD_S):
            self._slot_rpm()
        self._dc(dt)
        self._stepper(dt)
        self._update_interlock()
        if self._every("mapping", MAPPING_PERIOD_S):
            if self.mode == self.MODE_SENSOR:
                self._sensor_mappings()
            else:
                self._take_slot_events()
                self._take_swipe()

    # ------------------------------------------------------------ simulated sensors
    def _analog(self) -> None:
        rng = self.rng
        true_deg = 45.0 - 45.0 * math.cos(2 * math.pi * 0.08 * self.t)
        self.flex_raw = int(clamp(round(am.adc_from_flex_deg(true_deg) + rng.gauss(0, 3.0)), 0, am.ADC_MAX))
        self.flex_fault = am.flex_adc_is_fault(self.flex_raw)
        self.flex_deg = clamp(am.flex_deg_from_adc(self.flex_raw), am.FLEX_DEG_MIN, am.FLEX_DEG_MAX)
        self.flex_filt_deg = am.ema(self.flex_filt_deg, self.flex_deg) if self._flex_seeded else self.flex_deg
        self._flex_seeded = True

        temp = 22.0 + 0.3 * math.sin(2 * math.pi * self.t / 300)
        if self.temp_profile == "cycle":
            phase = self.t % 40.0
            if 25.0 < phase < 35.0:
                temp += 11.0 * math.sin(math.pi * (phase - 25.0) / 10.0)
        self.th_raw = int(clamp(round(am.adc_from_therm_c(temp) + rng.gauss(0, 1.0)), 0, am.ADC_MAX))
        self.th_fault = am.therm_adc_is_fault(self.th_raw)
        self.temp_c = am.therm_c_from_adc(self.th_raw)

    def _hand(self) -> int:
        """Swipe direction of a hand in view now (+1 left-to-right, -1 right-to-left), else 0."""
        phase = self.t % 10.0
        if 2.0 <= phase < 2.4:
            return -1
        if 6.0 <= phase < 6.4:
            return +1
        return 0

    def _tof(self) -> None:
        hand = self._hand()
        if hand:
            mm = int(round(TOF_HAND_MM + self.rng.gauss(0, TOF_NOISE_MM)))
            if 0 < mm < GESTURE_NEAR_MM:
                self._pass_dir = hand
                self._pass_last_near = self.t
            # valid range: same median + EMA as sensors_digital.cpp
            self.tof_raw_mm = mm
            self._tof_hist = (self._tof_hist + [mm])[-TOF_MEDIAN_N:]
            med = sorted(self._tof_hist)[len(self._tof_hist) // 2] if len(self._tof_hist) % 2 else \
                sum(sorted(self._tof_hist)[len(self._tof_hist) // 2 - 1:len(self._tof_hist) // 2 + 1]) / 2
            self.tof_filt_mm = self.tof_filt_mm + TOF_EMA_ALPHA * (med - self.tof_filt_mm) if self._tof_seeded else med
            self._tof_seeded = True
            self._tof_last_valid = self.t
        # no hand: every range fails (nothing in short-range view), values hold
        if self._pass_dir and not hand and self.t - self._pass_last_near >= GESTURE_CLEAR_S:
            self._swipe = self._pass_dir      # both zones clear long enough: report the swipe
            self._pass_dir = 0

    def tof_fault(self) -> bool:
        """tofFault(): no valid range yet, or none for longer than TOF_STALE_MS.
        Not in the telemetry line; tests use it as ground truth."""
        return self._tof_last_valid is None or self.t - self._tof_last_valid > TOF_STALE_S

    def _take_swipe(self) -> int:
        s, self._swipe = self._swipe, 0
        return s

    def _slot_disc(self, dt: float) -> None:
        rpm = DISC_RPM if (self.t % 20.0) < 10.0 else 0.0
        self._disc_phase += rpm / 60.0 * SLOT_TABS * dt
        while self._disc_phase >= 1.0:
            self._disc_phase -= 1.0
            self.slot_count += 1
            self._slot_pending += 1
            self._slot_edges = (self._slot_edges + [self.t])[-(SLOT_TABS + 1):]

    def _slot_rpm(self) -> None:
        """sensors_digital.cpp slotUpdate(): time across one revolution, with the
        slowing-down bound and the timeout."""
        e = self._slot_edges
        if not e or self.t - e[-1] > SLOT_TIMEOUT_S:
            self.slot_rpm = 0.0
            if e and self.t - e[-1] > SLOT_TIMEOUT_S:
                self._slot_edges = []          # stopped: the next spin starts fresh
            return
        edges = min(len(e) - 1, SLOT_TABS)
        if edges <= 0:
            self.slot_rpm = 0.0
            return
        newest, oldest, second = e[-1], e[-1 - edges], e[-edges]
        since = self.t - newest
        rpm = rpm_from_span(newest - oldest, edges)
        bound = rpm_from_span((newest - second) + since, edges)
        self.slot_rpm = bound if 0 < bound < rpm else rpm

    def _take_slot_events(self) -> int:
        n, self._slot_pending = self._slot_pending, 0
        return n

    # ------------------------------------------------------------ actuators
    def _stepper(self, dt: float) -> None:
        if self.step_pos == self.step_target:
            self._step_acc = 0.0
            return
        self._step_acc += dt
        while self._step_acc >= STEP_PERIOD_S and self.step_pos != self.step_target:
            self._step_acc -= STEP_PERIOD_S
            self.step_pos += 1 if self.step_target > self.step_pos else -1

    def stepper_move_relative(self, deg: float) -> None:
        self.step_target += round(deg * STEPS_PER_REV / 360.0)

    def servo_set(self, deg: float) -> None:
        self.servo_deg = clamp(deg, 0.0, 180.0)

    def dc_deg(self) -> float:
        return self._dc_counts * 360.0 / DC_COUNTS_PER_REV

    def dc_move_relative(self, deg: float) -> None:
        if self.interlock:
            return
        self.dc_mode = "position"
        self.dc_target_counts = self._dc_counts + round(deg * DC_COUNTS_PER_REV / 360.0)
        self.dc_arrived = False

    def dc_set_velocity(self, rpm: float) -> None:
        if self.interlock:
            return
        self.dc_mode = "velocity"
        self.dc_target_rpm = clamp(rpm, -DC_MAX_RPM, DC_MAX_RPM)

    def dc_stop(self) -> None:
        self.dc_mode = "idle"
        self.dc_pwm = 0

    def _dc(self, dt: float) -> None:
        drive_rpm = 0.0
        pwm = 0.0
        if self.dc_mode == "velocity" and self.dc_target_rpm != 0.0:
            drive_rpm = self.dc_target_rpm
            pwm = feedforward_pwm(drive_rpm)
        elif self.dc_mode == "position":
            err_deg = (self.dc_target_counts - self._dc_counts) * 360.0 / DC_COUNTS_PER_REV
            if abs(err_deg) <= DC_POS_TOL_DEG:
                self.dc_arrived = True        # coast inside the band
            else:
                self.dc_arrived = False
                u = clamp(DC_POS_KP * err_deg, -DC_POS_MAX_PWM, DC_POS_MAX_PWM)
                pwm = deadband_comp(u, DC_DEADBAND_RUN_PWM)
                drive_rpm = rpm_from_pwm(pwm)
        if self.interlock:
            drive_rpm, pwm = 0.0, 0.0
        self.dc_pwm = int(round(clamp(pwm, -255, 255)))
        # plant: first-order speed response, coasting toward 0 when not driven
        tau = DC_TAU_S if drive_rpm != 0.0 else DC_COAST_TAU_S
        self._dc_speed += (drive_rpm - self._dc_speed) * min(1.0, dt / tau)
        if drive_rpm == 0.0 and abs(self._dc_speed) < 0.5:
            self._dc_speed = 0.0
        self._dc_shaft_deg += self._dc_speed * 6.0 * dt
        counts = int(math.floor(self._dc_shaft_deg * DC_COUNTS_PER_REV / 360.0 + 0.5))
        raw_rpm = (counts - self._dc_counts) / DC_COUNTS_PER_REV / dt * 60.0
        self._dc_counts = counts
        self.dc_rpm += DC_RPM_EMA_ALPHA * (raw_rpm - self.dc_rpm)

    def _update_interlock(self) -> None:
        want = self.th_fault or self.temp_c > INTERLOCK_TEMP_C
        if want != self.interlock:
            if want and not self.interlock:
                self.dc_stop()
            self.interlock = want

    # ------------------------------------------------------------ tsp05.ino logic
    def set_mode(self, m: int) -> None:
        if m == self.mode:
            return
        self.mode = m
        self.dc_stop()
        self._map_rpm_sent = False
        self._take_slot_events()
        self._take_swipe()
        self._sweep_dir = 0

    def press_button(self) -> None:
        self.set_mode(self.MODE_SENSOR if self.mode == self.MODE_GUI else self.MODE_GUI)

    def _sensor_mappings(self) -> None:
        swipe = self._take_swipe()
        if swipe:
            self._sweep_dir = +1 if swipe < 0 else -1
        if self._sweep_dir:
            deg = self.servo_deg + self._sweep_dir * GESTURE_SERVO_DEG_S * MAPPING_PERIOD_S
            if deg >= MAP_SERVO_MAX_DEG:
                deg, self._sweep_dir = MAP_SERVO_MAX_DEG, 0
            if deg <= MAP_SERVO_MIN_DEG:
                deg, self._sweep_dir = MAP_SERVO_MIN_DEG, 0
            self.servo_set(deg)

        bend, rpm = self.flex_filt_deg, 0.0
        if not self.flex_fault and bend > MAP_FLEX_DEADZONE_DEG:
            f = clamp((bend - MAP_FLEX_DEADZONE_DEG) / (MAP_FLEX_FULL_DEG - MAP_FLEX_DEADZONE_DEG), 0.0, 1.0)
            rpm = f * MAP_DC_MAX_RPM
        if not self._map_rpm_sent or abs(rpm - self._last_map_rpm) >= 0.5:
            self.dc_set_velocity(rpm)
            self._last_map_rpm = rpm
            self._map_rpm_sent = True

        ev = self._take_slot_events()
        if ev:
            self.stepper_move_relative(ev * MAP_SLOT_JOG_DEG)

    def handle(self, line: str) -> None:
        """One received line, parsed exactly like tsp05.ino handleCommand(). Never replies."""
        line = line.replace("\r", "")
        self.cmd_count += 1
        if len(line) > RX_MAX or not line:
            self.ignored_count += 1
            return
        if line.startswith("M,"):
            m = c_atoi(line[2:])
            if m in (self.MODE_GUI, self.MODE_SENSOR):
                self.set_mode(m)
            else:
                self.ignored_count += 1
            return
        if self.mode != self.MODE_GUI:
            self.ignored_count += 1           # motor commands ignored in SENSOR mode
            return
        if line.startswith("S,"):
            self.servo_set(c_atof(line[2:]))
        elif line.startswith("P,"):
            self.stepper_move_relative(c_atof(line[2:]))
        elif line.startswith("D,") and len(line) >= 3:
            sub = line[2]
            if sub == "X":
                self.dc_stop()
            elif sub == "P" and line[3:4] == ",":
                self.dc_move_relative(c_atof(line[4:]))
            elif sub == "V" and line[3:4] == ",":
                self.dc_set_velocity(c_atof(line[4:]))
            else:
                self.ignored_count += 1
        else:
            self.ignored_count += 1

    def telemetry(self) -> str:
        """buildTelemetry(): same field order and decimals as tsp05.ino."""
        fields = [
            "T", str(int(round(self.t * 1000))), str(self.mode),
            str(self.flex_raw), f"{self.flex_deg:.1f}", f"{self.flex_filt_deg:.1f}",
            str(self.tof_raw_mm), f"{self.tof_filt_mm:.1f}",
            str(self.th_raw), f"{self.temp_c:.2f}",
            str(self.slot_count), f"{self.slot_rpm:.1f}",
            f"{self.servo_deg:.1f}", f"{self.dc_deg():.1f}", f"{self.dc_rpm:.1f}", str(self.dc_pwm),
            f"{self.step_pos * 360.0 / STEPS_PER_REV:.2f}",
            f"{self.step_target * 360.0 / STEPS_PER_REV:.2f}",
            "1" if self.interlock else "0",
        ]
        line = ",".join(fields)
        if self.bad_rate and self.rng.random() < self.bad_rate:
            line = self._corrupt(line)
        return line

    def _corrupt(self, line: str) -> str:
        """Malformed lines the bridge must survive: cut lines, noise, bad numbers."""
        kind = self.rng.randrange(4)
        if kind == 0:
            return line[: self.rng.randrange(2, len(line) - 1)]
        if kind == 1:
            return "".join(chr(self.rng.randrange(33, 127)) for _ in range(20))
        if kind == 2:
            return line.replace(",", ",x", 1)
        return line + ",99"


def open_pty(link: str | None) -> tuple[int, str]:
    master, slave = os.openpty()
    tty.setraw(slave)            # no echo, no CR/LF translation: behaves like a USB serial port
    # Never block on write: if nobody is reading, the pty buffer fills (only a
    # few KB on macOS) and output is dropped, like a real Uno's TX with no host.
    os.set_blocking(master, False)
    path = os.ttyname(slave)
    if link:
        try:
            os.unlink(link)
        except FileNotFoundError:
            pass
        os.symlink(path, link)
    # Keep the slave fd open so the port does not hang up between bridge restarts.
    return master, path


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--link", help="also create a symlink at this path (e.g. /tmp/tsp05)")
    ap.add_argument("--print", action="store_true", help="echo telemetry and commands to stdout")
    ap.add_argument("--temp", choices=("cycle", "room"), default="cycle",
                    help="cycle: pinch the thermistor every 40 s (trips the interlock); room: never")
    ap.add_argument("--bad-rate", type=float, default=0.0,
                    help="fraction of telemetry lines to corrupt (robustness testing)")
    ap.add_argument("--seed", type=int, help="random seed for repeatable runs")
    args = ap.parse_args()

    master, path = open_pty(args.link)
    uno = FakeUno(args.temp, args.bad_rate, args.seed)
    print(f"fake Arduino on {path}" + (f" (symlink {args.link})" if args.link else ""), flush=True)

    def stop(*_):
        if args.link:
            try:
                os.unlink(args.link)
            except FileNotFoundError:
                pass
        print(f"\nstopped: {uno.cmd_count} commands received, {uno.ignored_count} ignored")
        sys.exit(0)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    def send(text: str, end: str = "\n") -> None:
        try:
            os.write(master, (text + end).encode())
        except OSError:
            pass          # nobody reading yet; drop like a real Uno would
        if args.print:
            print(text, flush=True)

    send("# TSP05 starting (fake_arduino.py)", end="\r\n")   # Serial.println: CRLF
    rx = b""
    next_sim = next_tel = time.monotonic()
    while True:
        now = time.monotonic()
        timeout = max(0.0, min(next_sim, next_tel) - now)
        ready, _, _ = select.select([master], [], [], timeout)
        if ready:
            try:
                rx += os.read(master, 1024)
            except OSError:
                rx = b""
            while b"\n" in rx:
                raw, rx = rx.split(b"\n", 1)
                cmd = raw.decode(errors="replace")
                if not cmd.strip():
                    continue
                if args.print:
                    print(f"> {cmd.strip()}", flush=True)
                uno.handle(cmd)
            rx = rx[-256:]       # a runaway line without newline cannot grow forever
        now = time.monotonic()
        if now >= next_sim:
            uno.step(SIM_DT)
            next_sim += SIM_DT
            if now - next_sim > 0.5:          # fell far behind (laptop slept): resync
                next_sim = now + SIM_DT
        if now >= next_tel:
            send(uno.telemetry())             # buildTelemetry() ends with '\n' only
            next_tel += TELEMETRY_PERIOD_S
            if now - next_tel > 0.5:
                next_tel = now + TELEMETRY_PERIOD_S


if __name__ == "__main__":
    main()
