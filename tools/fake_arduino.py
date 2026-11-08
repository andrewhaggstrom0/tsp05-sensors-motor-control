#!/usr/bin/env python3
"""Fake TSP05 Uno for Track 5: a virtual serial port that speaks docs/protocol.md.

Lets the ROS 2 bridge and the Foxglove GUI be built and tested before the
integrated firmware exists. Other tracks can use it to check their parsing too.

What it does:
  * opens a pseudo-terminal (pty) and prints its path; the bridge opens that
    path exactly as it would open /dev/ttyACM0
  * sends one telemetry line at 20 Hz, fields in the exact protocol.md order:
      T,ms,mode,flex_raw,flex_deg,flex_filt_deg,tof_raw_mm,tof_filt_mm,th_raw,
      temp_C,slot_count,slot_rpm,servo_deg,dc_deg,dc_rpm,dc_pwm,step_deg,
      step_target_deg,interlock
  * simulates the sensors (smooth waves + noise) and the motors (servo slew,
    first-order DC motor with a position loop, stepper in 0.225 deg steps)
  * obeys every protocol command: M,0/1  S,<deg>  D,P,<deg>  D,V,<rpm>  D,X
    P,<deg>. Motor commands are ignored in SENSOR mode, as the protocol says.
  * SENSOR mode mapping: TOF -> servo, flex -> DC RPM, slot event -> stepper jog
  * thermal interlock: temp above INTERLOCK_ON_C stops the DC motor in both modes

Raw ADC counts come from tools/analog_model.py (which reads firmware/tsp05/
config.h), so flex_raw/th_raw are consistent with the real conversions.

Examples:
  python3 tools/fake_arduino.py                       # prints the pty path
  python3 tools/fake_arduino.py --link /tmp/tsp05     # also a stable symlink
  python3 tools/fake_arduino.py --print               # echo lines to stdout
  python3 tools/fake_arduino.py --bad-rate 0.05       # 5 % malformed lines
  python3 tools/fake_arduino.py --temp room           # never trips interlock

Standard library only. Works on Linux, macOS, and in the docker/ container.
"""
from __future__ import annotations

import argparse
import math
import os
import random
import select
import signal
import sys
import time
import tty
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import analog_model as am  # noqa: E402

TELEMETRY_HZ = 20
SIM_HZ = 100

# Mirrors of firmware behaviour. The real values will live in config.h
# (Track 2); keep these in step if they change.
SERVO_MIN_DEG, SERVO_MAX_DEG = 0.0, 180.0
SERVO_SLEW_DEG_S = 300.0          # HS-311 is ~0.19 s / 60 deg unloaded
DC_MAX_RPM = 120.0                # SPG30 output shaft at 12 V (approx.)
DC_TAU_S = 0.15                   # first-order velocity time constant
DC_DEADBAND_PWM = 40              # PWM added to any non-zero output
DC_POS_KP = 2.0                   # RPM per degree of position error
DC_POS_MAX_RPM = 60.0
DC_ARRIVED_DEG = 2.0
STEPPER_DEG_PER_STEP = 360.0 / 1600   # 1/8 microstepping
STEPPER_DEG_S = 90.0
STEPPER_JOG_DEG = 15.0            # SENSOR mode: one slot event
TOF_MIN_MM, TOF_MAX_MM = 50.0, 1000.0   # SENSOR mode: maps onto servo 0..180
FLEX_MAP_MAX_DEG = 90.0                 # SENSOR mode: 0..90 deg -> 0..DC_MAX_RPM
INTERLOCK_ON_C, INTERLOCK_OFF_C = 30.0, 28.0
SLOT_TABS = 4
SLOT_WINDOW_S = 0.1               # 10 Hz RPM window (Track 3)
SLOT_TIMEOUT_S = 2.0


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


@dataclass
class State:
    mode: int = 0                      # 0 GUI, 1 SENSOR
    # sensors
    flex_raw: int = 0
    flex_deg: float = 0.0
    flex_filt_deg: float = 0.0
    tof_raw_mm: float = 0.0
    tof_filt_mm: float = 0.0
    th_raw: int = 0
    temp_c: float = 0.0
    slot_count: int = 0
    slot_rpm: float = 0.0
    # actuators
    servo_cmd_deg: float = 90.0
    servo_deg: float = 90.0
    dc_mode: str = "stop"              # stop | vel | pos
    dc_target_rpm: float = 0.0
    dc_target_deg: float = 0.0
    dc_deg: float = 0.0
    dc_rpm: float = 0.0
    dc_pwm: int = 0
    step_pos: int = 0                  # steps
    step_target: int = 0               # steps
    interlock: bool = False
    # bookkeeping
    tof_hist: list = field(default_factory=list)
    slot_times: list = field(default_factory=list)
    slot_events_pending: int = 0
    slot_phase: float = 0.0


class FakeUno:
    def __init__(self, temp_profile: str, bad_rate: float, seed: int | None):
        self.rng = random.Random(seed)
        self.temp_profile = temp_profile
        self.bad_rate = bad_rate
        self.s = State()
        self.t0 = time.monotonic()
        self.cmd_count = 0
        self.ignored_count = 0

    # ------------------------------------------------------------- sensors
    def _sensors(self, t: float, dt: float) -> None:
        s, rng = self.s, self.rng
        # Flex: slow bend 0..90 deg, quantised through the ADC like the real one
        true_deg = 45.0 - 45.0 * math.cos(2 * math.pi * 0.08 * t)
        adc = am.adc_from_flex_deg(true_deg) + rng.gauss(0, 3.0)
        s.flex_raw = int(clamp(round(adc), 0, am.ADC_MAX))
        s.flex_deg = clamp(am.flex_deg_from_adc(s.flex_raw), am.FLEX_DEG_MIN, am.FLEX_DEG_MAX)
        s.flex_filt_deg = am.ema(s.flex_filt_deg, s.flex_deg) if t > dt else s.flex_deg

        # TOF: hand moving 100..900 mm, sensor noise + occasional outlier;
        # filter = median of 5 then EMA, like Track 3's plan
        true_mm = 500.0 + 400.0 * math.sin(2 * math.pi * 0.05 * t)
        raw = true_mm + rng.gauss(0, 8.0)
        if rng.random() < 0.02:
            raw += rng.choice((-1, 1)) * rng.uniform(80, 200)
        s.tof_raw_mm = clamp(round(raw), 0, 4000)
        s.tof_hist = (s.tof_hist + [s.tof_raw_mm])[-5:]
        med = sorted(s.tof_hist)[len(s.tof_hist) // 2]
        s.tof_filt_mm = med if t <= dt else s.tof_filt_mm + 0.3 * (med - s.tof_filt_mm)

        # Thermistor: room temperature; "cycle" adds a pinch every 40 s that
        # crosses INTERLOCK_ON_C so the interlock can be seen in the GUI
        temp = 24.0 + 0.3 * math.sin(2 * math.pi * t / 300)
        if self.temp_profile == "cycle":
            phase = t % 40.0
            if 25.0 < phase < 35.0:
                temp += 9.0 * math.sin(math.pi * (phase - 25.0) / 10.0)
        adc = am.adc_from_therm_c(temp) + rng.gauss(0, 1.0)
        s.th_raw = int(clamp(round(adc), 0, am.ADC_MAX))
        s.temp_c = am.therm_c_from_adc(s.th_raw)

        # Slot sensor: a disc with SLOT_TABS tabs spun by hand for 10 s,
        # then left still for 10 s
        disc_rpm = 30.0 if (t % 20.0) < 10.0 else 0.0
        s.slot_phase += disc_rpm / 60.0 * SLOT_TABS * dt
        while s.slot_phase >= 1.0:
            s.slot_phase -= 1.0
            s.slot_count += 1
            s.slot_events_pending += 1
            s.slot_times.append(t)
        s.slot_times = [x for x in s.slot_times if t - x <= SLOT_TIMEOUT_S]
        if len(s.slot_times) >= 2:
            span = s.slot_times[-1] - s.slot_times[0]
            s.slot_rpm = (len(s.slot_times) - 1) / SLOT_TABS / span * 60.0 if span > 0 else 0.0
        else:
            s.slot_rpm = 0.0

    # ------------------------------------------------------------- motors
    def _motors(self, dt: float) -> None:
        s = self.s
        # Interlock with hysteresis
        if s.temp_c >= INTERLOCK_ON_C:
            s.interlock = True
        elif s.temp_c <= INTERLOCK_OFF_C:
            s.interlock = False

        if s.mode == 1:   # SENSOR mapping
            frac = clamp((s.tof_filt_mm - TOF_MIN_MM) / (TOF_MAX_MM - TOF_MIN_MM), 0, 1)
            s.servo_cmd_deg = SERVO_MIN_DEG + frac * (SERVO_MAX_DEG - SERVO_MIN_DEG)
            s.dc_mode = "vel"
            s.dc_target_rpm = clamp(s.flex_filt_deg / FLEX_MAP_MAX_DEG, 0, 1) * DC_MAX_RPM
            if s.slot_events_pending:
                s.step_target += round(s.slot_events_pending * STEPPER_JOG_DEG / STEPPER_DEG_PER_STEP)
        s.slot_events_pending = 0

        # Servo: slew-limited
        step = SERVO_SLEW_DEG_S * dt
        s.servo_deg += clamp(s.servo_cmd_deg - s.servo_deg, -step, step)

        # DC motor: outer position P loop -> velocity target -> first-order plant
        if s.interlock or s.dc_mode == "stop":
            target_rpm = 0.0
        elif s.dc_mode == "pos":
            err = s.dc_target_deg - s.dc_deg
            if abs(err) < DC_ARRIVED_DEG and abs(s.dc_rpm) < 5.0:
                s.dc_mode, target_rpm = "stop", 0.0
            else:
                target_rpm = clamp(DC_POS_KP * err, -DC_POS_MAX_RPM, DC_POS_MAX_RPM)
        else:
            target_rpm = clamp(s.dc_target_rpm, -DC_MAX_RPM, DC_MAX_RPM)
        s.dc_rpm += (target_rpm - s.dc_rpm) * min(1.0, dt / DC_TAU_S)
        if abs(target_rpm) < 1e-6 and abs(s.dc_rpm) < 0.5:
            s.dc_rpm = 0.0
        s.dc_deg += s.dc_rpm * 6.0 * dt          # RPM -> deg/s
        if target_rpm == 0.0:
            s.dc_pwm = 0
        else:
            pwm = target_rpm / DC_MAX_RPM * (255 - DC_DEADBAND_PWM)
            s.dc_pwm = int(round(pwm + math.copysign(DC_DEADBAND_PWM, pwm)))

        # Stepper: constant speed toward the target, whole steps only
        max_steps = max(1, int(STEPPER_DEG_S / STEPPER_DEG_PER_STEP * dt))
        diff = s.step_target - s.step_pos
        s.step_pos += int(clamp(diff, -max_steps, max_steps))

    def step(self, dt: float) -> None:
        t = time.monotonic() - self.t0
        self._sensors(t, dt)
        self._motors(dt)

    # ------------------------------------------------------------- protocol
    def handle(self, line: str) -> str | None:
        """Apply one command. Returns a '#' comment line to send back, or None."""
        s = self.s
        parts = line.strip().split(",")
        self.cmd_count += 1
        try:
            if parts[0] == "M" and len(parts) == 2 and parts[1] in ("0", "1"):
                s.mode = int(parts[1])
                if s.mode == 0:            # leaving SENSOR mode: hold still
                    s.dc_mode, s.dc_target_rpm = "stop", 0.0
                    s.servo_cmd_deg = s.servo_deg
                return None
            if s.mode == 1 and parts[0] in ("S", "D", "P"):
                self.ignored_count += 1
                return f"# ignored in SENSOR mode: {line.strip()}"
            if parts[0] == "S" and len(parts) == 2:
                s.servo_cmd_deg = clamp(float(parts[1]), SERVO_MIN_DEG, SERVO_MAX_DEG)
                return None
            if parts[0] == "D" and len(parts) == 3 and parts[1] == "P":
                s.dc_mode, s.dc_target_deg = "pos", s.dc_deg + float(parts[2])
                return None
            if parts[0] == "D" and len(parts) == 3 and parts[1] == "V":
                s.dc_mode, s.dc_target_rpm = "vel", float(parts[2])
                return None
            if parts[0] == "D" and len(parts) == 2 and parts[1] == "X":
                s.dc_mode, s.dc_target_rpm = "stop", 0.0
                return None
            if parts[0] == "P" and len(parts) == 2:
                s.step_target += round(float(parts[1]) / STEPPER_DEG_PER_STEP)
                return None
        except ValueError:
            pass
        self.ignored_count += 1
        return f"# bad command: {line.strip()}"

    def telemetry(self) -> str:
        s = self.s
        ms = int((time.monotonic() - self.t0) * 1000)
        fields = [
            "T", ms, s.mode,
            s.flex_raw, f"{s.flex_deg:.1f}", f"{s.flex_filt_deg:.1f}",
            int(s.tof_raw_mm), int(round(s.tof_filt_mm)),
            s.th_raw, f"{s.temp_c:.1f}",
            s.slot_count, f"{s.slot_rpm:.1f}",
            f"{s.servo_deg:.1f}", f"{s.dc_deg:.1f}", f"{s.dc_rpm:.1f}", s.dc_pwm,
            f"{s.step_pos * STEPPER_DEG_PER_STEP:.3f}",
            f"{s.step_target * STEPPER_DEG_PER_STEP:.3f}",
            int(s.interlock),
        ]
        line = ",".join(str(f) for f in fields)
        if self.bad_rate and self.rng.random() < self.bad_rate:
            line = self._corrupt(line)
        return line

    def _corrupt(self, line: str) -> str:
        """Malformed lines the bridge must survive: cut lines, noise, bad numbers."""
        kind = self.rng.randrange(4)
        if kind == 0:
            return line[: self.rng.randrange(2, len(line) - 1)]        # truncated
        if kind == 1:
            return "".join(chr(self.rng.randrange(33, 127)) for _ in range(20))
        if kind == 2:
            return line.replace(",", ",x", 1)                          # non-numeric
        return line + ",99"                                            # extra field


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

    def send(text: str) -> None:
        try:
            os.write(master, (text + "\r\n").encode())   # Serial.println sends CRLF
        except OSError:
            pass          # nobody reading yet; drop like a real Uno would
        if args.print:
            print(text, flush=True)

    send("# tsp05 fake arduino")
    rx = b""
    sim_dt, tel_dt = 1.0 / SIM_HZ, 1.0 / TELEMETRY_HZ
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
                cmd = raw.decode(errors="replace").strip()
                if not cmd:
                    continue
                if args.print:
                    print(f"> {cmd}", flush=True)
                reply = uno.handle(cmd)
                if reply:
                    send(reply)
            rx = rx[-256:]       # a runaway line without newline cannot grow forever
        now = time.monotonic()
        if now >= next_sim:
            uno.step(sim_dt)
            next_sim += sim_dt
            if now - next_sim > 0.5:          # fell far behind (laptop slept): resync
                next_sim = now + sim_dt
        if now >= next_tel:
            send(uno.telemetry())
            next_tel += tel_dt
            if now - next_tel > 0.5:
                next_tel = now + tel_dt


if __name__ == "__main__":
    main()
