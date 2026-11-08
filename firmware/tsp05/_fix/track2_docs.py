import re, pathlib
# ---- docs/pinmap.md: measured stepper coil pairs ----
p = pathlib.Path("docs/pinmap.md"); s = p.read_text()
old = "## Stepper wiring (corrected colors)\nA = red, C = blue, B = black, D = green\n"
new = """## Stepper wiring (measured 2026-11-08, Track 2)
Coil pairs measured with a multimeter (12 V off, motor unplugged):
- Coil 1: red - green (35.9 ohm) -> EasyDriver A
- Coil 2: blue - black (~36 ohm) -> EasyDriver B
- Wires from different coils read open.

The earlier note (A = red, C = blue, B = black, D = green) paired red with blue;
on our motor that made it wiggle in place instead of turning.
To reverse direction, swap the two wires of ONE coil (power off).
"""
assert old in s, "pinmap section not found"
p.write_text(s.replace(old, new))

# ---- README.md ----
p = pathlib.Path("README.md"); s = p.read_text()

running = """<!-- Build/flash steps: Track 2 -->
### Flashing the firmware (Arduino Uno)
1. Install Arduino IDE 2 and the **Servo** library (Library Manager).
2. Open `firmware/tsp05/tsp05.ino`, board **Arduino Uno**, select the port
   (Linux: add yourself to `dialout` once: `sudo usermod -aG dialout $USER`, then log out/in).
3. Verify, then Upload. Open the Serial Monitor at **115200 baud, Newline**.
4. You should see `# TSP05 starting`, then a `T,...` line at 20 Hz (format in `docs/protocol.md`).
5. Quick test by hand: `S,90` (servo), `P,90` (stepper), `M,1` / `M,0` (mode).
   Motor commands are ignored in SENSOR mode (`M,1`).

Power: servo on its own 5-6 V supply, EasyDriver on current-limited 12 V,
all grounds tied to the Uno GND. Never plug or unplug the stepper while 12 V is on.
"""
assert "<!-- Build/flash steps: Track 2 -->\n" in s
s = s.replace("<!-- Build/flash steps: Track 2 -->\n", running, 1)

contrib = """### Yashasvi (Track 2: servo, stepper, button, firmware integration)
- **Servo** (`actuators.cpp`): found the real limits with `firmware/bringup/servo`:
  strains at 620 us, stalls at 2400 us; safe travel 640-2380 us = 183 deg (~9.5 us/deg).
  `SERVO_MIN_US`/`MAX_US` = 640/2351 so 0-180 deg commanded = 0-180 deg physical.
  Out-of-range requests are clamped.
- **Stepper** (`actuators.cpp`): non-blocking, one STEP pulse per loop pass when due
  (`micros()` timer), so the button, serial and telemetry keep running during moves.
  EasyDriver at 1/8 microstep = 1600 steps/rev, verified at +45, +90, -45 and 360 deg.
  Current set with the motor unplugged: TP1 = 1.70 V (pot minimum), ~0.28 A/phase,
  under the motor's ~0.33 A rating.
- **Button** (`button.cpp`): 30 ms debounce, toggles only on the confirmed press edge.
  50-press test: ___ / 50.
- **Integration** (`tsp05.ino`): command parser for every command in `docs/protocol.md`,
  GUI/SENSOR mode, SENSOR mappings (TOF -> servo, flex bend -> DC rpm with a dead zone,
  slot event -> fixed stepper jog), thermal interlock in both modes, and the 19-field
  `T` line at 20 Hz. Telemetry is written only as fast as the 64-byte TX buffer has
  room, so printing never stalls the loop. Ranges live in `config.h`.
- RAM use: ___ % of 2 KB.

"""
assert "## Challenges" in s
s = s.replace("## Challenges", contrib + "## Challenges", 1)

challenge = """
### Yashasvi
- **Stepper only wiggled in place.** Symptom: on `P,45` the shaft rocked back and forth;
  the bench supply sagged from 12 V to 9.4 V. Two causes: (1) the supply current limit
  was too low (raised to 0.33 A+, voltage then held at 12 V, still wiggled);
  (2) wrong coil pairs. Evidence: multimeter read 35.9 ohm red-green and open
  red-blue / red-black, so red's partner is green, not blue as the wiring note said.
  Fix: red+green on A, blue+black on B; the motor then turned 45/90/-45/360 deg
  correctly. Updated `docs/pinmap.md`.
"""
m = re.search(r"## Challenges\n(<!--.*?-->\n)?", s)
s = s[:m.end()] + challenge + s[m.end():]
p.write_text(s)
print("pinmap.md and README.md updated")
