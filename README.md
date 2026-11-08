# TSP05: Sensors & Motor Control

16-650 Systems Engineering. An Arduino Uno reads four sensors and drives three motors, with state and control exposed through ROS 2 and a Foxglove GUI.

## System Overview
<!-- Block diagram: docs/figures/block_diagram.png -->

| Sensor | Physical output | Controls |
|---|---|---|
| VL53L1X TOF | Distance (mm) | Servo angle |
| Flex/bend sensor (Adafruit 182) | Bend angle (deg) | DC motor velocity |
| Omron EE-SX1042 slot sensor | Event count, RPM | Stepper jog per event |
| 10k thermistor | Temperature (degC) | DC motor thermal interlock |

Control mode (GUI or SENSOR) is toggled by the debounced pushbutton or from the GUI.

## Hardware
- [Pin map and power](docs/pinmap.md)
- Wiring diagram: <!-- docs/figures/wiring.png -->

## Software
- Firmware: `firmware/tsp05/`
- ROS 2 bridge: `ros2_ws/src/tsp05_bridge/` (serial <-> `/tsp05/*` topics, launch file, tests)
- Foxglove layout: `foxglove/tsp05_layout.json`; command panel: `foxglove/tsp05-control-panel/`
- Fake Arduino (no hardware needed): `tools/fake_arduino.py`
- ROS 2 dev container: `docker/`
- [Serial protocol](docs/protocol.md)

![Track 5 architecture](docs/figures/bridge_architecture.png)

## Running
<!-- Build/flash steps: Track 2 -->

### ROS 2 bridge + Foxglove
The laptop side runs either in Docker (any OS) or natively on Ubuntu with ROS 2
Humble or Jazzy. Both use the same code.

**Docker** (macOS, Windows, or Linux; needs Docker Desktop / Docker Engine):

    docker/run.sh ros2 launch tsp05_bridge tsp05.launch.py fake:=true   # no hardware
    docker/test.sh                                                     # build + all tests
    ROS_DISTRO=humble docker/run.sh                                    # Humble image instead of Jazzy

With the real Uno on a Linux host, pass the device in:

    TSP05_SERIAL_DEVICE=/dev/ttyACM0 docker/run.sh ros2 launch tsp05_bridge tsp05.launch.py port:=/dev/ttyACM0

Docker Desktop on macOS cannot pass USB devices to a container. Share the port
over TCP on the Mac instead (`brew install socat`; the device name varies), then
point the bridge at it:

    socat TCP-LISTEN:7000,reuseaddr FILE:/dev/cu.usbmodem1101,b115200,raw,echo=0
    docker/run.sh ros2 launch tsp05_bridge tsp05.launch.py port:=socket://host.docker.internal:7000

**Native Ubuntu** (ROS 2 sourced):

    sudo apt install ros-$ROS_DISTRO-foxglove-bridge python3-serial
    sudo usermod -aG dialout $USER        # serial port access; log out and back in
    cd ros2_ws && colcon build && source install/setup.bash
    ros2 launch tsp05_bridge tsp05.launch.py port:=/dev/ttyACM0     # or fake:=true
    colcon test && colcon test-result --verbose

**Foxglove**
1. Foxglove desktop app: Open connection -> Foxglove WebSocket -> `ws://localhost:8765`.
2. Layout -> Import from file -> `foxglove/tsp05_layout.json` (plots and indicators).
3. Command panel, once per machine (Node 18+):
   `cd foxglove/tsp05-control-panel && npm ci && npm run local-install`,
   restart Foxglove, then Add panel -> "TSP05 Controls".

Launch arguments: `port`, `baud` (115200), `fake`, `fake_link` (`/tmp/tsp05`),
`foxglove`, `foxglove_port` (8765). Topics are listed in
`ros2_ws/src/tsp05_bridge/tsp05_bridge/topics.py`; they follow the team guide
with `fsr/*` renamed to `flex/*`.

## Calibration Results
<!-- Transfer function plot, filtering plot, error table per sensor -->

## Individual Contributions
### <Name 1>
### <Name 2>
### Harish Bhamitipadi Magesh (Track 3: TOF + slot sensor)
- **TOF (VL53L1X, I2C)** (`firmware/tsp05/sensors_digital.cpp`): continuous ranging in
  short mode, 33 ms timing budget, 40 ms inter-measurement period (the sensor's 100 ms
  default would cap it at 10 Hz). Polled every 5 ms and read only when a range is ready,
  so `loop()` never waits. I2C at 400 kHz with a Wire timeout, so a glitched bus cannot
  hang the loop. Only status-0 ranges are used; `tofFault()` is raised after 500 ms
  without one.
- **Filter**: median of 5 (removes single-sample outliers), then EMA with alpha = 0.3.
  Lag cost: the median delays a step by about 80 ms, and the EMA adds a 112 ms time constant.
- **Slot sensor (EE-SX1042, D3 / INT1)**: RISING-edge interrupt with a 1 ms glitch
  filter; edges are time-stamped into a ring buffer. `slotTakeEvents()` hands each event
  to the stepper jog exactly once.
- **RPM**: period method over one full revolution (4 tabs). Counting events in a 100 ms
  window would resolve only 150 RPM per count. Spanning a whole revolution also cancels
  unevenly cut tabs. RPM decays smoothly when the disc slows and reads 0 after 2 s.
- **Integration**: the full `tsp05.ino` with this module uses 52 % flash and 45 % RAM
  (team limit 75 %).
- **Tools and tests**: bring-up sketch (`firmware/bringup/digital/`);
  `tools/tof_calibration.py` (error logging, raw-vs-filtered recording, error table and
  plots, `--sim` rehearsal); host tests (`firmware/tests/test_sensors_digital.cpp`,
  37 checks) for the filter, faults, glitch filter, and RPM with uneven tabs.
- **Not yet verified on hardware**: TOF error table, raw-vs-filtered plot, 50-pass
  slot count test, RPM against the stepper at a known 30 RPM.
### <Name 4>
### Raoul Bouri (Track 5: ROS 2 bridge, Foxglove GUI, fake Arduino)
- **Fake Arduino** (`tools/fake_arduino.py`): a pty that speaks `docs/protocol.md`,
  sending the 19-field `T` line at 20 Hz and obeying every command. It simulates the
  sensors (raw ADC through Track 4's `analog_model.py`), servo slew, a DC motor with a
  position loop (+-2 deg arrival band), a 0.225 deg/step stepper, the SENSOR-mode
  mapping, and the 30 C / 28 C interlock. Bridge and GUI were built and tested before
  any firmware existed.
- **`tsp05_bridge` node**: 26 published topics, 6 command topics, port/baud as
  parameters (device path or `socket://`). A background reader thread
  auto-reconnects after a cable pull; malformed lines are counted and skipped.
  Measured: 19.99 Hz on every topic; command -> telemetry round trip median 31 ms,
  p95 52 ms (n = 200), mostly waiting for the next 50 ms telemetry line.
- **Foxglove**: layout with the graded raw-vs-filtered plots (TOF, flex), DC RPM
  measured vs commanded, stepper actual vs target, and mode/interlock/connection
  indicators. A custom extension panel ("TSP05 Controls") sends every command by
  clicking; STOP also works in SENSOR mode (sends `M,0` then `D,X`).
- **Docker dev environment** (`docker/`): `ros:jazzy` (or Humble) image so the
  laptop side runs the same on macOS and Linux.
- **Tests**: 48 pytest tests (`docker/test.sh`) plus 8 for the panel logic (`npm test`).
  They cover the parser against `protocol.md` itself, 5000 fuzzed lines, the live bridge
  against the fake Arduino (rates, every command, reconnect, 30 % corrupted lines), and
  checks that every topic in the layout and the panel exists in the bridge.
- Figure: [docs/figures/bridge_architecture.png](docs/figures/bridge_architecture.png)
  (`python3 tools/draw_bridge_diagram.py`).

## Challenges
<!-- Per person: problem, analysis of cause, resolution -->

### Harish Bhamitipadi Magesh
- **RPM read low with unevenly cut tabs.** The first version of the slow-down limit
  (so RPM decays instead of holding until the 2 s timeout) compared the gap in progress
  with the average tab gap. With alternating 200 / 300 ms gaps (a steady 60 RPM) it read
  as low as 55.6 RPM during every long gap. Cause: a gap that is normally long for that
  tab looked like the disc slowing down. Fix: compare the gap in progress with the same
  gap one revolution earlier. A host test now holds steady uneven-tab RPM within
  60.0 +- 0.5.
- Hardware challenges to follow.

### Raoul Bouri
- **The fake Arduino stalled when nothing read the port.** A `D,P,90` measured after
  a pause reported 86.2 deg with the motor still turning. Cause: a macOS pty holds only
  a few KB (2-3 s of telemetry). With no reader, `os.write()` blocked and froze the
  simulation. A real Uno never waits for the host; its TX simply drops bytes. Fix:
  non-blocking writes that drop a line when the buffer is full. After: `D,P,90` lands
  at 91.0 deg, and the simulation keeps running with no reader.
- **The GUI could publish on any topic.** I set foxglove_bridge's
  `client_topic_whitelist` to `/tsp05/cmd/.*`, but foxglove_bridge 3.6 (Jazzy) still
  accepted a client advertising `/tsp05/servo/deg`; its own log showed it. I removed
  the setting rather than ship something that looks like protection but isn't. The
  remaining risk is small by design: the bridge acts only on `/tsp05/cmd/*`, so a
  stray GUI publish can confuse a plot but cannot move a motor.
- **Measuring latency honestly.** My first run reported a 11 ms median, which was
  too good for 20 Hz telemetry. Each command went out a fixed 137 ms after the
  previous reply, so it always landed at the same point in the 50 ms telemetry
  period. Randomizing the timing gave 31 ms median / 52 ms p95. That matches the
  expected average wait for the next line (25 ms) plus about 6 ms of processing.
- **Not yet verified on hardware**: the real Uno (its reset when the port opens is
  handled by skipping non-`T` lines), the Mac `socat` route across machines (tested
  only inside one container), and the control panel inside the Foxglove app (tested
  in a browser harness that mimics Foxglove's panel API).
