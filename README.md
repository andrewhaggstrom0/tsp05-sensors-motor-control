# TSP05: Sensors & Motor Control

16-650 Systems Engineering. An Arduino Uno reads four sensors and drives three motors, with state and control exposed through ROS 2 and a Foxglove GUI.

## System Overview
<!-- Block diagram: docs/figures/block_diagram.png -->

| Sensor | Physical output | Controls |
|---|---|---|
| VL53L1X TOF | Distance (mm) | Servo angle |
| Force-sensitive resistor | Force (N) | DC motor velocity |
| Omron EE-SX1042 slot sensor | Event count, RPM | Stepper jog per event |
| 10k thermistor | Temperature (degC) | DC motor thermal interlock |

Control mode (GUI or SENSOR) is toggled by the debounced pushbutton or from the GUI.

## Hardware
- [Pin map and power](docs/pinmap.md)
- Wiring diagram: <!-- docs/figures/wiring.png -->

## Software
- Firmware: `firmware/tsp05/`
- ROS 2 bridge: `ros2_ws/src/tsp05_bridge/`
- Foxglove layout: `foxglove/`
- [Serial protocol](docs/protocol.md)

## Running
<!-- Build/flash, ROS launch, Foxglove connect steps -->

## Calibration Results
<!-- Transfer function plot, filtering plot, error table per sensor -->

## Individual Contributions
### <Name 1>
### <Name 2>
### <Name 3>
### <Name 4>
### <Name 5>

## Challenges
<!-- Per person: problem, analysis of cause, resolution -->
