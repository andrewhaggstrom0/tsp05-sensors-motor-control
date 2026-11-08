# TSP05 Controls (Foxglove extension panel)

One panel that sends every TSP05 command by clicking: GUI/SENSOR mode, servo
slider with 0/90/180 presets, DC move by N degrees and hold RPM, stepper -/+
N degrees, and a STOP button that works in both modes. It also shows the live
servo, DC and stepper values, the interlock state, and whether the Uno is
connected.

It publishes on `/tsp05/cmd/*` through foxglove_bridge; the `tsp05_bridge`
node turns those into serial commands (see `docs/protocol.md`).

## Build and install (Node 18+, any OS)

    npm ci
    npm test             # command logic unit tests
    npm run local-install   # installs into the Foxglove desktop app on this machine
    npm run package         # or build tsp05.tsp05-control-panel-0.1.0.foxe to drag into Foxglove

Then in Foxglove: connect to `ws://localhost:8765`, and add the panel
**TSP05 Controls** (Add panel, search "TSP05").

Motor buttons are disabled while the Uno is disconnected or in SENSOR mode
(the firmware ignores motor commands then). STOP in SENSOR mode switches to
GUI mode first, then stops the DC motor.
