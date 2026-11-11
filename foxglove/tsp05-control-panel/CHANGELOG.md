# Changelog

## 0.2.0

- Feedback from the bridge: DC moving / arrived / not arrived after 3 s, per-command status (applied / ignored / no effect), telemetry rate, TOF "no target".
- DC presets: -360/-90/-10/+10/+90/+360 deg and -60/30/60/120 rpm.
- Servo slider follows the live angle when not dragging.
- SENSOR-mode mapping card (swipe -> servo sweep, flex -> rpm, slot -> stepper jog).
- The Uno only counts as connected while telemetry arrives (bridge health topics no longer keep it "live").

## 0.1.0

- First version: mode toggle, servo, DC move/RPM, stepper, STOP, live status.
