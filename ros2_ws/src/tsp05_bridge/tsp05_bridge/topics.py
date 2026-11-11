"""ROS 2 topic names used by the bridge (no ROS import, so tools and tests
such as the Foxglove layout check can use them on any machine)."""

NS = "/tsp05"

# telemetry field -> topic, published as std_msgs/Float32
FLOAT_TOPICS = {
    "tof_raw_mm": "tof/raw_mm",
    "tof_filt_mm": "tof/filt_mm",
    "flex_raw": "flex/raw",
    "flex_deg": "flex/deg",
    "flex_filt_deg": "flex/deg_filt",
    "th_raw": "thermistor/raw",
    "temp_C": "thermistor/temp_c",
    "slot_count": "slot/count",
    "slot_rpm": "slot/rpm",
    "servo_deg": "servo/deg",
    "dc_deg": "dc/deg",
    "dc_rpm": "dc/rpm",
    "dc_pwm": "dc/pwm",
    "step_deg": "stepper/deg",
    "step_target_deg": "stepper/target_deg",
    "ms": "uno_ms",
}
# telemetry field -> topic with its own type: mode is std_msgs/Int8, interlock std_msgs/Bool
FLAG_TOPICS = {"mode": "mode", "interlock": "interlock"}
# Feedback worked out by the bridge (tsp05_bridge.feedback) from the commands it
# sends and the telemetry. dc/mode Int8; dc/target, bridge/rate_hz,
# bridge/line_age_ms, interlock/threshold_c Float32; dc/arrived, dc/settled,
# tof/fault Bool; cmd/status String.
FEEDBACK_TOPICS = ("dc/mode", "dc/target", "dc/arrived", "dc/settled", "tof/fault",
                   "cmd/status", "bridge/rate_hz", "bridge/line_age_ms",
                   "interlock/threshold_c")
# published by the bridge itself: std_msgs/Bool, std_msgs/String
STATUS_TOPICS = ("bridge/connected", "log")
# subscribed: GUI -> Uno
COMMAND_TOPICS = ("cmd/mode", "cmd/servo_deg", "cmd/dc_move_deg", "cmd/dc_rpm",
                  "cmd/dc_stop", "cmd/stepper_deg")
