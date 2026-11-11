"""ROS 2 node: TSP05 Uno serial <-> ROS 2 topics.

The Uno cannot run ROS 2, so this node translates between its serial
protocol (docs/protocol.md, parsed by tsp05_bridge.protocol) and topics that
Foxglove can plot and publish to.

    ros2 run tsp05_bridge bridge --ros-args -p port:=/dev/ttyACM0
    ros2 run tsp05_bridge bridge --ros-args -p port:=/tmp/tsp05      # fake Arduino
    ros2 run tsp05_bridge bridge --ros-args -p port:=socket://host.docker.internal:7000

Parameters:
    port               serial device or any pyserial URL     (/dev/ttyACM0)
    baud               baud rate                              (115200)
    reconnect_period   seconds between reopen attempts        (1.0)
    stats_period       seconds between rate/health log lines  (10.0)

Never crashes on bad input: malformed telemetry lines are counted and
skipped, and a lost port (cable pulled, Uno reset) is reopened automatically.
"""
from __future__ import annotations

import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool, Empty, Float32, Int8, String

from tsp05_bridge import protocol as p
from tsp05_bridge.feedback import Feedback, FeedbackConfig
from tsp05_bridge.topics import COMMAND_TOPICS, FEEDBACK_TOPICS, FLAG_TOPICS, FLOAT_TOPICS, NS

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - reported at startup
    serial = None

# mode is std_msgs/Int8 and interlock std_msgs/Bool; every other field is Float32
SPECIAL_TOPICS = {"mode": (FLAG_TOPICS["mode"], Int8), "interlock": (FLAG_TOPICS["interlock"], Bool)}
assert set(FLOAT_TOPICS) | set(SPECIAL_TOPICS) == set(p.TELEMETRY_FIELDS)


class BridgeNode(Node):
    def __init__(self, **kwargs) -> None:
        super().__init__("tsp05_bridge", **kwargs)
        self.port = self.declare_parameter("port", "/dev/ttyACM0").value
        self.baud = int(self.declare_parameter("baud", 115200).value)
        self.reconnect_period = float(self.declare_parameter("reconnect_period", 1.0).value)
        stats_period = float(self.declare_parameter("stats_period", 10.0).value)

        self._pubs = {f: self.create_publisher(Float32, f"{NS}/{t}", 10) for f, t in FLOAT_TOPICS.items()}
        for field, (topic, msg_type) in SPECIAL_TOPICS.items():
            self._pubs[field] = self.create_publisher(msg_type, f"{NS}/{topic}", 10)
        self._pub_connected = self.create_publisher(Bool, f"{NS}/bridge/connected", 10)
        self._pub_log = self.create_publisher(String, f"{NS}/log", 10)

        # Feedback worked out from commands + telemetry (tsp05_bridge.feedback).
        # Thresholds are parameters whose defaults mirror firmware/tsp05/config.h.
        cfg = FeedbackConfig()
        for name, default in vars(FeedbackConfig()).items():
            value = self.declare_parameter(name, default).value
            setattr(cfg, name, type(default)(value))
        self._fb = Feedback(cfg)
        self._fb_lock = threading.Lock()     # reader thread (telemetry) vs executor (commands)
        fb_types = {"dc/mode": Int8, "dc/target": Float32, "dc/arrived": Bool, "dc/settled": Bool,
                    "tof/fault": Bool, "cmd/status": String, "bridge/rate_hz": Float32,
                    "bridge/line_age_ms": Float32, "interlock/threshold_c": Float32}
        assert set(fb_types) == set(FEEDBACK_TOPICS)
        self._fb_pubs = {t: self.create_publisher(m, f"{NS}/{t}", 10) for t, m in fb_types.items()}

        # cmd topic -> (msg type, function from msg to command string)
        commands = {
            "cmd/mode": (Int8, lambda m: p.cmd_mode(int(m.data))),
            "cmd/servo_deg": (Float32, lambda m: p.cmd_servo_deg(m.data)),
            "cmd/dc_move_deg": (Float32, lambda m: p.cmd_dc_move_deg(m.data)),
            "cmd/dc_rpm": (Float32, lambda m: p.cmd_dc_rpm(m.data)),
            "cmd/dc_stop": (Empty, lambda m: p.cmd_dc_stop()),
            "cmd/stepper_deg": (Float32, lambda m: p.cmd_stepper_deg(m.data)),
        }
        assert set(commands) == set(COMMAND_TOPICS)
        for topic, (msg_type, build) in commands.items():
            self.create_subscription(msg_type, f"{NS}/{topic}",
                                     lambda msg, b=build, t=topic: self._on_command(t, b, msg), 10)

        self._ser = None
        self._ser_lock = threading.Lock()      # write (executor) vs reopen (reader thread)
        self._stop = threading.Event()
        self._n_lines = self._n_bad = self._n_cmds = 0
        self._last_bad = ""
        self._stats_t = time.monotonic()
        self.create_timer(stats_period, self._log_stats)
        self.create_timer(1.0, self._publish_connected)
        self.create_timer(0.5, self._publish_health)

        if serial is None:
            self.get_logger().fatal("pyserial is not installed (apt install python3-serial)")
            raise SystemExit(1)
        self._reader = threading.Thread(target=self._read_loop, daemon=True)
        self._reader.start()
        self.get_logger().info(f"bridge on {self.port} @ {self.baud}")

    # ------------------------------------------------------------ serial
    def _open(self) -> bool:
        try:
            ser = serial.serial_for_url(self.port, baudrate=self.baud, timeout=0.5)
        except (serial.SerialException, OSError, ValueError) as e:
            self.get_logger().warn(f"cannot open {self.port}: {e}; retrying",
                                   throttle_duration_sec=10.0)
            return False
        with self._ser_lock:
            self._ser = ser
        self.get_logger().info(f"connected to {self.port}")
        return True

    def _close(self) -> None:
        with self._ser_lock:
            ser, self._ser = self._ser, None
        if ser is not None:
            try:
                ser.close()
            except Exception:  # noqa: BLE001 - closing a dead port can fail any way
                pass

    def _read_loop(self) -> None:
        """Background thread: read lines continuously so data never piles up
        in OS/driver buffers (stale samples), and reopen the port if it drops."""
        while not self._stop.is_set():
            if self._ser is None and not self._open():
                self._stop.wait(self.reconnect_period)
                continue
            try:
                raw = self._ser.readline()
            except (serial.SerialException, OSError, TypeError, AttributeError) as e:
                # TypeError/AttributeError: pyserial internals after a hang-up
                self.get_logger().warn(f"lost {self.port}: {e}; reconnecting")
                self._close()
                continue
            if raw:
                self._handle_line(raw.decode("ascii", errors="replace").strip())

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        if not p.is_telemetry(line):
            if line.startswith("#"):
                self._pub_log.publish(String(data=line[1:].strip()))
                self.get_logger().info(f"uno: {line[1:].strip()}")
            return                       # noise, e.g. a half line after a reset
        try:
            t = p.parse_telemetry(line)
        except p.ParseError as e:
            self._n_bad += 1
            self._last_bad = f"{e} | {line[:60]!r}"
            return
        self._n_lines += 1
        for field, value in t.items():
            if field == "mode":
                self._pubs[field].publish(Int8(data=int(value)))
            elif field == "interlock":
                self._pubs[field].publish(Bool(data=bool(value)))
            else:
                self._pubs[field].publish(Float32(data=value))
        with self._fb_lock:
            self._fb.on_telemetry(t, time.monotonic())
            fb = self._fb
            self._fb_pubs["dc/mode"].publish(Int8(data=fb.dc_mode))
            self._fb_pubs["dc/target"].publish(Float32(data=float(fb.dc_target)))
            self._fb_pubs["dc/arrived"].publish(Bool(data=fb.dc_arrived))
            self._fb_pubs["dc/settled"].publish(Bool(data=fb.dc_settled))
            self._fb_pubs["tof/fault"].publish(Bool(data=fb.tof_fault))
            status = fb.take_status()
        self._publish_status(status)

    # ------------------------------------------------------------ commands
    def _on_command(self, topic: str, build, msg) -> None:
        try:
            cmd = build(msg)
        except ValueError as e:
            self.get_logger().warn(f"{topic}: rejected: {e}")
            return
        with self._ser_lock:
            ser = self._ser
            if ser is None:
                self.get_logger().warn(f"{topic}: not connected, dropped {cmd}",
                                       throttle_duration_sec=2.0)
                return
            try:
                ser.write(p.encode(cmd))
            except (serial.SerialException, OSError) as e:
                self.get_logger().warn(f"{topic}: write failed: {e}")
                return
        self._n_cmds += 1
        self.get_logger().info(f"sent {cmd}")
        with self._fb_lock:
            self._fb.on_command(cmd, time.monotonic())
            status = self._fb.take_status()
        self._publish_status(status)

    # ------------------------------------------------------------ feedback
    def _publish_status(self, lines) -> None:
        for line in lines:
            self._fb_pubs["cmd/status"].publish(String(data=line))
            self.get_logger().info(f"status: {line}")

    def _publish_health(self) -> None:
        now = time.monotonic()
        with self._fb_lock:
            self._fb.tick(now)
            status = self._fb.take_status()
            rate = self._fb.rate_hz(now)
            age = self._fb.line_age_s(now)
            threshold = self._fb.cfg.interlock_temp_c
        self._publish_status(status)
        self._fb_pubs["bridge/rate_hz"].publish(Float32(data=float(rate)))
        if age is not None:
            self._fb_pubs["bridge/line_age_ms"].publish(Float32(data=float(age * 1000.0)))
        self._fb_pubs["interlock/threshold_c"].publish(Float32(data=float(threshold)))

    # ------------------------------------------------------------ health
    def _publish_connected(self) -> None:
        self._pub_connected.publish(Bool(data=self._ser is not None))

    def _log_stats(self) -> None:
        now = time.monotonic()
        dt, self._stats_t = now - self._stats_t, now
        n, bad = self._n_lines, self._n_bad
        self._n_lines = self._n_bad = 0
        msg = f"telemetry {n / dt:.1f} Hz, {bad} malformed, {self._n_cmds} commands sent"
        if bad:
            self.get_logger().warn(f"{msg}; last bad: {self._last_bad}")
        elif self._ser is not None and n == 0:
            self.get_logger().warn(f"{msg}: port open but no telemetry (firmware running? baud?)")
        else:
            self.get_logger().info(msg)

    def destroy_node(self) -> None:
        self._stop.set()
        self._close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
