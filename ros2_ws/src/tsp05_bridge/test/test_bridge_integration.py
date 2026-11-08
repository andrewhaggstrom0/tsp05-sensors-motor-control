"""End-to-end test: tools/fake_arduino.py <-> BridgeNode <-> ROS 2 topics.

Needs ROS 2 (rclpy, std_msgs) and pyserial; skipped otherwise, so the
protocol tests still run with plain pytest on any machine.

    docker/test.sh                                   # in the container
    python3 -m pytest ros2_ws/src/tsp05_bridge/test  # native Ubuntu, ROS sourced
"""
import math
import os
import random
import subprocess
import sys
import tempfile
import time
from pathlib import Path

import pytest

rclpy = pytest.importorskip("rclpy")
pytest.importorskip("serial")
from rclpy.parameter import Parameter  # noqa: E402
from std_msgs.msg import Bool, Empty, Float32, Int8, String  # noqa: E402

PKG = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(PKG))
from tsp05_bridge.bridge_node import FLOAT_TOPICS, NS, SPECIAL_TOPICS, BridgeNode  # noqa: E402

FAKE = PKG.parent.parent.parent / "tools" / "fake_arduino.py"


def init_isolated():
    """Own ROS domain, so a bridge already running on this machine (domain 0 by
    default) cannot add its messages to the counts measured here."""
    rclpy.init(domain_id=random.randint(50, 99))
pytestmark = pytest.mark.skipif(not FAKE.exists(), reason="tools/fake_arduino.py not in this checkout")


class Fake:
    """Runs tools/fake_arduino.py behind a stable symlink."""

    def __init__(self, link: str, *args: str):
        self.link, self.args, self.proc = link, args, None

    def start(self):
        self.proc = subprocess.Popen(
            [sys.executable, str(FAKE), "--link", self.link, "--temp", "room", "--seed", "1", *self.args],
            stdout=subprocess.PIPE, text=True)
        assert "fake Arduino on" in self.proc.stdout.readline()

    def stop(self):
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            self.proc.wait(5)


class Harness:
    """Bridge node + a probe node that records every topic and can publish commands."""

    def __init__(self, port: str):
        self.bridge = BridgeNode(parameter_overrides=[
            Parameter("port", value=port), Parameter("reconnect_period", value=0.2)])
        self.probe = rclpy.create_node("probe")
        self.last, self.count, self.logs = {}, {}, []
        topics = {t: Float32 for t in FLOAT_TOPICS.values()}
        topics.update({t: m for t, m in SPECIAL_TOPICS.values()})
        topics["bridge/connected"] = Bool
        for topic, msg_type in topics.items():
            self.probe.create_subscription(msg_type, f"{NS}/{topic}",
                                           lambda m, t=topic: self._rx(t, m.data), 50)
        self.probe.create_subscription(String, f"{NS}/log", lambda m: self.logs.append(m.data), 50)
        self.pubs = {}
        self.ex = rclpy.executors.SingleThreadedExecutor()
        self.ex.add_node(self.bridge)
        self.ex.add_node(self.probe)

    def _rx(self, topic, value):
        self.last[topic] = value
        self.count[topic] = self.count.get(topic, 0) + 1

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)

    def wait(self, cond, timeout=3.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.ex.spin_once(timeout_sec=0.02)
            if cond():
                return True
        return False

    def send(self, topic, msg_type, value=None):
        if topic not in self.pubs:
            self.pubs[topic] = self.probe.create_publisher(msg_type, f"{NS}/cmd/{topic}", 10)
            self.spin(0.3)                     # let discovery match the subscription
        msg = msg_type() if value is None else msg_type(data=value)
        self.pubs[topic].publish(msg)

    def close(self):
        self.ex.shutdown()
        self.bridge.destroy_node()
        self.probe.destroy_node()


@pytest.fixture
def env():
    init_isolated()
    tmp = tempfile.mkdtemp()
    link = os.path.join(tmp, "tsp05")
    fake = Fake(link)
    fake.start()
    h = Harness(link)
    yield h, fake
    h.close()
    fake.stop()
    rclpy.shutdown()


def test_all_topics_published_at_20hz(env):
    h, _ = env
    assert h.wait(lambda: h.last.get("bridge/connected") is True), "bridge never connected"
    h.count.clear()
    h.spin(2.0)
    expected = set(FLOAT_TOPICS.values()) | {t for t, _ in SPECIAL_TOPICS.values()}
    assert expected <= set(h.count), f"missing: {expected - set(h.count)}"
    rate = h.count["servo/deg"] / 2.0
    assert 17 <= rate <= 23, f"telemetry rate {rate} Hz"


def test_gui_commands_move_motors(env):
    h, _ = env
    assert h.wait(lambda: "servo/deg" in h.last)
    h.send("servo_deg", Float32, 30.0)
    assert h.wait(lambda: abs(h.last["servo/deg"] - 30.0) < 0.5), h.last["servo/deg"]

    start = h.last["dc/deg"]
    h.send("dc_move_deg", Float32, 90.0)
    assert h.wait(lambda: abs(h.last["dc/deg"] - start - 90.0) <= 2.0 and h.last["dc/rpm"] == 0.0, 5.0)

    h.send("dc_rpm", Float32, -50.0)
    assert h.wait(lambda: abs(h.last["dc/rpm"] + 50.0) < 1.0)
    h.send("dc_stop", Empty)
    assert h.wait(lambda: h.last["dc/rpm"] == 0.0 and h.last["dc/pwm"] == 0.0)

    h.send("stepper_deg", Float32, 10.0)
    assert h.wait(lambda: h.last["stepper/target_deg"] == pytest.approx(9.9, abs=1e-3)
                  and h.last["stepper/deg"] == h.last["stepper/target_deg"])


def test_mode_switch_and_sensor_mode_ignores_motor_commands(env):
    h, _ = env
    assert h.wait(lambda: h.last.get("mode") == 0)
    h.send("mode", Int8, 1)
    assert h.wait(lambda: h.last.get("mode") == 1)
    h.send("servo_deg", Float32, 170.0)
    assert h.wait(lambda: any("ignored in SENSOR mode" in m for m in h.logs)), h.logs


def test_non_finite_command_rejected_without_crash(env):
    h, _ = env
    assert h.wait(lambda: "servo/deg" in h.last)
    h.send("servo_deg", Float32, 45.0)
    assert h.wait(lambda: abs(h.last["servo/deg"] - 45.0) < 0.5)
    h.send("servo_deg", Float32, math.nan)
    h.spin(0.5)
    assert abs(h.last["servo/deg"] - 45.0) < 0.5
    assert h.bridge._reader.is_alive()


def test_reconnects_after_port_loss(env):
    h, fake = env
    assert h.wait(lambda: h.last.get("bridge/connected") is True)
    fake.stop()                                   # like pulling the USB cable
    assert h.wait(lambda: h.last.get("bridge/connected") is False, 4.0)
    fake.start()
    h.count.clear()
    assert h.wait(lambda: h.count.get("servo/deg", 0) > 10, 6.0), "no telemetry after reconnect"


def test_survives_malformed_lines():
    init_isolated()
    link = os.path.join(tempfile.mkdtemp(), "tsp05")
    fake = Fake(link, "--bad-rate", "0.3")
    fake.start()
    h = Harness(link)
    try:
        h.spin(3.0)
        good = h.count.get("servo/deg", 0)
        assert h.bridge._reader.is_alive()
        assert h.bridge._n_bad > 0, "no malformed lines were seen"
        assert 30 <= good <= 50, f"{good} good lines in 3 s with 30 % corrupted"
    finally:
        h.close()
        fake.stop()
        rclpy.shutdown()
