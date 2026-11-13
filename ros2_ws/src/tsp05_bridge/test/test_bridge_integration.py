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
        self.last, self.count, self.logs, self.status = {}, {}, [], []
        topics = {t: Float32 for t in FLOAT_TOPICS.values()}
        topics.update({t: m for t, m in SPECIAL_TOPICS.values()})
        topics["bridge/connected"] = Bool
        topics.update({"dc/mode": Int8, "dc/target": Float32, "dc/target_deg": Float32,
                       "dc/target_rpm": Float32, "dc/arrived": Bool, "dc/settled": Bool,
                       "tof/fault": Bool, "bridge/rate_hz": Float32, "bridge/line_age_ms": Float32,
                       "interlock/threshold_c": Float32})
        for topic, msg_type in topics.items():
            self.probe.create_subscription(msg_type, f"{NS}/{topic}",
                                           lambda m, t=topic: self._rx(t, m.data), 50)
        self.probe.create_subscription(String, f"{NS}/log", lambda m: self.logs.append(m.data), 50)
        self.probe.create_subscription(String, f"{NS}/cmd/status", lambda m: self.status.append(m.data), 50)
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

    # Speed comes from 2 deg encoder steps through an EMA, so single readings scatter
    # by a few rpm (about +-2 at 30 rpm on hardware). Check the mean over 1 s.
    h.send("dc_rpm", Float32, -50.0)
    assert h.wait(lambda: abs(h.last["dc/rpm"] + 50.0) < 10.0)
    samples = []
    for _ in range(20):
        h.spin(0.05)
        samples.append(h.last["dc/rpm"])
    assert sum(samples) / len(samples) == pytest.approx(-50.0, abs=3.0), samples
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
    # The firmware ignores motor commands in SENSOR mode without replying.
    h.send("servo_deg", Float32, 170.0)
    assert not h.wait(lambda: h.last["servo/deg"] == 170.0, 1.0), "servo obeyed S in SENSOR mode"
    h.send("mode", Int8, 0)
    assert h.wait(lambda: h.last.get("mode") == 0)
    h.send("servo_deg", Float32, 170.0)
    assert h.wait(lambda: h.last["servo/deg"] == 170.0), "servo ignored S in GUI mode"


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


# ---------------------------------------------------------------- feedback topics
def test_feedback_topics_follow_a_dc_move(env):
    h, _ = env
    assert h.wait(lambda: {"dc/mode", "dc/target", "dc/settled", "bridge/rate_hz"} <= set(h.last))
    assert h.last["dc/mode"] == 0
    start = h.last["dc/deg"]
    h.send("dc_move_deg", Float32, 90.0)
    assert h.wait(lambda: h.last["dc/mode"] == 2 and h.last["dc/target"] == pytest.approx(start + 90.0, abs=1.0))
    assert h.wait(lambda: h.last["dc/settled"] is True and h.last["dc/arrived"] is True, 3.0)
    assert abs(h.last["dc/deg"] - h.last["dc/target"]) <= 2.0
    assert h.last["dc/target_deg"] == h.last["dc/target"] and math.isnan(h.last["dc/target_rpm"])
    assert h.wait(lambda: any(x.startswith("D,P,90.0: applied in ") for x in h.status)), h.status
    h.send("dc_rpm", Float32, -45.0)
    assert h.wait(lambda: h.last["dc/mode"] == 1 and h.last["dc/target"] == pytest.approx(-45.0)
                  and h.last["dc/settled"] is False)
    assert h.wait(lambda: h.last["dc/target_rpm"] == pytest.approx(-45.0) and math.isnan(h.last["dc/target_deg"]))
    h.send("dc_stop", Empty)
    assert h.wait(lambda: h.last["dc/mode"] == 0 and h.last["dc/target"] == 0.0)


def test_feedback_reports_ignored_commands_and_health(env):
    h, _ = env
    assert h.wait(lambda: h.last.get("mode") == 0)
    h.send("mode", Int8, 1)
    assert h.wait(lambda: h.last.get("mode") == 1)
    h.send("stepper_deg", Float32, 90.0)
    assert h.wait(lambda: "P,90.0: ignored (SENSOR mode)" in h.status), h.status
    assert h.wait(lambda: 17.0 <= h.last.get("bridge/rate_hz", 0) <= 23.0, 4.0), h.last.get("bridge/rate_hz")
    assert h.last["bridge/line_age_ms"] < 200
    assert h.last["interlock/threshold_c"] == 29.0


def test_feedback_tof_fault_follows_the_hand(env):
    """The fake's hand passes the TOF at 2.0-2.4 s of every 10 s."""
    h, _ = env
    seen = []
    end = time.monotonic() + 12.0
    while time.monotonic() < end and [v for v in seen][-3:] != [True, False, True]:
        h.ex.spin_once(timeout_sec=0.02)
        if "tof/fault" in h.last and (not seen or seen[-1] != h.last["tof/fault"]):
            seen.append(h.last["tof/fault"])
    assert seen[-3:] == [True, False, True], seen


def test_feedback_also_works_with_older_firmware():
    """19-field telemetry (firmware before dc_mode..tof_fault): the same topics,
    worked out by the bridge, and nothing counted as malformed."""
    init_isolated()
    link = os.path.join(tempfile.mkdtemp(), "tsp05")
    fake = Fake(link, "--legacy-telemetry")
    fake.start()
    h = Harness(link)
    try:
        assert h.wait(lambda: {"dc/mode", "dc/target", "tof/fault"} <= set(h.last))
        start = h.last["dc/deg"]
        h.send("dc_move_deg", Float32, 90.0)
        assert h.wait(lambda: h.last["dc/mode"] == 2 and h.last["dc/target"] == pytest.approx(start + 90.0, abs=1.0))
        assert h.wait(lambda: h.last["dc/settled"] is True, 3.0)
        assert h.bridge._n_bad == 0 and h.bridge._fb.source == "inferred"
    finally:
        h.close()
        fake.stop()
        rclpy.shutdown()
