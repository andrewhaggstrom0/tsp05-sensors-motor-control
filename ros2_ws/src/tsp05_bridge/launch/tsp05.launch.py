"""Start the TSP05 bridge and foxglove_bridge with one command.

    ros2 launch tsp05_bridge tsp05.launch.py                       # real Uno on /dev/ttyACM0
    ros2 launch tsp05_bridge tsp05.launch.py port:=/dev/ttyUSB0
    ros2 launch tsp05_bridge tsp05.launch.py fake:=true            # no hardware
    ros2 launch tsp05_bridge tsp05.launch.py foxglove:=false

Then in the Foxglove app: Open connection -> Foxglove WebSocket ->
ws://localhost:8765, and import foxglove/tsp05_layout.json.

Arguments:
    port            serial device or pyserial URL          (/dev/ttyACM0)
    baud            baud rate                              (115200)
    fake            also run tools/fake_arduino.py and use it (false)
    fake_link       pty symlink the fake Arduino creates   (/tmp/tsp05)
    foxglove        start foxglove_bridge                  (true)
    foxglove_port   websocket port                         (8765)
"""
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def find_fake_arduino() -> str:
    """tools/fake_arduino.py lives in the repo, not the install space. Walk up
    from this file (works for both colcon install layouts), else $TSP05_REPO."""
    here = Path(__file__).resolve()
    for parent in here.parents:
        candidate = parent / "tools" / "fake_arduino.py"
        if candidate.exists():
            return str(candidate)
    repo = os.environ.get("TSP05_REPO")
    if repo and (Path(repo) / "tools" / "fake_arduino.py").exists():
        return str(Path(repo) / "tools" / "fake_arduino.py")
    return ""


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("port")
    baud = LaunchConfiguration("baud")
    fake = LaunchConfiguration("fake")
    fake_link = LaunchConfiguration("fake_link")
    fake_script = find_fake_arduino()

    def bridge(port_value, condition):
        return Node(
            package="tsp05_bridge", executable="bridge", name="tsp05_bridge", output="screen",
            parameters=[{"port": port_value, "baud": baud}], condition=condition)

    actions = [
        DeclareLaunchArgument("port", default_value="/dev/ttyACM0"),
        DeclareLaunchArgument("baud", default_value="115200"),
        DeclareLaunchArgument("fake", default_value="false"),
        DeclareLaunchArgument("fake_link", default_value="/tmp/tsp05"),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        # The bridge retries every second, so it does not matter which starts first.
        bridge(port, UnlessCondition(fake)),
        bridge(fake_link, IfCondition(fake)),
        Node(
            package="foxglove_bridge", executable="foxglove_bridge", name="foxglove_bridge",
            output="screen", condition=IfCondition(LaunchConfiguration("foxglove")),
            parameters=[{
                "port": LaunchConfiguration("foxglove_port"),
                "address": "0.0.0.0",      # reachable from the host through Docker's port mapping
                "topic_whitelist": ["/tsp05/.*", "/rosout"],
                # Not client_topic_whitelist: foxglove_bridge 3.6 (Jazzy) accepts client
                # publishers on any topic regardless. Harmless here: the bridge only acts
                # on /tsp05/cmd/*, so a stray GUI publish cannot move a motor.
            }]),
    ]
    if fake_script:
        actions.append(ExecuteProcess(
            cmd=["python3", fake_script, "--link", fake_link],
            name="fake_arduino", output="screen", condition=IfCondition(fake)))
    else:
        actions.append(LogInfo(condition=IfCondition(fake),
                               msg="fake:=true but tools/fake_arduino.py not found; set TSP05_REPO"))
    return LaunchDescription(actions)
