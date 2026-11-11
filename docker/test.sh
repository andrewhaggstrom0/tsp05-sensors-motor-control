#!/usr/bin/env bash
# Build the ROS 2 workspace and run every test inside the dev container.
#   docker/test.sh                      Jazzy
#   ROS_DISTRO=humble docker/test.sh    Humble
# Natively on Ubuntu (ROS sourced), the same steps are:
#   cd ros2_ws && colcon build && colcon test && colcon test-result --verbose
set -euo pipefail
cd "$(dirname "$0")"
# No published ports: tests must run even while the bridge (port 8765) is up.
export TSP05_NO_PORTS=1
exec ./run.sh bash -c '
  set -e
  cd ros2_ws
  colcon build --event-handlers console_direct- >/dev/null
  colcon test --event-handlers console_direct- >/dev/null || true
  colcon test-result --verbose
'
