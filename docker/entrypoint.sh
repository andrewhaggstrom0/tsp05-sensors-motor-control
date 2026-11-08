#!/bin/bash
# Source ROS 2 and, if it was built for this ROS distro, the tsp05 workspace; then run the command.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
ws=/tsp05/ros2_ws/install
if [ -f "$ws/setup.bash" ]; then
  # ros2_ws/ is shared by every image (it is the mounted repo); a build made by the
  # Humble image would fail to source in the Jazzy one, and vice versa.
  if grep -q "/opt/ros/${ROS_DISTRO}\"" "$ws/setup.sh" 2>/dev/null; then
    source "$ws/setup.bash"
  else
    echo "note: ros2_ws/install was built for another ROS distro; run: cd ros2_ws && colcon build" >&2
  fi
fi
exec "$@"
