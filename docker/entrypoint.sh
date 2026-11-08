#!/bin/bash
# Source ROS 2 and, if it has been built, the tsp05 workspace, then run the command.
set -e
source "/opt/ros/${ROS_DISTRO}/setup.bash"
if [ -f /tsp05/ros2_ws/install/setup.bash ]; then
  source /tsp05/ros2_ws/install/setup.bash
fi
exec "$@"
