# TSP05 development image: ROS 2 + foxglove_bridge + pyserial.
# The image only provides the environment. The repo is mounted at /tsp05,
# so the same code also runs on a native Ubuntu install (see README "Running").
#
#   ROS_DISTRO=jazzy  -> Ubuntu 24.04 (default)
#   ROS_DISTRO=humble -> Ubuntu 22.04
ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}-ros-base

ARG ROS_DISTRO
RUN apt-get update \
 && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-foxglove-bridge \
      python3-serial \
      python3-pytest \
      socat \
 && rm -rf /var/lib/apt/lists/*

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

WORKDIR /tsp05
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
