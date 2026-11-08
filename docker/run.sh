#!/usr/bin/env bash
# Open a shell (or run a command) in the TSP05 dev container.
#   docker/run.sh                       interactive shell
#   docker/run.sh python3 tools/fake_arduino.py
#   ROS_DISTRO=humble docker/run.sh     use the Humble image instead of Jazzy
set -euo pipefail
cd "$(dirname "$0")"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
tty_flag=()
[ -t 0 ] && [ -t 1 ] || tty_flag=(-T)   # no terminal (scripts, CI): no pseudo-TTY
exec docker compose run --rm --build --service-ports "${tty_flag[@]}" dev "$@"
