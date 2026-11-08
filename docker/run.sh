#!/usr/bin/env bash
# Open a shell (or run a command) in the TSP05 dev container.
#   docker/run.sh                       interactive shell
#   docker/run.sh python3 tools/fake_arduino.py
#   ROS_DISTRO=humble docker/run.sh     use the Humble image instead of Jazzy
#   TSP05_SERIAL_DEVICE=/dev/ttyACM0 docker/run.sh   pass the real Uno in (Linux)
set -euo pipefail
cd "$(dirname "$0")"
export HOST_UID="$(id -u)" HOST_GID="$(id -g)"
tty_flag=()
[ -t 0 ] && [ -t 1 ] || tty_flag=(-T)   # no terminal (scripts, CI): no pseudo-TTY
files=(-f compose.yaml)
if [ -n "${TSP05_SERIAL_DEVICE:-}" ]; then
  [ -e "$TSP05_SERIAL_DEVICE" ] || { echo "no such device: $TSP05_SERIAL_DEVICE" >&2; exit 1; }
  files+=(-f compose.serial.yaml)
fi
exec docker compose "${files[@]}" run --rm --build --service-ports ${tty_flag[@]+"${tty_flag[@]}"} dev "$@"
