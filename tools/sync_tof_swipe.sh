#!/usr/bin/env bash
# Copies config.h and the swipe detector into firmware/bringup/tof_swipe so the sketch can compile.
# Edit the originals in firmware/tsp05, never the copies.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
src="$root/firmware/tsp05"
dst="$root/firmware/bringup/tof_swipe"
cp "$src/config.h" "$src/tof_gesture.h" "$src/tof_gesture.cpp" "$dst/"
echo "Copied swipe detector into $dst"
