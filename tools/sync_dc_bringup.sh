#!/usr/bin/env bash
# Copies the DC motor module into firmware/bringup/dc_pid so the sketch can compile.
# Edit the originals in firmware/tsp05, never the copies.
set -euo pipefail
root="$(cd "$(dirname "$0")/.." && pwd)"
src="$root/firmware/tsp05"
dst="$root/firmware/bringup/dc_pid"
cp "$src/config.h" "$src/dc_control.h" "$src/dc_control.cpp" "$src/dc_motor.h" "$src/dc_motor.cpp" "$dst/"
echo "Copied DC module into $dst"
