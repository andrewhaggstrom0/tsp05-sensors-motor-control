#!/usr/bin/env bash
# Share the Uno's USB serial port with the bridge in Docker (macOS).
# Docker Desktop on macOS cannot pass USB devices into a container, so socat
# relays the port over TCP and the bridge connects with
#   port:=socket://host.docker.internal:7777
#
#   tools/uno_relay.sh                          auto-detect the Uno
#   tools/uno_relay.sh /dev/cu.usbmodem11401    use this port
#   TSP05_RELAY_PORT=7800 tools/uno_relay.sh    another TCP port
#
# Needs socat (brew install socat). Listens on 127.0.0.1 only, so the Uno is not
# reachable from the network. Port 7777, not 7000: macOS's AirPlay Receiver
# already listens on 7000. Stop with Ctrl-C.
set -euo pipefail

tcp_port="${TSP05_RELAY_PORT:-7777}"
dev="${1:-}"

if [ -z "$dev" ]; then
  found=()
  for d in /dev/cu.usbmodem* /dev/cu.usbserial* /dev/cu.wchusbserial*; do
    [ -e "$d" ] && found+=("$d")
  done
  if [ "${#found[@]}" -ne 1 ]; then
    echo "Found ${#found[@]} USB serial ports; plug in the Uno or pass its port:" >&2
    ls /dev/cu.* >&2
    exit 1
  fi
  dev="${found[0]}"
fi
[ -e "$dev" ] || { echo "No such port: $dev" >&2; exit 1; }

socat_bin="$(command -v socat || true)"
for p in /opt/homebrew/bin/socat /usr/local/bin/socat; do
  [ -z "$socat_bin" ] && [ -x "$p" ] && socat_bin="$p"
done
[ -n "$socat_bin" ] || { echo "socat not found: brew install socat" >&2; exit 1; }

# Only one program can own the port (close the Arduino Serial Monitor and loggers).
if users="$(lsof "$dev" 2>/dev/null)" && [ -n "$users" ]; then
  echo "$dev is already open by:" >&2
  echo "$users" >&2
  exit 1
fi
if lsof -nP -iTCP:"$tcp_port" -sTCP:LISTEN >/dev/null 2>&1; then
  echo "TCP port $tcp_port is in use; set TSP05_RELAY_PORT to another port." >&2
  exit 1
fi

echo "Relaying $dev <-> 127.0.0.1:$tcp_port"
echo "Bridge: docker/run.sh ros2 launch tsp05_bridge tsp05.launch.py port:=socket://host.docker.internal:$tcp_port"
# fork: a new connection (bridge restart) reopens the port, which resets the Uno.
# ispeed/ospeed: socat 1.8 on macOS rejects the older b115200 option.
# rawer: raw bytes, no echo, no line editing.
exec "$socat_bin" -d TCP-LISTEN:"$tcp_port",bind=127.0.0.1,reuseaddr,fork \
  FILE:"$dev",ispeed=115200,ospeed=115200,rawer
