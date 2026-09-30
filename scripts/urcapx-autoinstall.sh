#!/usr/bin/env bash
# Install (or replace) the PolyScope X URCap on a robot as soon as its urservice answers —
# the network-side stand-in for the PolyScope 5 stick's magic file, since PolyScope X runs
# nothing from a USB stick. Run it from a checkout on any computer on the robot's network
# (the pick PC at boot, a laptop); it waits for the robot, then installs through the same
# endpoint System Manager uses (urcap/urcapx.py, no Remote mode needed).
#
#   scripts/urcapx-autoinstall.sh <robot-host> [port]      # port 80 on a robot, 8000 on the sim
#   URCAPX_WAIT_S=1800 (default) — how long to wait for the robot before giving up.
set -euo pipefail
cd "$(dirname "$0")/.."

host="${1:?usage: $0 <robot-host> [port]}"
port="${2:-80}"
wait_s="${URCAPX_WAIT_S:-1800}"
urcapx="$(find urcap/dist -maxdepth 1 -name 'perceptronic-*.urcapx' | sort | tail -1)"
[[ -f "$urcapx" ]] || { echo "no urcap/dist/perceptronic-*.urcapx — make urcap-package"; exit 1; }

deadline=$((SECONDS + wait_s))
until python3 urcap/urcapx.py list --host "$host" --port "$port" >/dev/null 2>&1; do
  if ((SECONDS >= deadline)); then
    echo "PolyScope X at $host:$port did not answer within ${wait_s}s"
    exit 1
  fi
  sleep 10
done
python3 urcap/urcapx.py install "$urcapx" --host "$host" --port "$port" --replace
