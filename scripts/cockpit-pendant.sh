#!/usr/bin/env bash
# The RealSense cockpit, reachable from the robot: the PolyScope 5 URCap runs on the
# controller and calls the cockpit over the cell network, so it listens on every
# interface (--bind 0.0.0.0), not loopback. --cors also lets the PolyScope X sim's
# page (localhost:8001) call it. Run from a LOCAL Terminal on the Mac (sudo: libusb
# must detach Apple's UVC driver; SSH sessions are denied the camera by TCC).
#
#   scripts/cockpit-pendant.sh            # the ur3 cell
#   UR_CELL=ur20 scripts/cockpit-pendant.sh
#
# Anyone on the cell network can then reach the cockpit's robot routes — the same
# exposure the controller's own unauthenticated ports already have on that network.
set -euo pipefail
cd "$(dirname "$0")/.."

cell="${UR_CELL:-ur3}"
cors="${PERCEPTRONICS_CORS:-http://localhost:8001,http://127.0.0.1:8001}"
robot="$(sed -n 's/^UR_HOST=//p' "perceptronics/cells/$cell.env" | head -1)"
iface="$(route -n get "${robot:-192.168.3.3}" 2>/dev/null | awk '/interface:/{print $2}')"
ip="$(ipconfig getifaddr "${iface:-en0}" 2>/dev/null || true)"

echo "cell $cell · robot ${robot:-?} · this Mac ${ip:-?} (${iface:-?})"
echo "In the pendant's Perceptronic node, set Cockpit to:  http://${ip:-<this-mac-ip>}:7621"
py="${PYTHON:-}"
if [ -z "$py" ]; then py=".venv/bin/python"; [ -x "$py" ] || py="$(command -v python3)"; fi
exec sudo "$py" -m perceptronics --cell "$cell" gui --rs-lean --bind 0.0.0.0 --cors "$cors"
