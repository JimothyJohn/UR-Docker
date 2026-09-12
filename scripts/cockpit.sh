#!/usr/bin/env bash
# The pilot's seat on macOS/Linux: the RGB-D cockpit for one cell.
#
#   scripts/cockpit.sh                 # sim cell (synthetic camera, PolyScope X sim)
#   scripts/cockpit.sh ur20            # the UR20 cell (real camera: sudo on macOS)
#   scripts/cockpit.sh ur3 --robot-dry-run
#   scripts/cockpit.sh ur20 --doctor   # pre-flight only
set -euo pipefail
cd "$(dirname "$0")/.."
cell="${1:-sim}"; shift || true
if [ "${1:-}" = "--doctor" ]; then
    shift
    exec uv run perception --cell "$cell" doctor "$@"
fi
# A real camera on macOS needs root to claim the USB interface; the sim cell (PERCEPTION_FAKE) does not.
if [ "$(uname)" = "Darwin" ] && [ "$cell" != "sim" ] && [ "$(id -u)" -ne 0 ]; then
    echo "macOS: the RealSense needs root — re-running under sudo" >&2
    exec sudo -E uv run perception --cell "$cell" gui "$@"
fi
exec uv run perception --cell "$cell" gui "$@"
