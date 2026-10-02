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
# Python >= 3.10, nothing to install: the package runs from this checkout. PYTHON overrides the choice.
py="${PYTHON:-}"
if [ -z "$py" ]; then py=".venv/bin/python"; [ -x "$py" ] || py="$(command -v python3 || true)"; fi
if [ -z "$py" ] || ! "$py" -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
    echo "Python 3.10 or newer is needed (found: ${py:-none}); set PYTHON=/path/to/python3" >&2
    exit 1
fi
if [ "${1:-}" = "--doctor" ]; then
    shift
    exec "$py" -m perceptronics --cell "$cell" doctor "$@"
fi
# A real camera on macOS needs root to claim the USB interface; the sim cell (PERCEPTRONICS_FAKE) does not.
if [ "$(uname)" = "Darwin" ] && [ "$cell" != "sim" ] && [ "$(id -u)" -ne 0 ]; then
    echo "macOS: the RealSense needs root — re-running under sudo" >&2
    exec sudo -E "$py" -m perceptronics --cell "$cell" gui "$@"
fi
exec "$py" -m perceptronics --cell "$cell" gui "$@"
