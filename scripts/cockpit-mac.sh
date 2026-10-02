#!/usr/bin/env bash
# The UR3e cockpit on the Mac Studio, exactly as run on the cell: lean RealSense open
# (the Mac loses the libusb claim race otherwise; docs/realsense.md §Troubleshooting) and
# CORS for the UR3 PolyScope X sim's page on :8001. The interpreter is resolved before
# sudo (root's PATH may hold an older python3); PYTHON overrides it.
# Run from a LOCAL Terminal (SSH sessions are denied the webcams by TCC).
#
#   scripts/cockpit-mac.sh                     # the ur3 cell
#   scripts/cockpit-mac.sh --robot-dry-run     # extra `gui` flags pass through
set -euo pipefail
cd "$(dirname "$0")/.."
py="${PYTHON:-}"
if [ -z "$py" ]; then py=".venv/bin/python"; [ -x "$py" ] || py="$(command -v python3)"; fi
exec sudo "$py" -m perceptronics --cell ur3 gui --rs-lean --cors http://localhost:8001 "$@"
