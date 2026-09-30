#!/usr/bin/env bash
# The UR3e cockpit on the Mac Studio, exactly as run on the cell: lean RealSense open
# (the Mac loses the libusb claim race otherwise; docs/realsense.md §Troubleshooting) and
# CORS for the UR3 PolyScope X sim's page on :8001. Runs the venv's entry point under
# sudo rather than `sudo uv run` (scripts/cockpit.sh), so root never touches uv's cache.
# Run from a LOCAL Terminal (SSH sessions are denied the webcams by TCC).
#
#   scripts/cockpit-mac.sh                     # the ur3 cell
#   scripts/cockpit-mac.sh --robot-dry-run     # extra `gui` flags pass through
set -euo pipefail
cd "$(dirname "$0")/.."
exec sudo .venv/bin/perceptronics --cell ur3 gui --rs-lean --cors http://localhost:8001 "$@"
