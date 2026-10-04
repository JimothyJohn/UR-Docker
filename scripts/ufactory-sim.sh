#!/usr/bin/env bash
# UFACTORY's firmware simulator as an 850, for tests/test_ufactory_sim.py.
#
#   scripts/ufactory-sim.sh up       start it and wait until the control port answers
#   scripts/ufactory-sim.sh status   print the firmware version string
#   scripts/ufactory-sim.sh down     stop and remove it
#
# The image is UFACTORY's (docs.ufactory.cc, "How to install UFACTORY Studio in
# Docker"): amd64 only, pinned here by digest. Inside it, xarm_start.sh <axis>
# <type> starts the firmware and UFACTORY Studio; "6 12" is the 850. Studio's
# web UI is on :18333, the control port on :502, the report streams on
# :30000-30003. Ports are bound to 127.0.0.1.
set -euo pipefail

IMAGE="danielwang123321/uf-ubuntu-docker@sha256:ad56ecec2ba7818b7221650b7f9d26d5cac3f75af7aeb545854a860bd35c125a"
NAME="${UFACTORY_SIM_NAME:-perceptronics-ufactory-sim}"
AXIS=6
TYPE=12 # 850
PYTHON="${PYTHON:-python3}"
DOCKER="${DOCKER:-docker}"
TIMEOUT_S="${UFACTORY_SIM_TIMEOUT_S:-300}"

version() {
    # One GET_VERSION over the control port: prints the version string, exit 1 if nothing answers.
    "$PYTHON" - <<'EOF'
import sys
sys.path.insert(0, ".")
from urctl.xarm import XArmClient, XArmError
try:
    print(XArmClient("127.0.0.1", timeout=2.0).get_version().raw)
except XArmError:
    sys.exit(1)
EOF
}

up() {
    if ! "$DOCKER" ps --format '{{.Names}}' | grep -qx "$NAME"; then
        "$DOCKER" rm -f "$NAME" >/dev/null 2>&1 || true
        local ports=()
        for p in 18333 502 503 504 30000 30001 30002 30003; do
            ports+=(-p "127.0.0.1:$p:$p")
        done
        "$DOCKER" run -d --name "$NAME" "${ports[@]}" "$IMAGE" sleep infinity >/dev/null
        "$DOCKER" exec -d "$NAME" /xarm_scripts/xarm_start.sh "$AXIS" "$TYPE"
    fi
    local deadline=$((SECONDS + TIMEOUT_S))
    until version; do
        if ((SECONDS > deadline)); then
            echo "the simulator's control port never answered within ${TIMEOUT_S}s" >&2
            "$DOCKER" logs --tail 50 "$NAME" >&2 || true
            return 1
        fi
        sleep 2
    done
}

case "${1:-}" in
up) up ;;
status) version ;;
down) "$DOCKER" rm -f "$NAME" >/dev/null ;;
*)
    echo "usage: $0 up|status|down" >&2
    exit 2
    ;;
esac
