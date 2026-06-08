#!/usr/bin/env bash
# poweron.sh — Bring a Universal Robots e-Series controller from a cold start
# all the way to RUNNING state, the same sequence a human performs on the
# teach pendant (PolyScope) by tapping the red "Initialize" button:
#
#   POWER_OFF  --(power on)-->   BOOTING  -->  IDLE  --(brake release)-->  RUNNING
#
# It also annotates the four backend interfaces a UR controller exposes so
# this script doubles as a tour of the network surface:
#
#   29999  Dashboard          Line-oriented ASCII commands. The high-level
#                             "remote control" surface — equivalent to
#                             tapping buttons in PolyScope. We drive the
#                             whole sequence from here.
#
#   30001  Primary Client     Bi-directional. Broadcasts a ~10 Hz binary
#                             state stream AND accepts raw URScript that
#                             executes immediately. The "scripting" backend.
#
#   30002  Secondary Client   Read-only mirror of Primary's broadcast,
#                             intended for external monitoring tools that
#                             must not be able to send commands.
#
#   30003  RT Interface       125/500 Hz binary state stream for hard
#                             real-time consumers (Matlab, ROS drivers
#                             pre-RTDE).
#
#   30004  RTDE               Modern subscription protocol — client picks
#                             which fields to send/receive and the cadence.
#                             What ros2_control / ur_robot_driver uses.
#
#   30020  Interpreter mode   (e-Series) URScript REPL that runs inside an
#                             already-loaded program — for line-by-line
#                             control without uploading a new .script.
#
# Run:    ./scripts/poweron.sh                      # localhost
#         UR_HOST=10.0.0.5 ./scripts/poweron.sh    # real robot

set -euo pipefail

HOST="${UR_HOST:-localhost}"
DASH_PORT="${UR_DASH_PORT:-29999}"
PRIMARY_PORT="${UR_PRIMARY_PORT:-30001}"
DEADLINE_S="${UR_TIMEOUT_S:-180}"

# ----- helpers ---------------------------------------------------------------

# True if a TCP port is accepting connections. Uses bash's /dev/tcp instead of
# `nc` so we don't depend on a netcat variant (BSD nc on macOS has no `-z`/`-q`).
_port_open() {
  (exec 3<>"/dev/tcp/$1/$2") 2>/dev/null
}

# Send one or more Dashboard commands in a single TCP session and print the
# server's response. The Dashboard server is line-oriented: each command is
# terminated by \n and the server replies with one line per command. We send
# `quit\n` last so the server closes the socket, giving us an EOF to read to.
# Uses bash /dev/tcp (not nc) for portability across GNU/BSD netcat.
dash() {
  exec 3<>"/dev/tcp/${HOST}/${DASH_PORT}" || return 1
  {
    for cmd in "$@"; do printf '%s\n' "$cmd"; done
    sleep 0.3
    printf 'quit\n'
  } >&3
  grep -v -e '^Connected:' -e '^Disconnected' <&3
  exec 3>&- 3<&-
}

# Block until `<cmd>` over Dashboard returns output containing `<want>`.
# Used to wait for state transitions (POWER_OFF -> IDLE -> RUNNING).
wait_for() {
  local cmd="$1" want="$2"
  local deadline=$(( SECONDS + DEADLINE_S ))
  while (( SECONDS < deadline )); do
    local out
    out="$(dash "$cmd" 2>/dev/null || true)"
    [[ -n "$out" ]] && printf '   %s\n' "$out"
    grep -q "$want" <<<"$out" && return 0
    sleep 2
  done
  printf 'ERROR: timed out waiting for "%s" from "%s"\n' "$want" "$cmd" >&2
  return 1
}

log() { printf '\n[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

# ----- 1. Wait for the Dashboard server socket itself ------------------------
# The container exposes 29999 from the moment it starts, BUT the Dashboard
# server only binds the socket after PolyScope (the Java OSGi runtime) finishes
# loading bundles — usually 20–40 s after the container boots.
log "Waiting for Dashboard server on ${HOST}:${DASH_PORT}..."
until _port_open "$HOST" "$DASH_PORT"; do sleep 1; done
log "Dashboard server is accepting connections."

# ----- 2. Wait for URControl (the C++ realtime backend) to come up -----------
# PolyScope can be up while URControl is still booting. In that window the
# Dashboard reports `Robotmode: NO_CONTROLLER`. We poll until it changes.
log "Waiting for URControl backend (robotmode != NO_CONTROLLER)..."
while dash robotmode | grep -q NO_CONTROLLER; do sleep 2; done
log "URControl is up. Current state:"
dash robotmode safetymode

# ----- 3. Clear any latched safety state -------------------------------------
# If the last shutdown left a protective stop or a popup on screen, those
# block the power-on sequence. These commands are no-ops in the happy path.
log "Clearing any latched safety popups / protective stops..."
dash 'close safety popup' 'close popup' || true
SM=$(dash safetymode | grep -oE 'Safetymode: [A-Z_]+' || true)
case "$SM" in
  *PROTECTIVE_STOP*) log "Protective stop active — releasing"; dash 'unlock protective stop' ;;
  *NORMAL*)          log "Safety mode is NORMAL." ;;
  *)                 log "Safety mode: $SM (proceeding)" ;;
esac

# ----- 4. Power on motors: POWER_OFF -> IDLE ---------------------------------
# `power on` energizes the 48 V motor bus. The robot transits through BOOTING
# while joint encoders initialize, then settles in IDLE (powered but brakes
# still engaged). On URSim there are no real motors, so the controller often
# blows straight through IDLE into RUNNING faster than we can poll — that's
# why we accept either IDLE or RUNNING here.
log "Powering on motors (POWER_OFF -> IDLE)..."
dash 'power on'
wait_for robotmode 'IDLE\|RUNNING'
log "Motors energized."

# ----- 5. Release brakes: IDLE -> RUNNING ------------------------------------
# Each joint has a mechanical brake. `brake release` runs the brake-release
# routine (you'll see the joints settle slightly under gravity on real
# hardware). End state is RUNNING — the robot is now movable. Safe to issue
# even if we're already RUNNING; the controller treats it as a no-op.
log "Releasing brakes (IDLE -> RUNNING)..."
dash 'brake release'
wait_for robotmode RUNNING
log "Brakes released. Robot is RUNNING."

# ----- 6. Tap the Primary Client interface (30001) ---------------------------
# Dashboard is the "buttons in the UI" surface. Primary Client is the
# "scripting" surface — anything you write to the socket is parsed as URScript
# and runs on the controller immediately. Here we send a single popup() call
# straight to URControl, bypassing Dashboard, to prove we can talk to the
# backend directly. NOTE: requires the controller to accept remote URScript;
# URSim does by default. On real hardware, PolyScope must be in "Remote
# Control" mode (top-right toggle) or have a program loaded that's running.
log "Demonstrating direct URScript on Primary Client (${HOST}:${PRIMARY_PORT})..."
if exec 4<>"/dev/tcp/${HOST}/${PRIMARY_PORT}" 2>/dev/null; then
  printf 'popup("Powered on via Dashboard — this popup came via Primary 30001", title="backend tap")\n' >&4
  sleep 0.5  # hold the socket open so URControl latches the program before FIN
  exec 4>&- 4<&-
fi

# ----- 7. Final report -------------------------------------------------------
log "Done. Final state:"
dash robotmode safetymode running
