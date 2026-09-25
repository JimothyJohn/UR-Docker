#!/usr/bin/env bash
# rs_probe.sh — does the RealSense stream from a long-lived process on this box?
#
# Runs the perception cockpit headless twice and samples its /api/info:
#   phase 1  first open after you (re-)plugged the camera
#   phase 2  the same again with no cable touch  (the "re-open" case)
# and prints frames read / fps / last error for each, then a verdict.
# Needs root on macOS (libusb must take the camera from Apple's UVC driver);
# it re-execs itself under sudo. No browser or SSH tunnel needed.
#
#   usage: scripts/rs_probe.sh [--port N] [--samples N] [--interval S] [--single] [--fake]
#   (--fake drives the synthetic camera instead — no root, no hardware; a self-test)
set -euo pipefail

PORT=7621
SAMPLES=4
INTERVAL=8
SINGLE=0
FAKE=0
RESULT_first=""
RESULT_reopen=""
while [ $# -gt 0 ]; do
  case "$1" in
    --port) PORT="$2"; shift 2 ;;
    --samples) SAMPLES="$2"; shift 2 ;;
    --interval) INTERVAL="$2"; shift 2 ;;
    --single) SINGLE=1; shift ;;
    --fake) FAKE=1; shift ;;
    -h|--help) sed -n '2,11p' "$0"; exit 0 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;;
  esac
done

if [ "$FAKE" = 0 ] && [ "$(uname)" = "Darwin" ] && [ "$(id -u)" -ne 0 ]; then
  echo "re-running under sudo (macOS needs root to open the camera)" >&2
  SUDO_ARGS=(--port "$PORT" --samples "$SAMPLES" --interval "$INTERVAL")
  [ "$SINGLE" = 1 ] && SUDO_ARGS+=(--single)
  exec sudo "$0" "${SUDO_ARGS[@]}"
fi
GUI_ARGS=(gui --no-browser --port "$PORT")
[ "$FAKE" = 1 ] && GUI_ARGS+=(--fake)

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
if [ -x "$ROOT/.venv/bin/perception" ]; then
  PERCEPTION=("$ROOT/.venv/bin/perception")
else
  PERCEPTION=(uv run --project "$ROOT" perception)
fi
URL="http://127.0.0.1:$PORT/api/info"
LOG_DIR="$(mktemp -d "${TMPDIR:-/tmp}/rs_probe.XXXXXX")"  # per run: a root-owned /tmp file from a sudo run blocks the next plain run
GUI_PID=""

cleanup() {
  if [ -n "$GUI_PID" ] && kill -0 "$GUI_PID" 2>/dev/null; then
    kill "$GUI_PID" 2>/dev/null || true
    for _ in 1 2 3 4 5 6 7 8 9 10; do kill -0 "$GUI_PID" 2>/dev/null || break; sleep 0.5; done
    kill -9 "$GUI_PID" 2>/dev/null || true
  fi
  GUI_PID=""
}
trap cleanup EXIT INT TERM

info() {  # -> "frames fps error"
  curl -s --max-time 3 "$URL" | python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    print("- - (no reply)"); sys.exit(0)
err = d.get("last_error") or "-"
print(d.get("frames_read", "-"), d.get("fps", "-"), err.replace("\n", " ")[:160])'
}

port_free() { ! curl -s --max-time 1 "$URL" >/dev/null 2>&1; }

run_phase() {  # $1 = label ; sets RESULT_<label> to "streamed"|"stalled"|"failed"
  local label="$1" first="" last="" frames fps err verdict
  echo
  echo "== phase $label: starting cockpit on :$PORT"
  if ! port_free; then echo "   port $PORT already in use — stop that process first" >&2; exit 1; fi
  "${PERCEPTION[@]}" "${GUI_ARGS[@]}" >"$LOG_DIR/$label.log" 2>&1 &
  GUI_PID=$!
  local up=0
  for _ in $(seq 1 40); do
    if ! port_free; then up=1; break; fi
    if ! kill -0 "$GUI_PID" 2>/dev/null; then break; fi
    sleep 0.5
  done
  if [ "$up" -ne 1 ]; then
    echo "   cockpit never answered on :$PORT (log: $LOG_DIR/$label.log)"
    tail -20 "$LOG_DIR/$label.log" | sed 's/^/   | /'
    cleanup; eval "RESULT_$label=failed"; return
  fi
  printf '   %-6s %-8s %-6s %s\n' "t(s)" "frames" "fps" "last_error"
  local t=0
  for i in $(seq 1 "$SAMPLES"); do
    sleep "$INTERVAL"; t=$((t + INTERVAL))
    read -r frames fps err <<<"$(info)"
    printf '   %-6s %-8s %-6s %s\n' "$t" "$frames" "$fps" "$err"
    [ "$i" -eq 1 ] && first="$frames"
    last="$frames"
  done
  if [ "$last" != "-" ] && [ "$first" != "-" ] && [ "${last:-0}" -gt "${first:-0}" ] 2>/dev/null; then
    verdict=streamed
  elif [ "${last:-0}" != "-" ] && [ "${last:-0}" -gt 0 ] 2>/dev/null; then
    verdict=stalled   # got some frames, then stopped
  else
    verdict=failed
  fi
  echo "   -> $verdict"
  echo "   SDK/log tail ($LOG_DIR/$label.log):"
  { grep -v '^$' "$LOG_DIR/$label.log" || true; } | tail -6 | sed 's/^/   | /'
  cleanup
  eval "RESULT_$label=$verdict"
}

PROBE_START="$(date '+%Y-%m-%d %H:%M:%S')"
echo "rs_probe: $(date '+%H:%M:%S') on $(hostname -s), cockpit = ${PERCEPTION[*]}, logs in $LOG_DIR"
run_phase first
if [ "$SINGLE" = 1 ]; then exit 0; fi
echo; echo "   waiting 5 s, then re-opening without touching the cable"
sleep 5
run_phase reopen

echo
echo "== verdict"
echo "   first open after plug : ${RESULT_first}"
echo "   re-open, no re-plug   : ${RESULT_reopen}"
case "${RESULT_first}/${RESULT_reopen}" in
  streamed/streamed) echo "   camera re-opens fine on this box — no workaround needed" ;;
  streamed/*)        echo "   only the first open after a plug works — workflow: re-plug, then ONE long-lived process" ;;
  *)                 echo "   even the first open after a plug did not stream — first-open-wins is dead; see the log tails above" ;;
esac
if [ "$(uname)" = "Darwin" ]; then
  # Every exclusive-open loss the kernel logged since the probe started, both
  # directions. Read all of it: python's own losses are the ones that matter,
  # and a tail of the last few lines hid them once (2026-09-25).
  CLAIMS="$(/usr/bin/log show --start "$PROBE_START" --predicate 'eventMessage CONTAINS "RealSense" AND eventMessage CONTAINS "exclusive access by pid"' --style compact 2>/dev/null \
    | grep -v '^Timestamp\|^Filtering\|com.apple.log' \
    | sed -E 's/^[0-9-]+ ([0-9:]+)\.[0-9]+ .*(python3[0-9.]*|UVCAssistant|[A-Za-z0-9._-]+)@.*Camera 435( with RGB Module)? *([A-Za-z]+)@([0-9]).*by pid ([0-9]+), ([A-Za-z0-9._-]+).*/   \1 lost=\2 iface=\4@\5 winner=\7/' || true)"
  RESETS="$(/usr/bin/log show --start "$PROBE_START" --predicate 'eventMessage CONTAINS "RealSense" AND eventMessage CONTAINS "reset API call"' --style compact 2>/dev/null | grep -c terminateDevice || true)"
  echo "   USB interface claims lost this run (OS log since $PROBE_START):"
  echo "     python lost $(printf '%s\n' "$CLAIMS" | grep -c 'lost=python' || true), UVCAssistant lost $(printf '%s\n' "$CLAIMS" | grep -c 'lost=UVCAssistant' || true), other $(printf '%s\n' "$CLAIMS" | grep -v 'lost=python\|lost=UVCAssistant' | grep -c 'lost=' || true)"
  printf '%s\n' "$CLAIMS" | grep -v 'lost=UVCAssistant' | grep . || echo "     (python lost none)"
  echo "   camera USB resets this run (libusb re-enumerates with capture on every handle open): ${RESETS:-0}"
  echo "   every reset re-runs the race with UVCAssistant and kills a stream already running"
fi
