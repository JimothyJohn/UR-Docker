#!/bin/bash
# urmagic_perceptronic.sh — installs the Perceptronic URCap on a PolyScope 5 robot by
# itself when the USB stick goes in.
#
# PolyScope 5 runs every `urmagic_*.sh` at the top level of a USB stick, as root, when the
# stick is plugged in — if Settings → Security → General has "Run magic files" (and "USB
# ports") enabled; the pendant shows "! USB !" while it runs and "<- USB" when it is done.
# PolyScope X has no such hook: install the .urcapx on the same stick through System
# Manager, or run scripts/urcapx-autoinstall.sh from a computer on the robot's network.
#
# What it does, idempotently (the stick can stay in, and it runs again after the restart):
#   1. checks the .urcap next to it against the sha256 it was written with;
#   2. if /root/.urcaps/<Bundle-SymbolicName>.jar is already that file, does nothing;
#   3. otherwise copies it there — the directory PolyScope's own URCaps screen installs
#      into (URCapsServiceImpl: ".urcaps", getDestinationURCapName; UR's remotetcp URCap
#      installs itself as /root/.urcaps/<symbolicname>.jar the same way) and re-installs
#      from at every start (installAllURCaps);
#   4. tells the operator over the Dashboard (addToLog + popup) and restarts the controller
#      so PolyScope loads it — with URMAGIC_RESTART=auto (the default) only while the arm
#      is powered off and no program is running; =never leaves the restart to the
#      operator; =always restarts regardless.
# Every run is logged next to this script (urmagic_perceptronic.log) — read it back on
# the laptop when something looks wrong.
#
# scripts/urcap5-usb.sh writes this to the stick with the three @…@ values filled in.
set -u

URCAP_FILE="perceptronic-ps5-0.9.0.urcap"
URCAP_SHA256="c26c0022c8f260f4dc2e4bbb5a57960f27029688404acf2d06e769b3c5ea7a9f"
SYMBOLIC_NAME="io.advin.perceptronic"

URCAPS_DIR="${URCAPS_DIR:-/root/.urcaps}"
DASHBOARD_HOST="${DASHBOARD_HOST:-127.0.0.1}"
DASHBOARD_PORT="${DASHBOARD_PORT:-29999}"
URMAGIC_RESTART="${URMAGIC_RESTART:-auto}"        # auto | never | always
URMAGIC_REBOOT_CMD="${URMAGIC_REBOOT_CMD:-reboot}"

here="$(cd "$(dirname "$0")" && pwd)"
log="$here/urmagic_perceptronic.log"
src="$here/$URCAP_FILE"
dest="$URCAPS_DIR/$SYMBOLIC_NAME.jar"

say() {
  local line
  line="$(date '+%Y-%m-%d %H:%M:%S') $*"
  printf '%s\n' "$line"
  printf '%s\n' "$line" >>"$log" 2>/dev/null || true
}

# One Dashboard command; prints its reply line, or nothing when the Dashboard is not there.
dash() {
  local reply=""
  if exec 3<>"/dev/tcp/$DASHBOARD_HOST/$DASHBOARD_PORT" 2>/dev/null; then
    read -r -t 5 _ <&3 || true          # "Connected: Universal Robots Dashboard Server"
    printf '%s\n' "$1" >&3 2>/dev/null || true
    read -r -t 5 reply <&3 || true
    printf 'quit\n' >&3 2>/dev/null || true
    exec 3>&- 3<&-
  fi
  printf '%s\n' "${reply//$'\r'/}"
}

sha_of() { sha256sum "$1" 2>/dev/null | cut -d' ' -f1; }

say "== $SYMBOLIC_NAME ($URCAP_FILE) from $here"
if [[ ! -f "$src" ]]; then
  say "ERROR: $URCAP_FILE is not next to this script"
  dash "popup Perceptronic: $URCAP_FILE is missing from the USB stick" >/dev/null
  exit 1
fi
got="$(sha_of "$src")"
if [[ "$got" != "$URCAP_SHA256" ]]; then
  say "ERROR: $URCAP_FILE on the stick has sha256 $got, expected $URCAP_SHA256"
  dash "popup Perceptronic: $URCAP_FILE on the USB stick is damaged - copy it again" >/dev/null
  exit 1
fi

if [[ -f "$dest" && "$(sha_of "$dest")" == "$URCAP_SHA256" ]]; then
  say "already installed: $dest"
  exit 0
fi

if ! mkdir -p "$URCAPS_DIR"; then
  say "ERROR: cannot create $URCAPS_DIR"
  exit 1
fi
if ! cp "$src" "$dest.tmp" || ! mv -f "$dest.tmp" "$dest"; then
  rm -f "$dest.tmp"
  say "ERROR: cannot write $dest"
  dash "popup Perceptronic: could not write $dest" >/dev/null
  exit 1
fi
chmod 644 "$dest"
say "installed $dest"
dash "addToLog Perceptronic URCap $URCAP_FILE installed from USB" >/dev/null

mode="$(dash robotmode)"
prog="$(dash programState)"
restart=no
case "$URMAGIC_RESTART" in
  always) restart=yes ;;
  never) restart=no ;;
  *) [[ "$mode" == "Robotmode: POWER_OFF" && "$prog" == STOPPED* ]] && restart=yes ;;
esac

if [[ "$restart" == yes ]]; then
  say "restarting the controller so PolyScope loads it (${mode:-no Dashboard}; program ${prog:-unknown})"
  sync
  # shellcheck disable=SC2086  # a command line, on purpose (tests set `echo reboot`)
  $URMAGIC_REBOOT_CMD
else
  say "not restarting (${mode:-no Dashboard}; program ${prog:-unknown}): restart the robot to load it"
  dash "popup Perceptronic URCap installed from USB. Restart the robot to load it." >/dev/null
fi
exit 0
