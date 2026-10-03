#!/usr/bin/env bash
# Deploy the pick PC from this checkout: build the wheel, copy it and deploy/pi/ to the PC,
# run deploy/pi/install.sh there under sudo, then print `perceptronics doctor` from the PC.
#
#   scripts/deploy-pi.sh pi@192.168.3.10                      # cell ur3 (the default)
#   scripts/deploy-pi.sh pi@192.168.3.10 --cell ur3 --robot-host 192.168.3.3
#   scripts/deploy-pi.sh pi@192.168.3.10 --allow-from 192.168.3.0/24
#   scripts/deploy-pi.sh pi@10.0.0.56 --cell-if eth0 --cell-address 192.168.3.20/24   # the defaults
#   scripts/deploy-pi.sh pi@10.0.0.56 --cell-if none          # leave the PC's network alone
#   scripts/deploy-pi.sh pi@192.168.3.10 --doctor-only
#   scripts/deploy-pi.sh pi@192.168.3.10 --rollback            # previous release, restart
#
# --cell / --robot-host rewrite /etc/perceptronics/cell.env on the PC (the old one is kept
# beside it); without them an existing cell.env is left alone. --cell-if / --cell-address
# set the cell port up for a robot nobody configured (install.sh: a fixed address, and a
# one-lease DHCP server for the robot when no other DHCP server answers there). Authentication is your
# SSH key (or ssh's own password prompt); sudo on the PC prompts on the terminal (ssh -t),
# or, run without a terminal (an agent), must be passwordless (`sudo -n`, fails fast).
# Nothing here reads, stores or echoes a password.

# Remote commands are built on this side on purpose, every argument through printf %q
# (remote_cmd), so SC2029's "expands on the client side" is the intent here.
# shellcheck disable=SC2029
set -euo pipefail

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
die() { printf 'deploy-pi: %s\n' "$*" >&2; exit 1; }
log() { printf '[deploy-pi] %s\n' "$*"; }

[ $# -ge 1 ] || { usage; exit 2; }
case "$1" in -h | --help) usage; exit 0 ;; esac
target="$1"; shift
case "$target" in -*) die "first argument must be user@host, got ${target}" ;; esac

install_args=()
mode=deploy
reconfigure=0
while [ $# -gt 0 ]; do
    case "$1" in
        --cell | --robot-host | --allow-from | --cell-if | --cell-address)
            [ $# -ge 2 ] || die "$1 needs a value"
            install_args+=("$1" "$2")
            case "$1" in --cell | --robot-host) reconfigure=1 ;; esac
            shift 2
            ;;
        --reconfigure) reconfigure=1; shift ;;
        --doctor-only) mode=doctor; shift ;;
        --rollback) mode=rollback; shift ;;
        -h | --help) usage; exit 0 ;;
        *) die "unknown argument: $1 (see --help)" ;;
    esac
done
[ "$reconfigure" = 1 ] && install_args+=(--reconfigure)

repo="$(cd "$(dirname "$0")/.." && pwd)"
ssh_opts=(-o ConnectTimeout=10)
# sudo on the PC: with a terminal, allocate one so sudo can prompt; without (an agent's
# shell), `sudo -n` fails at once instead of hanging on a prompt nobody can answer.
if [ -t 0 ]; then
    tty_opt=(-t)
    sudo_cmd=(sudo)
else
    tty_opt=(-T)
    sudo_cmd=(sudo -n)
fi

# Quote each argument for the remote shell, so a value is never re-split or expanded there.
remote_cmd() {
    local out="" a
    for a in "$@"; do out+="$(printf '%q' "$a") "; done
    printf '%s' "$out"
}

run_doctor() {
    log "perceptronics doctor on ${target}:"
    # The doctor exits non-zero when a check fails; show it, don't abort on it.
    ssh "${tty_opt[@]}" "${ssh_opts[@]}" "$target" "$(remote_cmd "${sudo_cmd[@]}" perceptronics-doctor)" \
        || log "doctor reported failures (above)"
}

command -v ssh >/dev/null || die "ssh not found"
log "checking ${target} is reachable"
ssh "${ssh_opts[@]}" "$target" true || die "cannot ssh to ${target}"

case "$mode" in
    doctor) run_doctor; exit 0 ;;
    rollback)
        ssh "${tty_opt[@]}" "${ssh_opts[@]}" "$target" \
            "$(remote_cmd "${sudo_cmd[@]}" /opt/perceptronics/deploy/install.sh --rollback)"
        run_doctor
        exit 0
        ;;
esac

python3 -m pip --version >/dev/null 2>&1 || die "python3 with pip not found (needed to build the wheel)"
arch="$(ssh "${ssh_opts[@]}" "$target" uname -m)"
[ "$arch" = aarch64 ] || log "warning: ${target} is ${arch}, not aarch64 — install.sh builds natively, carrying on"

stage_local="$(mktemp -d)"
trap 'rm -rf "$stage_local"' EXIT
log "building the wheel (pip wheel)"
python3 -m pip wheel "$repo" --no-deps --wheel-dir "$stage_local" -q
wheel="$(find "$stage_local" -maxdepth 1 -name '*-py3-none-any.whl' | head -n 1)"
[ -n "$wheel" ] || die "pip wheel produced no pure-Python wheel"
log "built $(basename "$wheel")"

stage_remote="$(ssh "${ssh_opts[@]}" "$target" 'mktemp -d /tmp/perceptronics-deploy.XXXXXX')"
[ -n "$stage_remote" ] || die "could not create a staging directory on ${target}"
log "copying to ${target}:${stage_remote}"
scp -q "${ssh_opts[@]}" "$wheel" "$repo"/deploy/pi/* "${target}:${stage_remote}/"

log "running install.sh on ${target} (sudo; the first run builds librealsense — tens of minutes)"
status=0
ssh "${tty_opt[@]}" "${ssh_opts[@]}" "$target" "$(remote_cmd "${sudo_cmd[@]}" bash "${stage_remote}/install.sh" \
    --wheel "${stage_remote}/$(basename "$wheel")" ${install_args[@]+"${install_args[@]}"})" || status=$?
ssh "${ssh_opts[@]}" "$target" "$(remote_cmd rm -rf "$stage_remote")" || true
[ "$status" -eq 0 ] || die "install.sh failed on ${target} (exit ${status})"

run_doctor
cell_address=192.168.3.20
for ((i = 0; i < ${#install_args[@]}; i++)); do
    [ "${install_args[$i]}" = --cell-address ] && cell_address="${install_args[$((i + 1))]%/*}"
done
if [ "$cell_address" = 192.168.3.20 ]; then
    log "done. On the pendant nothing to type: the URCap's Cockpit field defaults to this PC's cell address 192.168.3.20;" \
        "a robot already on 192.168.3.x/24 (the UR3e: 192.168.3.3) stays as it is; one on DHCP gets 192.168.3.3 from this PC." \
        "The whole procedure: PLUG-AND-PLAY.md"
else
    log "done. On the pendant: Installation -> URCaps -> Perceptronic -> Cockpit = ${cell_address} -> Save"
fi
