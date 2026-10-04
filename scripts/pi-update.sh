#!/usr/bin/env bash
# Update bundles for pick PCs that have no SSH (a card flashed from the image, unseeded):
# build one from this checkout, or send one to a pick PC's setup portal and follow it.
#
#   scripts/pi-update.sh bundle [--librealsense TAR] [--out DIR]
#   scripts/pi-update.sh push BUNDLE [http://192.168.3.20:7621]
#
# bundle: the wheel of this checkout + deploy/pi/ in one file (target/pi-update/
# perceptronics-update-<version>-<rev>.tar), checked member by member against its manifest on
# the PC. --librealsense adds a prebuilt library (scripts/pi-image.sh caches one in
# target/pi-image/cache/) — needed only when the bundle pins a librealsense the PC doesn't have
# yet, because a cell PC has no internet to build one.
# push: logs in as PERCEPTRONICS_ADMIN_USER / PERCEPTRONICS_ADMIN_PASSWORD (default admin/admin),
# uploads, and prints the PC's install log until it reports done / failed / rolled back.
# An operator does the same on http://<pick PC>:7621/setup with no tools at all.
#
# PYTHON=/path/to/python3 picks the Python that builds the wheel (default: the first with pip).
set -euo pipefail

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
die() { printf 'pi-update: %s\n' "$*" >&2; exit 1; }
log() { printf '[pi-update] %s\n' "$*"; }

repo="$(cd "$(dirname "$0")/.." && pwd)"
admin="${repo}/deploy/pi/perceptronics-admin"

[ $# -ge 1 ] || { usage; exit 2; }
cmd="$1"; shift

# The first Python that has pip (a repo .venv made from requirements-dev.txt has none).
find_python() {
    local cand
    for cand in ${PYTHON:+"$PYTHON"} python3 /opt/homebrew/bin/python3 /usr/local/bin/python3 /usr/bin/python3; do
        if command -v "$cand" >/dev/null 2>&1 && "$cand" -m pip --version >/dev/null 2>&1 \
            && "$cand" -c 'import sys; sys.exit(sys.version_info < (3, 10))'; then
            echo "$cand"
            return 0
        fi
    done
    return 1
}

case "$cmd" in
    bundle)
        lrs="" out="${repo}/target/pi-update"
        while [ $# -gt 0 ]; do
            case "$1" in
                --librealsense) lrs="${2:?--librealsense needs a tarball}"; shift 2 ;;
                --out) out="${2:?--out needs a directory}"; shift 2 ;;
                -h | --help) usage; exit 0 ;;
                *) die "unknown argument: $1 (see --help)" ;;
            esac
        done
        py="$(find_python)" || die "no python3 >= 3.10 with pip found; set PYTHON=/path/to/python3"
        rev="$(git -C "$repo" rev-parse HEAD | cut -c1-12)"
        [ -z "$(git -C "$repo" status --porcelain -- perceptronics urctl deploy/pi pyproject.toml)" ] || rev="${rev}-dirty"
        stage="$(mktemp -d)"
        trap 'rm -rf "$stage"' EXIT
        log "building the wheel ($py -m pip wheel)"
        "$py" -m pip wheel "$repo" --no-deps --wheel-dir "$stage" -q
        wheel="$(find "$stage" -maxdepth 1 -name 'perceptronics-*-py3-none-any.whl' | head -n 1)"
        [ -n "$wheel" ] || die "pip wheel produced no perceptronics wheel"
        args=(make-bundle --wheel "$wheel" --deploy "${repo}/deploy/pi" --out "$out" --source-rev "$rev")
        [ -n "$lrs" ] && args+=(--librealsense "$lrs")
        bundle="$("$py" "$admin" "${args[@]}")"
        "$py" "$admin" verify "$bundle" >/dev/null
        log "wrote ${bundle} ($(du -h "$bundle" | cut -f1))"
        log "install it: scripts/pi-update.sh push ${bundle#"${repo}/"}   (or upload it on http://<pick PC>:7621/setup)"
        ;;
    push)
        [ $# -ge 1 ] || die "push needs a bundle (scripts/pi-update.sh bundle makes one)"
        bundle="$1" url="${2:-http://192.168.3.20:7621}"
        case "$url" in http://*) ;; *) url="http://${url}" ;; esac
        case "$url" in http://*:*) ;; *) url="${url}:7621" ;; esac
        py="$(find_python)" || py=python3
        exec "$py" "$admin" push "$bundle" --url "$url"
        ;;
    -h | --help) usage ;;
    *) die "unknown command: ${cmd} (bundle | push)" ;;
esac
