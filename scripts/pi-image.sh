#!/usr/bin/env bash
# The pick PC image, from the laptop: build it on an arm64 Pi, flash it to a card, seed it.
#
#   scripts/pi-image.sh build nick@10.0.0.56 [--cell ur3] [--robot-host IP] [--no-xz]
#       builds the wheel here, copies it and deploy/pi/ to ~/pi-image/stage on the Pi, runs
#       deploy/pi/image/build.sh there (sudo), and fetches the image into target/pi-image/.
#   scripts/pi-image.sh flash target/pi-image/<name>.img.xz disk4 [--yes]
#       macOS: writes the image to a removable/external disk (refuses anything else; asks you
#       to type the disk id unless --yes). Needs sudo for the raw device.
#   scripts/pi-image.sh seed /Volumes/bootfs --hostname pickpc2 --user nick \
#       --ssh-key ~/.ssh/id_ed25519.pub --address 192.168.3.21/24 [--robot-host IP] ...
#       writes the per-board cloud-init seed (deploy/pi/image/seed.py --help for every flag).
#
# The image is the same for every board; the seed is what makes it one board. A card can be
# re-seeded and re-flashed as often as you like. Nothing here reads, stores or echoes a
# password (the Wi-Fi one comes from PERCEPTRONICS_WIFI_PSK or seed.py's prompt).

# Remote commands are built on this side on purpose, every argument through printf %q.
# shellcheck disable=SC2029
set -euo pipefail

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "$0"; }
die() { printf 'pi-image: %s\n' "$*" >&2; exit 1; }
log() { printf '[pi-image] %s\n' "$*"; }

repo="$(cd "$(dirname "$0")/.." && pwd)"
out_dir="${repo}/target/pi-image"
ssh_opts=(-o ConnectTimeout=10)

remote_cmd() {
    local out="" a
    for a in "$@"; do out+="$(printf '%q' "$a") "; done
    printf '%s' "$out"
}

find_python() {
    local p
    for p in "${PYTHON:-}" python3 /opt/homebrew/bin/python3 /usr/local/bin/python3; do
        [ -n "$p" ] || continue
        command -v "$p" >/dev/null 2>&1 || continue
        "$p" -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' 2>/dev/null || continue
        printf '%s' "$p"
        return 0
    done
    return 1
}

cmd_build() {
    [ $# -ge 1 ] || die "build needs user@host"
    local target="$1"; shift
    case "$target" in -*) die "build: first argument must be user@host, got ${target}" ;; esac
    local py
    py="$(find_python)" || die "no python3 >= 3.10 found (set PYTHON=)"
    "$py" -m pip --version >/dev/null 2>&1 || die "${py} has no pip (set PYTHON= to one that does)"
    local rev
    rev="$(git -C "$repo" rev-parse --short=12 HEAD)"
    git -C "$repo" diff --quiet HEAD -- || rev="${rev}-dirty"

    local stage
    stage="$(mktemp -d)"
    trap 'rm -rf "$stage"' RETURN
    log "building the wheel"
    "$py" -m pip wheel "$repo" --no-deps --wheel-dir "$stage" -q
    local wheel
    wheel="$(find "$stage" -maxdepth 1 -name '*-py3-none-any.whl' | head -n 1)"
    [ -n "$wheel" ] || die "pip wheel produced no pure-Python wheel"

    ssh "${ssh_opts[@]}" "$target" true || die "cannot ssh to ${target}"
    [ "$(ssh "${ssh_opts[@]}" "$target" uname -m)" = aarch64 ] || die "${target} is not aarch64"
    log "staging on ${target}:~/pi-image/stage"
    ssh "${ssh_opts[@]}" "$target" 'rm -rf ~/pi-image/stage && mkdir -p ~/pi-image/stage/deploy ~/pi-image/out ~/pi-image/cache'
    scp -q -r "${ssh_opts[@]}" "${repo}/deploy/pi" "${target}:pi-image/stage/deploy/"
    scp -q "${ssh_opts[@]}" "$wheel" "${target}:pi-image/stage/"

    local tty_opt=(-T) sudo_cmd=(sudo -n)
    if [ -t 0 ]; then tty_opt=(-t); sudo_cmd=(sudo); fi
    local home
    home="$(ssh "${ssh_opts[@]}" "$target" 'printf %s "$HOME"')"
    log "building on ${target} (first build compiles librealsense: ~10 min on a Pi 5)"
    ssh "${tty_opt[@]}" "${ssh_opts[@]}" "$target" "$(remote_cmd "${sudo_cmd[@]}" bash \
        "${home}/pi-image/stage/deploy/pi/image/build.sh" \
        --wheel "${home}/pi-image/stage/$(basename "$wheel")" \
        --out "${home}/pi-image/out" --cache "${home}/pi-image/cache" \
        --source-rev "$rev" "$@")" || die "build.sh failed on ${target}"

    local newest
    newest="$(ssh "${ssh_opts[@]}" "$target" 'ls -t ~/pi-image/out/*.manifest.txt | head -n 1')"
    [ -n "$newest" ] || die "no image in ~/pi-image/out on ${target}"
    local name
    name="$(basename "$newest" .manifest.txt)"
    mkdir -p "$out_dir"
    log "fetching ${name} into ${out_dir}"
    scp -q "${ssh_opts[@]}" "${target}:pi-image/out/${name}.*" "$out_dir/"
    (cd "$out_dir" && shasum -a 256 -c "$(ls "${name}".img*.sha256)") || die "fetched image fails its sha256"
    log "done: $(ls "${out_dir}/${name}".img*[!6])"
}

disk_field() {  # disk_field diskN "Field Name"
    diskutil info "$1" | sed -n "s/^ *$2: *//p" | head -n 1
}

cmd_flash() {
    [ "$(uname -s)" = Darwin ] || die "flash is for macOS (on Linux use rpi-imager or dd yourself)"
    [ $# -ge 2 ] || die "flash needs IMAGE DISK (e.g. target/pi-image/x.img.xz disk4)"
    local image="$1" disk="${2#/dev/}" yes=0
    shift 2
    [ "${1:-}" = --yes ] && yes=1
    [ -f "$image" ] || die "no image ${image}"
    [[ "$disk" =~ ^disk[0-9]+$ ]] || die "DISK must be a whole disk like disk4 (see: diskutil list external)"

    local sum="${image}.sha256"
    if [ -f "$sum" ]; then
        (cd "$(dirname "$image")" && shasum -a 256 -c "$(basename "$sum")" >/dev/null) || die "${image} fails its sha256"
        log "sha256 ok"
    fi

    diskutil info "$disk" >/dev/null 2>&1 || die "no disk ${disk}"
    [ "$(disk_field "$disk" "Part of Whole")" = "$disk" ] || die "${disk} is not a whole disk"
    local boot_whole
    boot_whole="$(disk_field / "Part of Whole")"
    [ "$disk" != "$boot_whole" ] || die "${disk} is this Mac's boot disk"
    local location removable bytes
    location="$(disk_field "$disk" "Device Location")"
    removable="$(disk_field "$disk" "Removable Media")"
    bytes="$(disk_field "$disk" "Disk Size" | sed -n 's/.*(\([0-9]*\) Bytes).*/\1/p')"
    [ "$location" = External ] || [ "$removable" = Removable ] \
        || die "${disk} is ${location:-?}/${removable:-?}: refusing anything but a removable or external disk"
    [[ "$bytes" =~ ^[0-9]+$ ]] || die "could not read the size of ${disk}"
    [ "$bytes" -le $((512 * 1000 * 1000 * 1000)) ] || die "${disk} is over 512 GB: refusing"

    diskutil list "$disk"
    if [ "$yes" = 0 ]; then
        local answer
        printf 'Erase %s (%s, %s bytes) and write %s? Type the disk id to continue: ' \
            "$disk" "$(disk_field "$disk" "Device / Media Name")" "$bytes" "$(basename "$image")"
        read -r answer
        [ "$answer" = "$disk" ] || die "not confirmed"
    fi
    diskutil unmountDisk "/dev/${disk}" >/dev/null
    log "writing (sudo dd to /dev/r${disk})"
    case "$image" in
        *.xz) xz -dc "$image" | sudo dd of="/dev/r${disk}" bs=4m ;;
        *) sudo dd if="$image" of="/dev/r${disk}" bs=4m ;;
    esac
    sync
    diskutil mountDisk "/dev/${disk}" >/dev/null 2>&1 || true
    log "written. Seed it next: scripts/pi-image.sh seed /Volumes/bootfs --hostname ... (then eject)"
}

cmd_seed() {
    local py
    py="$(find_python)" || die "no python3 >= 3.10 found (set PYTHON=)"
    exec "$py" "${repo}/deploy/pi/image/seed.py" "$@"
}

[ $# -ge 1 ] || { usage; exit 2; }
sub="$1"; shift
case "$sub" in
    build) cmd_build "$@" ;;
    flash) cmd_flash "$@" ;;
    seed) cmd_seed "$@" ;;
    -h | --help | help) usage ;;
    *) die "unknown command ${sub} (build | flash | seed)" ;;
esac
