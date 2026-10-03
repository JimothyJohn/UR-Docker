#!/usr/bin/env bash
# Build a flashable pick PC image ON an arm64 Debian box (the pick PC itself is fine): the
# pinned Raspberry Pi OS Lite image with librealsense, the perceptronics release, the
# service, the udev rules and the firewall already installed by deploy/pi/install.sh --image.
# scripts/pi-image.sh build copies this over and runs it for you.
#
#   sudo ./build.sh --wheel perceptronics-X-py3-none-any.whl --out DIR [--cache DIR]
#                   [--cell ur3] [--robot-host 192.168.3.3] [--allow-from CIDR]
#                   [--source-rev GITSHA] [--grow-mib 3072] [--no-xz]
#
# Native chroot, no qemu: the build host must be aarch64. Nothing in the image is started.
# What makes each board itself (host name, user + SSH key, cell address, which robot) is
# NOT in the image: it is the cloud-init seed on the boot partition, written after flashing
# by scripts/pi-image.sh seed. So one image serves every board.
#
# --cache keeps the base download and a tarball of the built librealsense, so a rebuild
# skips the 10-minute compile (install.sh still checks the tarball's build stamp).
# Output: DIR/perceptronics-pickpc-<date>-<rev>.img.xz + .sha256 + .manifest.txt
set -euo pipefail

# ---- pins ----------------------------------------------------------------------------
# The release the first Pi 5 was verified on (2026-10-02: /etc/rpi-issue on pickpc says
# "Raspberry Pi reference 2026-09-15"); sha256 from downloads.raspberrypi.com's .sha256 file.
readonly BASE_NAME="2026-09-15-raspios-trixie-arm64-lite.img.xz"
readonly BASE_URL="https://downloads.raspberrypi.com/raspios_lite_arm64/images/raspios_lite_arm64-2026-09-15/${BASE_NAME}"
readonly BASE_SHA256="cdf4f3bfac35ae947b46e4e767f935453810549779ac3290e05a6754aee627e5"
readonly LIBRS_DIR="librealsense-2.58.4"
readonly MARGIN_MIB=256

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly HERE
readonly DEPLOY_DIR="${HERE}/.."

log() { printf '[image %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '[image] ERROR: %s\n' "$*" >&2; exit 1; }
usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"; }

wheel="" out="" cache="/var/cache/perceptronics-image" cell="ur3" robot_host="" allow_from=""
source_rev="unknown" grow_mib=3072 compress=1
while [ $# -gt 0 ]; do
    case "$1" in
        --wheel) wheel="${2:?--wheel needs a path}"; shift 2 ;;
        --out) out="${2:?--out needs a directory}"; shift 2 ;;
        --cache) cache="${2:?--cache needs a directory}"; shift 2 ;;
        --cell) cell="${2:?--cell needs a name}"; shift 2 ;;
        --robot-host) robot_host="${2:?--robot-host needs an address}"; shift 2 ;;
        --allow-from) allow_from="${2:?--allow-from needs a CIDR}"; shift 2 ;;
        --source-rev) source_rev="${2:?--source-rev needs a value}"; shift 2 ;;
        --grow-mib) grow_mib="${2:?--grow-mib needs a number}"; shift 2 ;;
        --no-xz) compress=0; shift ;;
        -h | --help) usage; exit 0 ;;
        *) die "unknown argument: $1 (see --help)" ;;
    esac
done
[ "$(id -u)" -eq 0 ] || die "run as root"
[ "$(uname -m)" = aarch64 ] || die "build on an aarch64 host (native chroot, no emulation)"
[ -f "$wheel" ] || die "--wheel is required and must exist"
[ -n "$out" ] || die "--out is required"
[[ "$grow_mib" =~ ^[0-9]+$ ]] || die "--grow-mib: expected a number of MiB"
[[ "$source_rev" =~ ^[A-Za-z0-9._+-]+$ ]] || die "--source-rev: letters, digits and ._+- only"
for tool in losetup sfdisk e2fsck resize2fs xz curl chroot sha256sum; do
    command -v "$tool" >/dev/null || die "${tool} not found (PATH=${PATH})"
done

mkdir -p "$out" "$cache"
work="$(mktemp -d "${cache}/work.XXXXXX")"
img="${work}/image.img"
root="${work}/root"
loopdev=""
mounted=()

cleanup() {
    local i
    for ((i = ${#mounted[@]} - 1; i >= 0; i--)); do
        umount "${mounted[$i]}" 2>/dev/null || umount -l "${mounted[$i]}" 2>/dev/null || true
    done
    mounted=()
    if [ -n "$loopdev" ]; then losetup -d "$loopdev" 2>/dev/null || true; fi
    loopdev=""
    rm -rf "$work"
}
trap cleanup EXIT

mnt() {  # mnt [mount options...] SRC DEST
    mount "$@"
    mounted+=("${*: -1}")
}

# ---- base image ----------------------------------------------------------------------
base="${cache}/${BASE_NAME}"
if [ ! -f "$base" ] || [ "$(sha256sum "$base" | cut -d' ' -f1)" != "$BASE_SHA256" ]; then
    log "downloading ${BASE_NAME}"
    curl -fsSL --retry 3 -o "${base}.part" "$BASE_URL"
    mv "${base}.part" "$base"
fi
[ "$(sha256sum "$base" | cut -d' ' -f1)" = "$BASE_SHA256" ] || die "${BASE_NAME}: sha256 mismatch, refusing it"
log "base ${BASE_NAME} (sha256 ok)"

log "decompressing and growing root by ${grow_mib} MiB"
xz -dc "$base" >"$img"
truncate -s "+${grow_mib}M" "$img"
echo ", +" | sfdisk -q -N 2 "$img"
loopdev="$(losetup -fP --show "$img")"
e2fsck -fy "${loopdev}p2" >/dev/null || [ $? -le 1 ] || die "e2fsck failed on the grown root"
resize2fs "${loopdev}p2" >/dev/null

mkdir -p "$root"
mnt "${loopdev}p2" "$root"
mnt "${loopdev}p1" "${root}/boot/firmware"
for d in dev dev/pts proc sys; do
    mnt --bind "/${d}" "${root}/${d}"
done
# The librealsense build happens on the host's disk, not in the image.
mkdir -p "${cache}/build" "${root}/var/tmp/perceptronics-build"
mnt --bind "${cache}/build" "${root}/var/tmp/perceptronics-build"

# No service may start inside the chroot (apt postinst scripts ask policy-rc.d).
printf '#!/bin/sh\nexit 101\n' >"${root}/usr/sbin/policy-rc.d"
chmod 0755 "${root}/usr/sbin/policy-rc.d"
cp "${root}/etc/resolv.conf" "${work}/resolv.conf.image"
cp -L /etc/resolv.conf "${root}/etc/resolv.conf"

# ---- librealsense from the cache --------------------------------------------------------
librs_tar="${cache}/${LIBRS_DIR}.tar"
if [ -f "$librs_tar" ]; then
    log "restoring ${LIBRS_DIR} from the cache (install.sh re-checks its build stamp)"
    tar -C "${root}/opt" -xf "$librs_tar"
fi

# ---- install, as on a live PC ------------------------------------------------------------
stage="${root}/tmp/perceptronics-image"
mkdir -p "$stage"
cp "${DEPLOY_DIR}"/install.sh "${DEPLOY_DIR}"/perceptronics-cockpit.service "${DEPLOY_DIR}"/nftables.conf \
    "${DEPLOY_DIR}"/cell.env.template "${DEPLOY_DIR}"/perceptronics-doctor "${DEPLOY_DIR}"/README.md "$stage/"
cp "$wheel" "$stage/"
install_args=(--image --wheel "/tmp/perceptronics-image/$(basename "$wheel")" --cell "$cell")
[ -n "$robot_host" ] && install_args+=(--robot-host "$robot_host")
[ -n "$allow_from" ] && install_args+=(--allow-from "$allow_from")
log "install.sh ${install_args[*]}"
chroot "$root" /usr/bin/env -i PATH=/usr/sbin:/usr/bin:/sbin:/bin HOME=/root LANG=C.UTF-8 \
    bash /tmp/perceptronics-image/install.sh "${install_args[@]}"

if [ ! -f "$librs_tar" ] || ! tar -xOf "$librs_tar" "${LIBRS_DIR}/.perceptronics-build-stamp" 2>/dev/null \
    | cmp -s - "${root}/opt/${LIBRS_DIR}/.perceptronics-build-stamp"; then
    log "caching ${LIBRS_DIR} for the next build"
    tar -C "${root}/opt" -cf "${librs_tar}.part" "$LIBRS_DIR"
    mv "${librs_tar}.part" "$librs_tar"
fi

# ---- what the image is, readable on every board it becomes -------------------------------
stamp="$(date -u +%Y%m%d)-${source_rev:0:12}"
name="perceptronics-pickpc-${stamp}"
{
    echo "IMAGE=${name}"
    echo "BUILT_UTC=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    echo "SOURCE_REV=${source_rev}"
    echo "BASE=${BASE_NAME}"
    echo "BASE_SHA256=${BASE_SHA256}"
    echo "WHEEL=$(basename "$wheel")"
    echo "WHEEL_SHA256=$(sha256sum "$wheel" | cut -d' ' -f1)"
    echo "LIBREALSENSE=$(cat "${root}/opt/${LIBRS_DIR}/.perceptronics-build-stamp")"
    echo "DEFAULT_CELL=${cell}"
} >"${work}/image-release"
install -m 0644 "${work}/image-release" "${root}/etc/perceptronics/image-release"

# ---- generalise: nothing from this build may identify a board ------------------------------
log "cleaning the image"
chroot "$root" apt-get clean
rm -rf "${root}/var/lib/apt/lists/"* "$stage" "${root}/usr/sbin/policy-rc.d"
cp "${work}/resolv.conf.image" "${root}/etc/resolv.conf"
rm -f "${root}"/etc/ssh/ssh_host_* "${root}/root/.bash_history"
# Raspberry Pi OS ships machine-id as "uninitialized" (first-boot generation); keep it so.
[ "$(cat "${root}/etc/machine-id")" = uninitialized ] || die "machine-id was set during the build"
find "${root}/var/log" -type f -name '*.log' -exec truncate -s 0 {} +

dpkg_list="${work}/packages.txt"
# shellcheck disable=SC2016  # dpkg-query's own format, not shell
chroot "$root" dpkg-query -W -f='${Package} ${Version}\n' >"$dpkg_list"
df_line="$(df -h --output=size,used,avail "$root" | tail -n 1)"

for ((i = ${#mounted[@]} - 1; i >= 0; i--)); do umount "${mounted[$i]}"; done
mounted=()

# ---- shrink: the first boot's `resize` (cmdline.txt) grows root to the card ------------------
log "shrinking root to its contents + ${MARGIN_MIB} MiB"
e2fsck -fy "${loopdev}p2" >/dev/null || [ $? -le 1 ] || die "e2fsck failed before the shrink"
resize2fs -M "${loopdev}p2" >/dev/null 2>&1
block_size="$(dumpe2fs -h "${loopdev}p2" 2>/dev/null | awk -F: '/^Block size/ {gsub(/ /, "", $2); print $2}')"
block_count="$(dumpe2fs -h "${loopdev}p2" 2>/dev/null | awk -F: '/^Block count/ {gsub(/ /, "", $2); print $2}')"
fs_bytes=$((block_size * block_count + MARGIN_MIB * 1024 * 1024))
resize2fs "${loopdev}p2" "$((fs_bytes / 1024))K" >/dev/null 2>&1
losetup -d "$loopdev"
loopdev=""
start_sector="$(sfdisk -d "$img" | awk -F'[=,]' '/image.img2/ {gsub(/ /, "", $2); print $2}')"
[[ "$start_sector" =~ ^[0-9]+$ ]] || die "could not read the root partition's start sector"
part_sectors=$(((fs_bytes + 511) / 512))
echo "${start_sector}, ${part_sectors}" | sfdisk -q -N 2 "$img"
truncate -s "$(((start_sector + part_sectors) * 512))" "$img"

# ---- out -----------------------------------------------------------------------------------
manifest="${out}/${name}.manifest.txt"
{
    cat "${work}/image-release"
    echo "ROOT_BEFORE_SHRINK=${df_line}"
    echo "IMAGE_BYTES=$(stat -c %s "$img")"
    echo "--- packages"
    cat "$dpkg_list"
} >"$manifest"
if [ "$compress" = 1 ]; then
    log "compressing (xz)"
    xz -T0 -6 -c "$img" >"${out}/${name}.img.xz.part"
    mv "${out}/${name}.img.xz.part" "${out}/${name}.img.xz"
    (cd "$out" && sha256sum "${name}.img.xz" >"${name}.img.xz.sha256")
    log "wrote ${out}/${name}.img.xz ($(du -h "${out}/${name}.img.xz" | cut -f1))"
else
    mv "$img" "${out}/${name}.img"
    (cd "$out" && sha256sum "${name}.img" >"${name}.img.sha256")
    log "wrote ${out}/${name}.img"
fi
