#!/usr/bin/env bash
# The pick PC's installer: a Raspberry-Pi-class arm64 box (Debian bookworm/trixie, no
# desktop, no GPU) that runs the RealSense cockpit headless next to a UR e-Series.
# Run as root ON THE PC; scripts/deploy-pi.sh copies it over and runs it for you.
#
#   sudo ./install.sh --wheel ur_docker-0.1.0-py3-none-any.whl [--cell ur3] \
#                     [--robot-host 192.168.3.3] [--allow-from 192.168.3.0/24] [--reconfigure]
#                     [--cell-if eth0|none] [--cell-address 192.168.3.20/24]
#   sudo ./install.sh --wheel ... --image        # into an image root under chroot (no live system)
#   sudo /opt/perceptronics/deploy/install.sh --rollback        # back to the previous release
#   sudo /opt/perceptronics/deploy/install.sh --network --cell-address 10.20.0.50/24 \
#        --robot-host 10.20.0.10 [--gateway 10.20.0.1|none] [--dns 10.20.0.1|none] [--cell-dhcp auto|off]
#        # what the setup portal (http://<PC>:7621/setup) runs through perceptronics-admin
#   sudo /opt/perceptronics/deploy/install.sh --uninstall [--purge]
#
# Idempotent: re-running with the same wheel installs nothing new (the service is only
# restarted); librealsense is rebuilt only when its pinned tag/commit/options change;
# /etc/perceptronics/cell.env is written only when missing or with --reconfigure (the old
# one is kept as cell.env.<timestamp>).
#
# Long-lived process this installs: perceptronics-cockpit.service, the cockpit on TCP :7621 and
# the PolyScope Pick node's pick server on TCP :7622.
#   stop:  sudo systemctl stop perceptronics-cockpit     logs: journalctl -u perceptronics-cockpit -f
# A `perceptronics pick-server` sidecar also binds :7622 — stop it before (re)starting the unit.
#
# The cell port (--cell-if, default eth0) is set up for a robot nobody has configured: it
# holds --cell-address (default 192.168.3.20/24, what the URCap's empty Cockpit field means)
# and perceptronics-cell-dhcp.service hands a robot on DHCP the cell's UR_HOST (192.168.3.3)
# — but only when no other DHCP server answers there. A port already on another network
# (the bench's office LAN) is left alone. --cell-if none skips all of it.
#   stop:  sudo systemctl disable --now perceptronics-cell-dhcp   logs: journalctl -u perceptronics-cell-dhcp
# The network settings in use are saved to /etc/perceptronics/network.env and are the defaults
# of the next run, so an update (or a redeploy without the flags) keeps what the setup portal
# or an earlier --cell-if / --cell-address chose. --network forces them onto the cell port
# (the plain install never re-addresses a configured port) and keeps 192.168.3.20/24 on it as a
# second, rescue address unless the new network holds that address: a mistyped plant network
# never locks the operator out of the setup page.
#
# Long-lived units this also installs: perceptronics-admin.path, which starts
# perceptronics-admin.service (root, oneshot: /usr/local/sbin/perceptronics-admin run) when
# the cockpit queues a setup-portal request or an update bundle.
#   stop:  sudo systemctl disable --now perceptronics-admin.path   logs: journalctl -u perceptronics-admin
#
# --image: the root is an image being built (deploy/pi/image/build.sh runs this under chroot),
# not a running PC. Units are enabled but nothing is started or reloaded: no udev reload (no
# udevd in a chroot), and no firewall load, which would land in the BUILD host's kernel.
set -euo pipefail

IMAGE=0

# ---- pins ----------------------------------------------------------------------------
# perceptronics/realsense.py binds the C API with ctypes and checks enum ordinals written
# against librealsense 2.58 (`_check_enums`); v2.58.4 is also what Dockerfile.perceptronics
# builds. It was the newest release tag on 2026-09-28 (`git ls-remote --tags`); the commit
# is checked after the clone so a moved tag cannot slip a different tree in.
readonly LIBREALSENSE_TAG="v2.58.4"
readonly LIBREALSENSE_COMMIT="34d6c778e1134d8505adcd56bb57acbe7598a459"
readonly LIBREALSENSE_REPO="https://github.com/IntelRealSense/librealsense.git"
readonly LIBREALSENSE_PREFIX="/opt/librealsense-${LIBREALSENSE_TAG#v}"
readonly LIBREALSENSE_LINK="/opt/librealsense"
# RSUSB = librealsense's own libusb UVC backend: no kernel patches on a stock Pi kernel.
# Graphical examples OFF also forces CHECK_FOR_UPDATES off (CMake/global_config.cmake), so no
# OpenSSL/libcurl; OFF is passed explicitly anyway. Python bindings OFF: we use ctypes.
readonly LIBREALSENSE_CMAKE_OPTS=(
    -DCMAKE_BUILD_TYPE=Release
    -DFORCE_RSUSB_BACKEND=ON
    -DBUILD_EXAMPLES=OFF
    -DBUILD_GRAPHICAL_EXAMPLES=OFF
    -DBUILD_TOOLS=OFF
    -DBUILD_PYTHON_BINDINGS=OFF
    -DBUILD_WITH_CUDA=OFF
    -DBUILD_UNIT_TESTS=OFF
    -DCHECK_FOR_UPDATES=OFF
)

# ---- layout --------------------------------------------------------------------------
readonly APP_ROOT="/opt/perceptronics"
readonly RELEASES="${APP_ROOT}/releases"
readonly CURRENT="${APP_ROOT}/current"
readonly PREVIOUS="${APP_ROOT}/previous"
readonly DEPLOY_COPY="${APP_ROOT}/deploy"
readonly ETC_DIR="/etc/perceptronics"
readonly CELL_ENV="${ETC_DIR}/cell.env"
readonly STATE_DIR="/var/lib/perceptronics"
readonly SVC_USER="perceptronics"
readonly UNIT="perceptronics-cockpit.service"
readonly UDEV_RULES="/etc/udev/rules.d/99-realsense-libusb.rules"
readonly LDCONF="/etc/ld.so.conf.d/librealsense.conf"
readonly NFT_CONF="/etc/nftables.conf"
readonly NFT_BACKUP="/etc/nftables.conf.pre-perceptronics"
readonly NFT_MARKER="# perceptronics-cockpit firewall"
readonly BUILD_ROOT="/var/tmp/perceptronics-build"
readonly CELL_DHCP_UNIT="perceptronics-cell-dhcp.service"
readonly CELL_DHCP_CONF="${ETC_DIR}/cell-dhcp.conf"
readonly NM_HOOK="/etc/NetworkManager/dispatcher.d/50-perceptronics-cell"
readonly NM_CELL_CON="perceptronics-cell"
readonly KEEP_RELEASES=3
readonly NETWORK_ENV="${ETC_DIR}/network.env"
readonly RESCUE_CIDR="192.168.3.20/24"
readonly ADMIN_BIN="/usr/local/sbin/perceptronics-admin"
readonly ADMIN_PATH_UNIT="perceptronics-admin.path"
readonly ADMIN_UNIT="perceptronics-admin.service"
readonly ADMIN_QUEUE="${STATE_DIR}/admin/queue"
readonly ADMIN_STATE="/var/lib/perceptronics-admin"

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly HERE

log() { printf '[install %s] %s\n' "$(date +%H:%M:%S)" "$*"; }
die() { printf '[install] ERROR: %s\n' "$*" >&2; exit 1; }

usage() { awk 'NR > 1 && /^#/ { sub(/^# ?/, ""); print; next } NR > 1 { exit }' "${BASH_SOURCE[0]}"; }

# ---- apt -----------------------------------------------------------------------------
# Each package is here for one reason:
#   python3, python3-venv  the runtime (stdlib only, >= 3.10) and `python3 -m venv` (ensurepip)
#   git, ca-certificates   clone librealsense at the pinned tag over HTTPS; its CMake also
#                          fetches nlohmann/json, fastcdr, yaml-cpp and sqlite at build time
#   cmake, build-essential compile librealsense (build-essential = gcc/g++/make)
#   pkg-config             listed by librealsense's doc/installation.md for its CMake probes
#   libusb-1.0-0-dev       the RSUSB backend links the system libusb (else CMake clones one)
#   libudev-dev            event-driven USB hotplug in the SDK (installation.md: "optional but
#                          recommended") — the cockpit re-opens a camera that dropped out
#   nftables               the inbound firewall below
#   usbutils               `lsusb`, the deploy skill's "is the D435 (8086:0b07) there?" check
#   dnsmasq-base           the cell port's one-lease DHCP server (only the binary; already
#                          on Raspberry Pi OS Lite trixie, where NetworkManager pulls it in)
readonly APT_PACKAGES=(
    python3 python3-venv
    git ca-certificates
    cmake build-essential pkg-config
    libusb-1.0-0-dev libudev-dev
    nftables usbutils
    dnsmasq-base
)

apt_install() {
    local missing=()
    local p
    for p in "${APT_PACKAGES[@]}"; do
        dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed" || missing+=("$p")
    done
    if [ "${#missing[@]}" -eq 0 ]; then
        log "apt: all ${#APT_PACKAGES[@]} packages present"
        return
    fi
    log "apt: installing ${missing[*]}"
    DEBIAN_FRONTEND=noninteractive apt-get update -q
    DEBIAN_FRONTEND=noninteractive apt-get install -y -q --no-install-recommends "${missing[@]}"
}

check_python() {
    python3 -c 'import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)' \
        || die "python3 >= 3.10 required (found $(python3 -V 2>&1))"
}

# ---- librealsense ----------------------------------------------------------------------
build_stamp() {
    printf '%s %s %s\n' "$LIBREALSENSE_TAG" "$LIBREALSENSE_COMMIT" "${LIBREALSENSE_CMAKE_OPTS[*]}"
}

mem_mib() { awk -v k="$1" '$1 == k":" {print int($2 / 1024)}' /proc/meminfo; }

SWAPFILE=""
cleanup_swap() {
    if [ -n "$SWAPFILE" ] && [ -f "$SWAPFILE" ]; then
        swapoff "$SWAPFILE" 2>/dev/null || true
        rm -f "$SWAPFILE"
        log "removed the temporary build swapfile"
    fi
}

# A 2 GB Pi runs out of memory compiling librealsense in parallel. Size the job count to
# RAM (~1.5 GiB per compiler) and, when RAM + swap is under 4 GiB, add a temporary
# swapfile for the build only (removed afterwards, also on failure). Sets JOBS.
JOBS=1
prepare_build_memory() {
    local ram swap cpus need
    ram="$(mem_mib MemTotal)"
    swap="$(mem_mib SwapTotal)"
    cpus="$(nproc)"
    if [ $((ram + swap)) -lt 4096 ]; then
        need=$((4096 - ram - swap))
        [ "$need" -lt 1024 ] && need=1024
        log "RAM ${ram} MiB + swap ${swap} MiB < 4 GiB: adding a ${need} MiB swapfile for the build"
        SWAPFILE="${BUILD_ROOT}/build.swap"
        fallocate -l "${need}M" "$SWAPFILE" || dd if=/dev/zero of="$SWAPFILE" bs=1M count="$need" status=none
        chmod 600 "$SWAPFILE"
        mkswap "$SWAPFILE" >/dev/null
        swapon "$SWAPFILE"
    fi
    JOBS=$((ram / 1536))
    [ "$JOBS" -lt 1 ] && JOBS=1
    [ "$JOBS" -gt "$cpus" ] && JOBS="$cpus"
    return 0
}

install_librealsense() {
    local stamp_file="${LIBREALSENSE_PREFIX}/.perceptronics-build-stamp"
    if [ -f "$stamp_file" ] && [ "$(cat "$stamp_file")" = "$(build_stamp)" ] \
        && [ -e "${LIBREALSENSE_PREFIX}/lib/librealsense2.so" ]; then
        log "librealsense ${LIBREALSENSE_TAG}: already built (${LIBREALSENSE_PREFIX})"
    else
        local free_kib
        mkdir -p "$BUILD_ROOT"
        free_kib="$(df -Pk "$BUILD_ROOT" | awk 'NR == 2 {print $4}')"
        [ "$free_kib" -ge $((5 * 1024 * 1024)) ] || die "need >= 5 GiB free under ${BUILD_ROOT} to build librealsense"
        local src="${BUILD_ROOT}/librealsense-${LIBREALSENSE_TAG}"
        rm -rf "$src" "${src}-build"
        log "librealsense ${LIBREALSENSE_TAG}: cloning"
        git -c advice.detachedHead=false clone -q --depth 1 --branch "$LIBREALSENSE_TAG" "$LIBREALSENSE_REPO" "$src"
        local got
        got="$(git -C "$src" rev-parse HEAD)"
        [ "$got" = "$LIBREALSENSE_COMMIT" ] \
            || die "librealsense ${LIBREALSENSE_TAG} is commit ${got}, expected ${LIBREALSENSE_COMMIT} — refusing to build"
        trap cleanup_swap EXIT
        prepare_build_memory
        log "librealsense: configuring + building with -j${JOBS} (tens of minutes on a Pi 5)"
        cmake -S "$src" -B "${src}-build" "${LIBREALSENSE_CMAKE_OPTS[@]}" \
            -DCMAKE_INSTALL_PREFIX="$LIBREALSENSE_PREFIX" >"${BUILD_ROOT}/cmake-configure.log" 2>&1 \
            || die "cmake configure failed — see ${BUILD_ROOT}/cmake-configure.log"
        cmake --build "${src}-build" -j"$JOBS" --target realsense2 >"${BUILD_ROOT}/cmake-build.log" 2>&1 \
            || die "librealsense build failed — see ${BUILD_ROOT}/cmake-build.log"
        rm -rf "$LIBREALSENSE_PREFIX"
        cmake --install "${src}-build" >"${BUILD_ROOT}/cmake-install.log" 2>&1 \
            || die "cmake install failed — see ${BUILD_ROOT}/cmake-install.log"
        install -m 0644 "${src}/config/99-realsense-libusb.rules" "${LIBREALSENSE_PREFIX}/99-realsense-libusb.rules"
        build_stamp >"$stamp_file"
        cleanup_swap
        trap - EXIT
        rm -rf "$src" "${src}-build"
        log "librealsense ${LIBREALSENSE_TAG}: installed to ${LIBREALSENSE_PREFIX}"
    fi
    ln -sfn "$LIBREALSENSE_PREFIX" "$LIBREALSENSE_LINK"
    echo "${LIBREALSENSE_LINK}/lib" >"$LDCONF"
    ldconfig
    # The SDK's own rules (MODE 0666, GROUP plugdev for every RealSense PID incl. the D435's
    # 0b07): with them a normal user opens the camera over libusb — no root in the service.
    if ! cmp -s "${LIBREALSENSE_PREFIX}/99-realsense-libusb.rules" "$UDEV_RULES"; then
        install -m 0644 "${LIBREALSENSE_PREFIX}/99-realsense-libusb.rules" "$UDEV_RULES"
        if [ "$IMAGE" = 0 ]; then
            udevadm control --reload-rules
            udevadm trigger --subsystem-match=usb
        fi
        log "udev: installed ${UDEV_RULES} (re-plug the camera if it was already attached)"
    fi
}

# ---- service user ----------------------------------------------------------------------
ensure_user() {
    local g
    for g in plugdev video; do
        getent group "$g" >/dev/null || groupadd --system "$g"
    done
    if ! id "$SVC_USER" >/dev/null 2>&1; then
        useradd --system --user-group --home-dir "$STATE_DIR" --no-create-home \
            --shell /usr/sbin/nologin "$SVC_USER"
        log "created system user ${SVC_USER}"
    fi
    usermod -a -G plugdev,video "$SVC_USER"
    install -d -o "$SVC_USER" -g "$SVC_USER" -m 0750 "$STATE_DIR" \
        "${STATE_DIR}/captures" "${STATE_DIR}/captures/calibration"
}

# ---- the app: one venv per wheel, `current` / `previous` symlinks -------------------------
install_app() {
    local wheel="$1"
    [ -f "$wheel" ] || die "wheel not found: ${wheel}"
    case "$(basename "$wheel")" in
        *-py3-none-any.whl) ;;
        *) die "expected a pure-Python wheel (*-py3-none-any.whl), got $(basename "$wheel")" ;;
    esac
    local version sha id dest
    version="$(basename "$wheel" | cut -d- -f2)"
    sha="$(sha256sum "$wheel" | cut -c1-12)"
    id="${version}-${sha}"
    dest="${RELEASES}/${id}"
    mkdir -p "$RELEASES" "${APP_ROOT}/wheels"
    if [ -x "${dest}/bin/perceptronics" ] && [ -f "${dest}/.complete" ]; then
        log "app: release ${id} already installed"
    else
        log "app: installing release ${id}"
        rm -rf "$dest"
        python3 -m venv "$dest"
        # --no-index --no-deps: the runtime is stdlib-only, so nothing is fetched on the PC.
        if ! PIP_DISABLE_PIP_VERSION_CHECK=1 "${dest}/bin/pip" install -q --no-index --no-deps "$wheel"; then
            rm -rf "$dest"
            die "pip could not install ${wheel}"
        fi
        touch "${dest}/.complete"
    fi
    cp -f "$wheel" "${APP_ROOT}/wheels/"
    local now=""
    [ -L "$CURRENT" ] && now="$(readlink -f "$CURRENT")"
    if [ "$now" != "$dest" ]; then
        [ -n "$now" ] && ln -sfn "$now" "$PREVIOUS"
        ln -sfn "$dest" "$CURRENT"
        log "app: current -> ${id}${now:+ (previous -> $(basename "$now"))}"
    fi
    prune_releases
}

prune_releases() {
    local keep_cur keep_prev r n=0
    keep_cur="$(readlink -f "$CURRENT" 2>/dev/null || true)"
    keep_prev="$(readlink -f "$PREVIOUS" 2>/dev/null || true)"
    # newest first; keep current, previous and up to KEEP_RELEASES in total
    while IFS= read -r r; do
        n=$((n + 1))
        if [ "$r" != "$keep_cur" ] && [ "$r" != "$keep_prev" ] && [ "$n" -gt "$KEEP_RELEASES" ]; then
            rm -rf "$r"
            log "app: pruned old release $(basename "$r")"
        fi
    done < <(find "$RELEASES" -mindepth 1 -maxdepth 1 -type d ! -name '*.tmp' -printf '%T@ %p\n' | sort -rn | cut -d' ' -f2-)
}

# ---- /etc/perceptronics/cell.env ----------------------------------------------------------
# The shipped cell profile (perceptronics/cells/<cell>.env, read with the package's own parser)
# minus the lines that describe another computer (the Mac's webcam names), plus this PC's
# lines from cell.env.template, plus --robot-host. One flat KEY=VALUE file that both systemd
# (EnvironmentFile=) and `perceptronics --cell /etc/perceptronics/cell.env` read the same way.
write_cell_env() {
    local cell="$1" robot_host="$2"
    local py="${CURRENT}/bin/python"
    mkdir -p "$ETC_DIR"
    if [ -f "$CELL_ENV" ]; then
        cp -p "$CELL_ENV" "${CELL_ENV}.$(date +%Y%m%d-%H%M%S)"
    fi
    "$py" - "$cell" "$robot_host" "${HERE}/cell.env.template" "${CELL_ENV}.new" <<'PY'
import sys
from perceptronics.cell import load_cell, parse_env_text

cell, robot_host, template, out = sys.argv[1:5]
# Host-specific to the Mac Studio the shipped cells were written on: webcams by
# AVFoundation name and their focus lock. A Pi with extra webcams sets PERCEPTRONICS_VIEWS
# to /dev/videoN by hand.
DROP = {"PERCEPTRONICS_VIEWS", "PERCEPTRONICS_VIEW_FOCUS"}
try:
    values = {k: v for k, v in load_cell(cell).items() if k not in DROP}
except ValueError as exc:
    sys.exit(f"--cell: {exc}")
with open(template, encoding="utf-8") as fh:
    values.update(parse_env_text(fh.read()))
if robot_host:
    values["UR_HOST"] = robot_host
if not values.get("UR_HOST"):
    sys.exit(f"cell {cell!r} has no UR_HOST: pass --robot-host <controller IP>")
bad = set('"\'\\$`#\n\r')
lines = [
    "# /etc/perceptronics/cell.env - written by /opt/perceptronics/deploy/install.sh",
    f"# from the shipped cell {cell!r} + cell.env.template. Read by perceptronics-cockpit.service",
    "# (EnvironmentFile=) and by `perceptronics --cell /etc/perceptronics/cell.env ...`.",
    "# The hand-eye is not here: install.sh moves the profile's PERCEPTRONICS_T_FLANGE_CAMERA into",
    "# PERCEPTRONICS_HANDEYE_FILE, which `perceptronics calibrate --apply` replaces (an environment",
    "# value would win over every calibration made on this PC).",
]
for key, value in values.items():
    if bad & set(value):
        sys.exit(f"{key}: value {value!r} has a character systemd and the cell parser read differently")
    lines.append(f"{key}={value}")
with open(out, "w", encoding="utf-8") as fh:
    fh.write("\n".join(lines) + "\n")
PY
    chown root:"$SVC_USER" "${CELL_ENV}.new"
    chmod 0640 "${CELL_ENV}.new"
    mv "${CELL_ENV}.new" "$CELL_ENV"
    log "wrote ${CELL_ENV} (cell ${cell})"
}

cell_value() { sed -n "s/^$1=//p" "$CELL_ENV" | tail -n 1; }

# The hand-eye lives in PERCEPTRONICS_HANDEYE_FILE, never in cell.env: HandEye.from_env takes
# the environment first, so a PERCEPTRONICS_T_FLANGE_CAMERA line would silently undo every
# `perceptronics calibrate --apply` on this PC at the next restart. The profile's value seeds
# the file when there is none; a file already there is a calibration made here and is kept.
# Runs on every install, so a cell.env written before this rule is fixed too.
handeye_out_of_env() {
    local py="${CURRENT}/bin/python" file result
    file="$(cell_value PERCEPTRONICS_HANDEYE_FILE)"
    [ -n "$file" ] || { log "hand-eye: no PERCEPTRONICS_HANDEYE_FILE in ${CELL_ENV}: left as is"; return; }
    result="$("$py" - "$CELL_ENV" "$file" "${CELL_ENV}.new" <<'PY'
import sys
from pathlib import Path

from perceptronics.cell import without_key
from perceptronics.handeye import ENV_T_FLANGE_CAMERA, parse_pose_text, seed_calibration_file

cell_env, file, out = sys.argv[1:4]
rest, raw = without_key(Path(cell_env).read_text(encoding="utf-8"), ENV_T_FLANGE_CAMERA)
if raw is None:
    print("none")
    sys.exit(0)
wrote = seed_calibration_file(file, parse_pose_text(raw), source=f"cell profile, seeded by install.sh from {cell_env}")
Path(out).write_text(rest, encoding="utf-8")
print("seeded" if wrote else "kept")
PY
)"
    case "$result" in
        none) return ;;
        seeded) log "hand-eye: the profile's pose seeded ${file} (calibrate --apply replaces it)" ;;
        kept) log "hand-eye: kept ${file} (a calibration made on this PC); dropped the profile's pose" ;;
        *) die "hand-eye: could not move PERCEPTRONICS_T_FLANGE_CAMERA out of ${CELL_ENV}" ;;
    esac
    cp -p "$CELL_ENV" "${CELL_ENV}.$(date +%Y%m%d-%H%M%S)"
    chown root:"$SVC_USER" "${CELL_ENV}.new"
    chmod 0640 "${CELL_ENV}.new"
    mv "${CELL_ENV}.new" "$CELL_ENV"
    chown "$SVC_USER":"$SVC_USER" "$file"
    chmod 0640 "$file"
}

# ---- firewall --------------------------------------------------------------------------
cell_net() {
    local allow="$1" host
    if [ -n "$allow" ]; then
        echo "$allow"
        return
    fi
    host="$(cell_value UR_HOST)"
    if [[ "$host" =~ ^([0-9]{1,3})\.([0-9]{1,3})\.([0-9]{1,3})\.[0-9]{1,3}$ ]]; then
        echo "${BASH_REMATCH[1]}.${BASH_REMATCH[2]}.${BASH_REMATCH[3]}.0/24"
        return
    fi
    die "UR_HOST=${host:-<empty>} is not an IPv4 address: pass --allow-from <cell subnet CIDR>"
}

install_firewall() {
    local net="$1" cell_if="$2" one nft_net
    [[ "$net" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}/[0-9]{1,2}(,[0-9]{1,3}(\.[0-9]{1,3}){3}/[0-9]{1,2})*$ ]] \
        || die "--allow-from ${net}: expected an IPv4 CIDR (or several, comma-separated)"
    # several subnets (the setup portal's network + the rescue subnet) are an nft anonymous set
    nft_net="$net"
    if [[ "$net" == *,* ]]; then
        nft_net="{ "
        for one in ${net//,/ }; do nft_net+="${one}, "; done
        nft_net="${nft_net%, } }"
    fi
    local rendered
    rendered="$(mktemp)"
    sed -e "s#@CELL_NET@#${nft_net}#g" -e "s#@CELL_IF@#${cell_if}#g" "${HERE}/nftables.conf" >"$rendered"
    nft -c -f "$rendered" || die "nftables.conf failed nft's syntax check"
    if [ -f "$NFT_CONF" ] && ! grep -qF "$NFT_MARKER" "$NFT_CONF" && [ ! -f "$NFT_BACKUP" ]; then
        cp -p "$NFT_CONF" "$NFT_BACKUP"
        log "firewall: kept the original ${NFT_CONF} as ${NFT_BACKUP}"
    fi
    install -m 0644 "$rendered" "$NFT_CONF"
    rm -f "$rendered"
    systemctl enable -q nftables.service
    [ "$IMAGE" = 1 ] || systemctl restart nftables.service
    log "firewall: inbound SSH from anywhere; :7621/:7622 from ${net} only; DHCP on ${cell_if}; everything else dropped"
}

# ---- the cell port ---------------------------------------------------------------------
ip_to_int() {
    local a b c d
    IFS=. read -r a b c d <<<"$1"
    echo $(((a << 24) | (b << 16) | (c << 8) | d))
}

prefix_mask() {
    local p="$1" m
    m=$(((0xffffffff << (32 - p)) & 0xffffffff))
    echo "$((m >> 24 & 255)).$((m >> 16 & 255)).$((m >> 8 & 255)).$((m & 255))"
}

# Give the cell port its fixed address with NetworkManager. Never touches a port that is
# already on another network: on the bench that is the office LAN this install runs over.
configure_cell_address() {
    local cell_if="$1" cidr="$2" have con
    if [ "$IMAGE" = 1 ]; then
        write_cell_keyfile "$cell_if" "$cidr"
        return
    fi
    if ! command -v nmcli >/dev/null || ! nmcli -t general status >/dev/null 2>&1; then
        log "cell port: NetworkManager not running — give ${cell_if} ${cidr} (no gateway) by hand"
        return
    fi
    have="$(ip -4 -o addr show dev "$cell_if" 2>/dev/null | awk '{print $4}')"
    if printf '%s\n' "$have" | grep -qx "$cidr"; then
        log "cell port: ${cell_if} already has ${cidr}"
        return
    fi
    if [ -n "$have" ]; then
        log "cell port: ${cell_if} is on another network ($(echo "$have" | tr '\n' ' ')) — left alone;" \
            "on the cell, re-run with that port free (or set ${cidr} by hand)"
        return
    fi
    # A profile of someone else's for this port that already carries the address (the
    # bench's hand-made one): keep it rather than add a second. "For this port" = it names the
    # port, or it is an Ethernet profile that names none (netplan's `match: {}`, as Raspberry
    # Pi OS writes it: it applies to any Ethernet port).
    local bound
    while IFS=: read -r con; do
        if [ -z "$con" ] || [ "$con" = "$NM_CELL_CON" ]; then continue; fi
        bound="$(nmcli -g connection.interface-name,connection.type connection show "$con" 2>/dev/null \
            | tr '\n' ' ')"
        case "$bound" in
            "${cell_if} "* | " 802-3-ethernet "*) ;;
            *) continue ;;
        esac
        if nmcli -g ipv4.addresses connection show "$con" 2>/dev/null | grep -qF "$cidr"; then
            log "cell port: NetworkManager profile ${con} already gives ${cell_if} ${cidr} — kept"
            return
        fi
    done < <(nmcli -t -f NAME connection show)
    if nmcli -t -f NAME connection show | grep -qx "$NM_CELL_CON"; then
        nmcli connection modify "$NM_CELL_CON" connection.interface-name "$cell_if" \
            ipv4.method manual ipv4.addresses "$cidr" ipv4.gateway "" ipv4.never-default yes
    else
        nmcli connection add type ethernet con-name "$NM_CELL_CON" ifname "$cell_if" \
            connection.autoconnect yes connection.autoconnect-priority 100 \
            ipv4.method manual ipv4.addresses "$cidr" ipv4.never-default yes ipv6.method link-local >/dev/null
    fi
    if [ "$(cat "/sys/class/net/${cell_if}/carrier" 2>/dev/null)" = 1 ]; then
        nmcli connection up "$NM_CELL_CON" >/dev/null || log "cell port: ${NM_CELL_CON} did not come up — check nmcli"
    fi
    log "cell port: ${cell_if} = ${cidr} (NetworkManager profile ${NM_CELL_CON}; up when a cable is in)"
}

# ---- --network: what the setup portal changes ---------------------------------------------
in_network() {
    local ip="$1" cidr="$2" prefix="${2#*/}"
    [ $(($(ip_to_int "$ip") >> (32 - prefix))) -eq $(($(ip_to_int "${cidr%/*}") >> (32 - prefix))) ]
}

network_of() {
    local cidr="$1" prefix="${1#*/}" n
    n=$(($(ip_to_int "${cidr%/*}") & (0xffffffff << (32 - prefix)) & 0xffffffff))
    echo "$((n >> 24 & 255)).$((n >> 16 & 255)).$((n >> 8 & 255)).$((n & 255))/${prefix}"
}

set_cell_value() {
    local key="$1" value="$2"
    [ -f "$CELL_ENV" ] || die "no ${CELL_ENV} to change ${key} in (install first)"
    cp -p "$CELL_ENV" "${CELL_ENV}.$(date +%Y%m%d-%H%M%S)"
    awk -v k="$key" -v v="$value" 'BEGIN { done = 0 }
        index($0, k "=") == 1 { if (!done) print k "=" v; done = 1; next }
        { print }
        END { if (!done) print k "=" v }' "$CELL_ENV" >"${CELL_ENV}.new"
    chown root:"$SVC_USER" "${CELL_ENV}.new"
    chmod 0640 "${CELL_ENV}.new"
    mv "${CELL_ENV}.new" "$CELL_ENV"
    log "${CELL_ENV}: ${key}=${value}"
}

# Unlike configure_cell_address, this re-addresses the cell port whatever is on it: the
# operator asked for it on the setup page. Our own profile, priority 100, so it also wins over
# Raspberry Pi OS's eth0 profile on the next boot; the rescue address rides along.
force_cell_address() {
    local cell_if="$1" cidr="$2" gateway="$3" dns="$4" rescue="$5" addrs="$2"
    [ "$rescue" = 1 ] && addrs+=",${RESCUE_CIDR}"
    if ! command -v nmcli >/dev/null || ! nmcli -t general status >/dev/null 2>&1; then
        die "--network: NetworkManager is not running"
    fi
    local route=(ipv4.gateway "" ipv4.never-default yes)
    [ -n "$gateway" ] && route=(ipv4.gateway "$gateway" ipv4.never-default no)
    local settings=(connection.interface-name "$cell_if" connection.autoconnect yes
        connection.autoconnect-priority 100 ipv4.method manual ipv4.addresses "$addrs" "${route[@]}"
        ipv4.dns "$dns" ipv4.ignore-auto-dns yes ipv6.method link-local)
    if nmcli -t -f NAME connection show | grep -qx "$NM_CELL_CON"; then
        nmcli connection modify "$NM_CELL_CON" "${settings[@]}"
    else
        nmcli connection add type ethernet con-name "$NM_CELL_CON" ifname "$cell_if" "${settings[@]}" >/dev/null
    fi
    if [ "$(cat "/sys/class/net/${cell_if}/carrier" 2>/dev/null)" = 1 ]; then
        nmcli connection up "$NM_CELL_CON" >/dev/null || log "cell port: ${NM_CELL_CON} did not come up — check nmcli"
    fi
    log "cell port: ${cell_if} = ${addrs}${gateway:+, gateway ${gateway}}${dns:+, DNS ${dns}}"
}

save_network_env() {
    local cell_if="$1" cidr="$2" gateway="$3" dns="$4" cell_dhcp="$5" allow_from="$6"
    mkdir -p "$ETC_DIR"
    cat >"${NETWORK_ENV}.new" <<NETENV
# /etc/perceptronics/network.env — written by install.sh (and so by the setup portal).
# The defaults of install.sh's next run: an update keeps these network settings.
CELL_IF=${cell_if}
CELL_ADDRESS=${cidr}
GATEWAY=${gateway}
DNS=${dns}
CELL_DHCP=${cell_dhcp}
ALLOW_FROM=${allow_from}
NETENV
    chmod 0644 "${NETWORK_ENV}.new"
    mv "${NETWORK_ENV}.new" "$NETWORK_ENV"
}

# Sets the saved_* variables from network.env; every value is checked again by main's guards.
load_saved_network() {
    saved_cell_if="" saved_cell_address="" saved_gateway="" saved_dns="" saved_cell_dhcp="" saved_allow_from=""
    [ -f "$NETWORK_ENV" ] || return 0
    local key value
    while IFS='=' read -r key value; do
        case "$key" in
            CELL_IF) saved_cell_if="$value" ;;
            CELL_ADDRESS) saved_cell_address="$value" ;;
            GATEWAY) saved_gateway="$value" ;;
            DNS) saved_dns="$value" ;;
            CELL_DHCP) saved_cell_dhcp="$value" ;;
            ALLOW_FROM) saved_allow_from="$value" ;;
        esac
    done <"$NETWORK_ENV"
}

apply_network() {
    local cell_if="$1" cidr="$2" robot_host="$3" gateway="$4" dns="$5" cell_dhcp="$6" allow_from="$7"
    [ "$cell_if" != none ] || die "--network: this PC's cell port is not managed (installed with --cell-if none)"
    [ -f "$CELL_ENV" ] || die "--network: no ${CELL_ENV} (install first)"
    [ -n "$robot_host" ] && set_cell_value UR_HOST "$robot_host"
    local robot rescue=1
    robot="$(cell_value UR_HOST)"
    in_network "${RESCUE_CIDR%/*}" "$cidr" && rescue=0
    [ -n "$gateway" ] && ! in_network "$gateway" "$cidr" && die "--gateway ${gateway} is not on ${cidr}"
    force_cell_address "$cell_if" "$cidr" "$gateway" "$dns" "$rescue"
    if [ -z "$allow_from" ]; then
        # the cockpit from: this network, the rescue subnet, and a robot routed in from elsewhere
        allow_from="$(network_of "$cidr")"
        [ "$rescue" = 1 ] && allow_from+=",$(network_of "$RESCUE_CIDR")"
        [[ "$robot" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] && ! in_network "$robot" "$cidr" && allow_from+=",${robot}/32"
    fi
    install_firewall "$allow_from" "$cell_if"
    if [ "$cell_dhcp" = off ]; then
        remove_cell_dhcp
        log "cell DHCP: off"
    else
        install_cell_dhcp "$cell_if" "$cidr"
    fi
    save_network_env "$cell_if" "$cidr" "$gateway" "$dns" "$cell_dhcp" "$allow_from"
    systemctl restart "$UNIT"
    log "network: done; the cockpit restarted with UR_HOST=${robot}"
}

# The image has no running NetworkManager: write the profile nmcli would have made. Every
# board flashed from it comes up with the cell address on the cell port.
write_cell_keyfile() {
    local cell_if="$1" cidr="$2" dir=/etc/NetworkManager/system-connections
    [ -d /etc/NetworkManager ] || die "cell port: no NetworkManager in the image"
    install -d -m 0755 "$dir"
    (umask 077 && cat >"${dir}/${NM_CELL_CON}.nmconnection") <<KEYFILE
[connection]
id=${NM_CELL_CON}
type=ethernet
interface-name=${cell_if}
autoconnect=true
autoconnect-priority=100

[ipv4]
method=manual
address1=${cidr}
never-default=true

[ipv6]
method=link-local
KEYFILE
    chmod 0600 "${dir}/${NM_CELL_CON}.nmconnection"
    log "cell port: ${cell_if} = ${cidr} (image: ${dir}/${NM_CELL_CON}.nmconnection)"
}

# One DHCP lease, the cell's UR_HOST, on the cell port — started only after no other DHCP
# server answered there (perceptronics.cellnet probe).
install_cell_dhcp() {
    local cell_if="$1" cidr="$2" robot prefix
    robot="$(cell_value UR_HOST)"
    prefix="${cidr#*/}"
    if ! [[ "$robot" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] \
        || [ $(($(ip_to_int "$robot") >> (32 - prefix))) -ne $(($(ip_to_int "${cidr%/*}") >> (32 - prefix))) ]; then
        log "cell DHCP: UR_HOST=${robot:-<empty>} is not on ${cidr} — not serving (the robot needs a static address)"
        remove_cell_dhcp
        return
    fi
    local tpl
    for tpl in cell-dhcp.conf "$CELL_DHCP_UNIT" 50-perceptronics-cell; do
        sed -e "s#@CELL_IF@#${cell_if}#g" -e "s#@ROBOT_ADDRESS@#${robot}#g" \
            -e "s#@CELL_MASK@#$(prefix_mask "$prefix")#g" "${HERE}/${tpl}" >"${BUILD_ROOT}.${tpl}"
    done
    install -m 0644 "${BUILD_ROOT}.cell-dhcp.conf" "$CELL_DHCP_CONF"
    /usr/sbin/dnsmasq --test --conf-file="$CELL_DHCP_CONF" >/dev/null 2>&1 \
        || die "cell-dhcp.conf failed dnsmasq's syntax check"
    install -m 0644 "${BUILD_ROOT}.${CELL_DHCP_UNIT}" "/etc/systemd/system/${CELL_DHCP_UNIT}"
    if [ -d "$(dirname "$NM_HOOK")" ]; then
        install -m 0755 "${BUILD_ROOT}.50-perceptronics-cell" "$NM_HOOK"
    fi
    rm -f "${BUILD_ROOT}.cell-dhcp.conf" "${BUILD_ROOT}.${CELL_DHCP_UNIT}" "${BUILD_ROOT}.50-perceptronics-cell"
    if [ "$IMAGE" = 1 ]; then
        systemctl enable -q "$CELL_DHCP_UNIT"
        log "cell DHCP: ${CELL_DHCP_UNIT} enabled for ${robot} on ${cell_if} (image: probes on the first boot)"
        return
    fi
    systemctl daemon-reload
    systemctl enable -q "$CELL_DHCP_UNIT"
    systemctl restart "$CELL_DHCP_UNIT"
    if systemctl is-active -q "$CELL_DHCP_UNIT"; then
        log "cell DHCP: serving ${robot} on ${cell_if}"
    else
        log "cell DHCP: not serving now (another DHCP server answered, or ${cell_if} has no cable);" \
            "it re-checks whenever ${cell_if} comes up — journalctl -u ${CELL_DHCP_UNIT}"
    fi
}

remove_cell_dhcp() {
    systemctl disable --now "$CELL_DHCP_UNIT" 2>/dev/null || true
    rm -f "/etc/systemd/system/${CELL_DHCP_UNIT}" "$CELL_DHCP_CONF" "$NM_HOOK"
    systemctl daemon-reload
}

# ---- systemd ---------------------------------------------------------------------------
install_units() {
    install -m 0644 "${HERE}/${UNIT}" "/etc/systemd/system/${UNIT}"
    install -m 0755 "${HERE}/perceptronics-doctor" /usr/local/bin/perceptronics-doctor
    if [ "$IMAGE" = 1 ]; then
        systemctl enable -q "$UNIT"
        log "systemd: ${UNIT} enabled (image: starts on the first boot)"
        return
    fi
    systemctl daemon-reload
    systemctl enable -q "$UNIT"
    systemctl restart "$UNIT"
    log "systemd: ${UNIT} enabled and (re)started"
}

# ---- the setup portal / update queue's root side ------------------------------------------
install_admin() {
    install -m 0755 "${HERE}/perceptronics-admin" "$ADMIN_BIN"
    install -m 0644 "${HERE}/${ADMIN_PATH_UNIT}" "/etc/systemd/system/${ADMIN_PATH_UNIT}"
    install -m 0644 "${HERE}/${ADMIN_UNIT}" "/etc/systemd/system/${ADMIN_UNIT}"
    # the queue is the cockpit's to write; the status and the work area are root's to write
    install -d -o "$SVC_USER" -g "$SVC_USER" -m 0750 "${STATE_DIR}/admin" "$ADMIN_QUEUE"
    install -d -o root -g "$SVC_USER" -m 0750 "$ADMIN_STATE"
    install -d -o root -g root -m 0700 "${ADMIN_STATE}/work"
    if [ "$IMAGE" = 1 ]; then
        systemctl enable -q "$ADMIN_PATH_UNIT"
        return
    fi
    systemctl daemon-reload
    # never restart perceptronics-admin.service here: an update runs this installer from it
    systemctl enable -q --now "$ADMIN_PATH_UNIT"
    log "setup portal: ${ADMIN_PATH_UNIT} watching ${ADMIN_QUEUE}"
}

copy_deploy_files() {
    mkdir -p "$DEPLOY_COPY"
    local f
    for f in install.sh perceptronics-cockpit.service nftables.conf cell.env.template perceptronics-doctor README.md \
        cell-dhcp.conf perceptronics-cell-dhcp.service 50-perceptronics-cell \
        perceptronics-admin perceptronics-admin.path perceptronics-admin.service; do
        [ "${HERE}/${f}" -ef "${DEPLOY_COPY}/${f}" ] && continue
        install -m 0644 "${HERE}/${f}" "${DEPLOY_COPY}/${f}"
    done
    chmod 0755 "${DEPLOY_COPY}/install.sh" "${DEPLOY_COPY}/perceptronics-doctor" "${DEPLOY_COPY}/50-perceptronics-cell" \
        "${DEPLOY_COPY}/perceptronics-admin"
}

# ---- rollback / uninstall -------------------------------------------------------------
rollback() {
    [ -L "$PREVIOUS" ] || die "no previous release to roll back to"
    local cur prev
    cur="$(readlink -f "$CURRENT")"
    prev="$(readlink -f "$PREVIOUS")"
    [ -x "${prev}/bin/perceptronics" ] || die "previous release ${prev} is incomplete"
    ln -sfn "$prev" "$CURRENT"
    ln -sfn "$cur" "$PREVIOUS"
    systemctl restart "$UNIT"
    log "rolled back: current -> $(basename "$prev") (previous -> $(basename "$cur"))"
}

uninstall() {
    local purge="$1"
    systemctl disable --now "$UNIT" "$ADMIN_PATH_UNIT" 2>/dev/null || true
    rm -f "/etc/systemd/system/${UNIT}" /usr/local/bin/perceptronics-doctor "$ADMIN_BIN" \
        "/etc/systemd/system/${ADMIN_PATH_UNIT}" "/etc/systemd/system/${ADMIN_UNIT}"
    remove_cell_dhcp
    systemctl daemon-reload
    if [ -f "$NFT_CONF" ] && grep -qF "$NFT_MARKER" "$NFT_CONF"; then
        if [ -f "$NFT_BACKUP" ]; then
            mv "$NFT_BACKUP" "$NFT_CONF"
        else
            rm -f "$NFT_CONF"
        fi
        nft delete table inet perceptronics 2>/dev/null || true
        systemctl restart nftables.service 2>/dev/null || true
    fi
    rm -rf "$APP_ROOT"
    log "removed ${UNIT}, the cell DHCP server, the firewall table and ${APP_ROOT}"
    if [ "$purge" = 1 ]; then
        rm -rf "$ETC_DIR" "$STATE_DIR" "$ADMIN_STATE" "$LIBREALSENSE_PREFIX" "$LIBREALSENSE_LINK" "$LDCONF" "$UDEV_RULES"
        ldconfig
        udevadm control --reload-rules
        userdel "$SVC_USER" 2>/dev/null || true
        nmcli connection delete "$NM_CELL_CON" >/dev/null 2>&1 || true
        log "purged ${ETC_DIR}, ${STATE_DIR} (calibrations, snapshots), librealsense, udev rules, user, the ${NM_CELL_CON} profile"
    else
        log "kept ${ETC_DIR}, ${STATE_DIR} (calibrations), librealsense and the ${SVC_USER} user (--purge removes them)"
    fi
}

# ---- main ------------------------------------------------------------------------------
main() {
    local wheel="" cell="ur3" robot_host="" allow_from="" reconfigure=0 action=install purge=0
    local cell_if="eth0" cell_address="192.168.3.20/24" gateway="" dns="" cell_dhcp="auto"
    local saved_cell_if saved_cell_address saved_gateway saved_dns saved_cell_dhcp saved_allow_from
    # what an earlier run (or the setup portal) chose is the default; a flag below overrides it
    if [[ " $* " != *" --image "* ]]; then
        load_saved_network
        cell_if="${saved_cell_if:-$cell_if}"
        cell_address="${saved_cell_address:-$cell_address}"
        gateway="$saved_gateway" dns="$saved_dns" cell_dhcp="${saved_cell_dhcp:-$cell_dhcp}"
        allow_from="$saved_allow_from"
    fi
    while [ $# -gt 0 ]; do
        case "$1" in
            --wheel) wheel="${2:?--wheel needs a path}"; shift 2 ;;
            --cell) cell="${2:?--cell needs a name}"; shift 2 ;;
            --robot-host) robot_host="${2:?--robot-host needs an address}"; shift 2 ;;
            --allow-from) allow_from="${2:?--allow-from needs a CIDR}"; shift 2 ;;
            --cell-if) cell_if="${2:?--cell-if needs an interface or none}"; shift 2 ;;
            --cell-address) cell_address="${2:?--cell-address needs a CIDR}"; shift 2 ;;
            --gateway) gateway="${2:?--gateway needs an address or none}"; shift 2 ;;
            --dns) dns="${2:?--dns needs addresses or none}"; shift 2 ;;
            --cell-dhcp) cell_dhcp="${2:?--cell-dhcp needs auto or off}"; shift 2 ;;
            --network) action=network; shift ;;
            --reconfigure) reconfigure=1; shift ;;
            --image) IMAGE=1; shift ;;
            --rollback) action=rollback; shift ;;
            --uninstall) action=uninstall; shift ;;
            --purge) purge=1; shift ;;
            -h | --help) usage; exit 0 ;;
            *) die "unknown argument: $1 (see --help)" ;;
        esac
    done
    if [ -n "$robot_host" ] && ! [[ "$robot_host" =~ ^[A-Za-z0-9.:-]+$ ]]; then
        die "--robot-host ${robot_host}: expected an IP address or host name"
    fi
    [[ "$cell_if" =~ ^[A-Za-z0-9_.-]{1,15}$ ]] || die "--cell-if ${cell_if}: expected an interface name or none"
    [[ "$cell_address" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}/([89]|[12][0-9]|30)$ ]] \
        || die "--cell-address ${cell_address}: expected an IPv4 CIDR such as 192.168.3.20/24"
    [ "$gateway" = none ] && gateway=""
    [ "$dns" = none ] && dns=""
    [ -z "$gateway" ] || [[ "$gateway" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}$ ]] || die "--gateway ${gateway}: expected an IPv4 address"
    [ -z "$dns" ] || [[ "$dns" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}(,[0-9]{1,3}(\.[0-9]{1,3}){3}){0,2}$ ]] \
        || die "--dns ${dns}: expected up to three comma-separated IPv4 addresses"
    case "$cell_dhcp" in auto | off) ;; *) die "--cell-dhcp ${cell_dhcp}: expected auto or off" ;; esac
    [ "$(id -u)" -eq 0 ] || die "run as root (sudo $0 ...)"
    case "$action" in
        network)
            apply_network "$cell_if" "$cell_address" "$robot_host" "$gateway" "$dns" "$cell_dhcp" "$allow_from"
            return
            ;;
        rollback) rollback; return ;;
        uninstall) uninstall "$purge"; return ;;
    esac
    [ -n "$wheel" ] || die "--wheel is required (scripts/deploy-pi.sh builds and passes it)"
    [ "$(uname -s)" = Linux ] || die "this installer is for the Linux pick PC"
    command -v systemctl >/dev/null || die "systemd is required"

    apt_install
    check_python
    install_librealsense
    ensure_user
    install_app "$wheel"
    if [ ! -f "$CELL_ENV" ] || [ "$reconfigure" = 1 ]; then
        write_cell_env "$cell" "$robot_host"
    else
        log "kept ${CELL_ENV} (--reconfigure rewrites it from the cell profile)"
    fi
    handeye_out_of_env
    local net
    net="$(cell_net "$allow_from")"
    install_firewall "$net" "$cell_if"
    copy_deploy_files
    install_units
    install_admin
    if [ "$cell_if" = none ]; then
        remove_cell_dhcp
        log "cell port: --cell-if none — no cell address, no DHCP server"
    else
        configure_cell_address "$cell_if" "$cell_address"
        if [ "$cell_dhcp" = off ]; then
            remove_cell_dhcp
            log "cell DHCP: off (network.env)"
        else
            install_cell_dhcp "$cell_if" "$cell_address"
        fi
    fi
    save_network_env "$cell_if" "$cell_address" "$gateway" "$dns" "$cell_dhcp" "$allow_from"
    if [ "$IMAGE" = 1 ]; then
        log "done (image root)"
    else
        log "done. Check it: sudo perceptronics-doctor"
    fi
}

main "$@"
