#!/usr/bin/env bash
# The pick PC's installer: a Raspberry-Pi-class arm64 box (Debian bookworm/trixie, no
# desktop, no GPU) that runs the RealSense cockpit headless next to a UR e-Series.
# Run as root ON THE PC; scripts/deploy-pi.sh copies it over and runs it for you.
#
#   sudo ./install.sh --wheel ur_docker-0.1.0-py3-none-any.whl [--cell ur3] \
#                     [--robot-host 192.168.3.3] [--allow-from 192.168.3.0/24] [--reconfigure]
#                     [--cell-if eth0|none] [--cell-address 192.168.3.20/24]
#   sudo /opt/perceptronics/deploy/install.sh --rollback        # back to the previous release
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
set -euo pipefail

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
        udevadm control --reload-rules
        udevadm trigger --subsystem-match=usb
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
    "# After `perceptronics calibrate --apply` on this PC, delete the PERCEPTRONICS_T_FLANGE_CAMERA",
    "# line and restart: an environment value wins over the saved hand-eye file.",
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
    local net="$1" cell_if="$2"
    [[ "$net" =~ ^[0-9]{1,3}(\.[0-9]{1,3}){3}/[0-9]{1,2}$ ]] || die "--allow-from ${net}: expected an IPv4 CIDR"
    local rendered
    rendered="$(mktemp)"
    sed -e "s#@CELL_NET@#${net}#g" -e "s#@CELL_IF@#${cell_if}#g" "${HERE}/nftables.conf" >"$rendered"
    nft -c -f "$rendered" || die "nftables.conf failed nft's syntax check"
    if [ -f "$NFT_CONF" ] && ! grep -qF "$NFT_MARKER" "$NFT_CONF" && [ ! -f "$NFT_BACKUP" ]; then
        cp -p "$NFT_CONF" "$NFT_BACKUP"
        log "firewall: kept the original ${NFT_CONF} as ${NFT_BACKUP}"
    fi
    install -m 0644 "$rendered" "$NFT_CONF"
    rm -f "$rendered"
    systemctl enable -q nftables.service
    systemctl restart nftables.service
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
    # bench's hand-made one): keep it rather than add a second.
    while IFS=: read -r con; do
        if [ -z "$con" ] || [ "$con" = "$NM_CELL_CON" ]; then continue; fi
        if [ "$(nmcli -g connection.interface-name connection show "$con" 2>/dev/null)" = "$cell_if" ] \
            && nmcli -g ipv4.addresses connection show "$con" 2>/dev/null | grep -qF "$cidr"; then
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
    systemctl daemon-reload
    systemctl enable -q "$UNIT"
    systemctl restart "$UNIT"
    log "systemd: ${UNIT} enabled and (re)started"
}

copy_deploy_files() {
    mkdir -p "$DEPLOY_COPY"
    local f
    for f in install.sh perceptronics-cockpit.service nftables.conf cell.env.template perceptronics-doctor README.md \
        cell-dhcp.conf perceptronics-cell-dhcp.service 50-perceptronics-cell; do
        [ "${HERE}/${f}" -ef "${DEPLOY_COPY}/${f}" ] && continue
        install -m 0644 "${HERE}/${f}" "${DEPLOY_COPY}/${f}"
    done
    chmod 0755 "${DEPLOY_COPY}/install.sh" "${DEPLOY_COPY}/perceptronics-doctor" "${DEPLOY_COPY}/50-perceptronics-cell"
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
    systemctl disable --now "$UNIT" 2>/dev/null || true
    rm -f "/etc/systemd/system/${UNIT}" /usr/local/bin/perceptronics-doctor
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
        rm -rf "$ETC_DIR" "$STATE_DIR" "$LIBREALSENSE_PREFIX" "$LIBREALSENSE_LINK" "$LDCONF" "$UDEV_RULES"
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
    local cell_if="eth0" cell_address="192.168.3.20/24"
    while [ $# -gt 0 ]; do
        case "$1" in
            --wheel) wheel="${2:?--wheel needs a path}"; shift 2 ;;
            --cell) cell="${2:?--cell needs a name}"; shift 2 ;;
            --robot-host) robot_host="${2:?--robot-host needs an address}"; shift 2 ;;
            --allow-from) allow_from="${2:?--allow-from needs a CIDR}"; shift 2 ;;
            --cell-if) cell_if="${2:?--cell-if needs an interface or none}"; shift 2 ;;
            --cell-address) cell_address="${2:?--cell-address needs a CIDR}"; shift 2 ;;
            --reconfigure) reconfigure=1; shift ;;
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
    [ "$(id -u)" -eq 0 ] || die "run as root (sudo $0 ...)"
    case "$action" in
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
    local net
    net="$(cell_net "$allow_from")"
    install_firewall "$net" "$cell_if"
    copy_deploy_files
    install_units
    if [ "$cell_if" = none ]; then
        remove_cell_dhcp
        log "cell port: --cell-if none — no cell address, no DHCP server"
    else
        configure_cell_address "$cell_if" "$cell_address"
        install_cell_dhcp "$cell_if" "$cell_address"
    fi
    log "done. Check it: sudo perceptronics-doctor"
}

main "$@"
