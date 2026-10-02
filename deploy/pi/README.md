# The pick PC: a Raspberry-Pi-class box beside a UR e-Series

A small arm64 computer that owns the RealSense D435 and runs the cockpit headless as
a systemd service. The robot's PolyScope 5 URCap talks to it over Ethernet:

- **Perceptronic** (Installation node) calls the cockpit's HTTP API on **:7621**,
  including `GET /api/color.png` for the feed on the pendant;
- **3D Pick** (program node; "Perceptronic Pick" before URCap 0.7.0) runs URScript that opens a socket to the pick server
  on **:7622**.

Nothing here needs a desktop, a GPU, Docker or a network connection at runtime. The
runtime is stdlib-only Python plus librealsense, which the installer builds from source.

## Supported hardware and OS

| | |
| --- | --- |
| Board | **Raspberry Pi 4 Model B, 4 GB** (the kit board, `hardware/BOM.md` K1; Nick, 2026-09-29: efficient compute), a Pi 5, or a CM4/CM5 industrial box — any **arm64** board with a **USB 3** port and Ethernet. Not a Pi 3 (USB 2 only). 2 GB boards work: the installer adds a temporary swapfile for the build. |
| OS | **Debian arm64** (the target — Nick, 2026-09-28), bookworm (12) or trixie (13), minimal, no desktop; Raspberry Pi OS Lite (64-bit) is Debian and works the same. A RevPi Connect 5 gets a Debian image, not RevPi OS. Needs Python ≥ 3.10 (bookworm has 3.11, trixie 3.13) and systemd. |
| Camera | One Intel RealSense **D435** (USB ID `8086:0b07`), connected **straight to a USB 3 port** (blue) with a short cable, no hub. |
| Network | Ethernet on the robot's subnet. A static address is easiest to type into the pendant. |
| Robot | UR e-Series on PolyScope 5 with the Perceptronic URCap (`urcap/dist/perceptronic-ps5-*.urcap`, see `urcap/perceptronic-ps5/README.md`). |

**Power:** the D435 is powered from the USB port. Raspberry Pi's documentation gives a
Pi 4 **1.2 A total** for USB peripherals on the recommended 3 A supply (the kit's 5 V
HDR-30-5, trimmed to 5.1 V). A Pi 5 limits USB to 600 mA unless it runs on the 5 V / 5 A
supply (or `usb_max_current_enable=1` is set in `config.txt`). Neither has **been tested
with a D435 in this repo**. If the camera drops out under load, check the supply first.

## 1. Flash and first boot

1. Flash **Debian arm64** (for a Raspberry Pi or a CM4/CM5 box: Debian's Raspberry Pi
   image), create a user in the `sudo` group with your laptop's SSH key in
   `~/.ssh/authorized_keys`, and set the host name, e.g. `pickpc`. (Raspberry Pi Imager's
   **Raspberry Pi OS Lite (64-bit)** is Debian too: set the user, public-key SSH and host
   name in its OS customisation settings.)
2. Plug in Ethernet on the robot's network and the D435 (USB 3), then boot.
3. Give it a fixed address on the cell subnet. On Raspberry Pi OS (NetworkManager):

       sudo nmcli con mod "Wired connection 1" ipv4.method manual \
            ipv4.addresses 192.168.3.10/24 ipv4.gateway 192.168.3.1
       sudo nmcli con up "Wired connection 1"

   (`nmcli con show` lists the connection names.) On Debian with ifupdown, use
   `/etc/network/interfaces`. Or reserve the address on the cell's DHCP server.
4. The first install needs internet access on the PC. apt and the librealsense build
   fetch from Debian mirrors and GitHub (plus sqlite.org), see *Open items*.

## 2. Deploy (one command, from your laptop)

From a checkout of this repo on the laptop (it needs `python3` with `pip`, and `ssh`):

    scripts/deploy-pi.sh pi@192.168.3.10 --cell ur3 --robot-host 192.168.3.3

This builds the wheel (`python3 -m pip wheel`), copies it and `deploy/pi/`
to the PC, runs `install.sh` there under `sudo`, and prints `perceptronics doctor` from the
PC. The first run compiles librealsense, which takes tens of minutes on a Pi 5 and longer on a Pi 4 (not yet timed). Later runs
reuse it. Run from a terminal, and sudo on the PC prompts for your password. Run from an
agent's shell (no terminal), the PC's sudo must be passwordless (Raspberry Pi OS's first
user is), or the script stops with sudo's error rather than hanging.

What `install.sh` does, idempotently:

| Step | Result |
| --- | --- |
| apt | `python3 python3-venv git ca-certificates cmake build-essential pkg-config libusb-1.0-0-dev libudev-dev nftables usbutils` (each one's reason is in the script) |
| librealsense | **v2.58.4** (the ctypes binding checks enum ordinals written against 2.58; the same tag as `Dockerfile.perceptronics`), commit-checked after the clone, built with `-DFORCE_RSUSB_BACKEND=ON` (libusb, no kernel patches), no examples, tools, graphical examples or Python bindings, and `CHECK_FOR_UPDATES=OFF`. Installed to `/opt/librealsense-2.58.4` (`/opt/librealsense` → it), registered with `ldconfig`. |
| udev | the SDK's own `99-realsense-libusb.rules` (MODE 0666, group plugdev), so the service opens the camera **without root** |
| user | system user `perceptronics` in `plugdev` + `video`, state in `/var/lib/perceptronics` |
| app | a venv per wheel under `/opt/perceptronics/releases/<version>-<sha>`, `pip install --no-index --no-deps` (nothing fetched), `/opt/perceptronics/current` and `previous` symlinks, the three newest releases kept |
| config | `/etc/perceptronics/cell.env` from the shipped cell (`perceptronics/cells/<cell>.env`) minus the Mac's webcam lines, plus `cell.env.template`, plus `--robot-host`. Written only when missing or with `--reconfigure` (`--cell` / `--robot-host` on `deploy-pi.sh` imply it). The old file is kept as `cell.env.<timestamp>`. |
| firewall | `/etc/nftables.conf` (the original is kept as `.pre-perceptronics`). Inbound traffic is dropped except loopback, replies, ICMP, SSH, and :7621/:7622 from the cell subnet (`--allow-from CIDR`, default `UR_HOST`'s /24). |
| service | `perceptronics-cockpit.service` enabled and restarted, plus `/usr/local/bin/perceptronics-doctor` |

## 3. Point the pendant at it

On the pendant: **Installation** tab → **URCaps** → **Perceptronic** → **Cockpit**:
type `http://192.168.3.10:7621` (the PC's address) → **Save**. The Pick node uses the
same host. It learns the pick port (:7622) from the cockpit. No `--cors` is needed,
because the node is Java on the controller, not a web page.

## Day-to-day

| | |
| --- | --- |
| health | `sudo perceptronics-doctor` (add `--json`, `--no-robot`; `--stream` opens the camera, so stop the service first) |
| logs | `journalctl -u perceptronics-cockpit -f` |
| stop / start | `sudo systemctl stop perceptronics-cockpit` / `sudo systemctl start perceptronics-cockpit` |
| the cockpit UI from a laptop | `ssh -L 7621:127.0.0.1:7621 pi@192.168.3.10`, then open http://127.0.0.1:7621 |
| config | edit `/etc/perceptronics/cell.env`, then `sudo systemctl restart perceptronics-cockpit` |
| calibration | `perceptronics calibrate --apply` saves to `/var/lib/perceptronics/captures/calibration/handeye.json`. Then **delete the `PERCEPTRONICS_T_FLANGE_CAMERA` line** in `cell.env` and restart, because an environment value wins over the file (CLAUDE.md, the stale hand-eye gotcha). |
| audit | every robot action: `/var/lib/perceptronics/audit.jsonl` |

**Update:** run the same `scripts/deploy-pi.sh pi@<ip>` from a newer checkout. It
installs a new release beside the old one, moves `current`, and restarts the service.
`cell.env` is kept.

**Rollback:** `scripts/deploy-pi.sh pi@<ip> --rollback`, or on the PC
`sudo /opt/perceptronics/deploy/install.sh --rollback`. This swaps `current` and `previous`
and restarts the service.

**Uninstall:** `sudo /opt/perceptronics/deploy/install.sh --uninstall` removes the service,
the firewall table and `/opt/perceptronics`. It keeps `/etc/perceptronics`, the calibrations
in `/var/lib/perceptronics`, librealsense and the user. Add `--purge` to remove those too.

## Ports

| Port | Direction | What | Who may connect |
| --- | --- | --- | --- |
| 22/tcp | in | SSH | anyone (key auth; tighten in `nftables.conf` if the PC is on a wider network) |
| 7621/tcp | in | cockpit HTTP API (`perceptronics gui --port`), incl. `/api/color.png` | cell subnet only |
| 7622/tcp | in | pick server for the 3D Pick node (`--pick-port`) | cell subnet only |
| 29999, 30001, 30004/tcp | out | robot Dashboard, Primary, RTDE (`UR_*_PORT` in `cell.env`) | — |

Both inbound services are **unauthenticated** (a trusted cell network, like the robot's
own ports). The firewall is what keeps them on the cell.

A `perceptronics pick-server` sidecar also binds **:7622**. Never run one next to this
service. If you did, `pkill -f "pick-server --bind"` before restarting the cockpit, or the
cockpit warns and runs without its pick server.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| doctor: `no RealSense device enumerated` | `lsusb` must list `8086:0b07`. If it doesn't, check the cable and power. If it does, the udev rules are missing: `ls /etc/udev/rules.d/99-realsense-libusb.rules`, then re-plug. |
| doctor: camera `usb 2.x` (warn) | The link fell back to USB 2. The cockpit negotiates 640×480 @ 15 by itself. For 848×480 @ 30, use a blue port, a short cable, no hub. `lsusb -t` shows the link speed: `5000M` is USB 3, `480M` is USB 2. |
| `RS2_USB_STATUS_ACCESS` / `claim usb interface` | udev rules not applied to an already-plugged camera: re-plug it. Check that `id perceptronics` lists `plugdev`. |
| pendant: cockpit unreachable | `sudo nft list ruleset`: the pendant's address must be inside `CELL_NET`. Re-run with `--allow-from <subnet>`. |
| Pick node answers `-4` (no fresh frame) over and over | The camera dropped out. The cockpit re-opens it by itself, and `journalctl -u perceptronics-cockpit` says why. Re-plug if it doesn't recover. |
| service restarts in a loop | `journalctl -u perceptronics-cockpit -b`. A bad value in `cell.env` is the usual cause. Compare it with `perceptronics cells`. |

## Open items (not verified yet)

- **Nothing here has run on a Pi yet** (written 2026-09-28). The scripts are checked by
  `tests/test_deploy_pi.py`: syntax, shellcheck, the unit's command line under the real
  CLI, the pins, and the cell.env writer. Still unexercised: the librealsense build on
  arm64 Debian outside Docker, `nft -c` on the rendered firewall, `systemd-analyze
  verify` on the unit, and the hardened unit actually opening the D435.
- **Non-root camera access** is expected from the SDK's udev rules (MODE 0666) with the
  RSUSB backend. It is not yet observed with the D435 on this setup. If the open fails
  with an access error while root works, look first at the `DeviceAllow=` /
  `DevicePolicy=` lines in the unit.
- **Build needs internet.** librealsense's CMake fetches nlohmann/json, fastcdr, yaml-cpp
  (GitHub) and sqlite (sqlite.org) during the build (`BUILD_ROSBAG2` defaults ON; left as
  the Dockerfile's verified build does). For an air-gapped PC, build once on an identical
  networked board and copy `/opt/librealsense-2.58.4` across. `install.sh` skips the build
  when its stamp file matches.
- **Restart back-off** (`RestartSteps=`, `RestartMaxDelaySec=`) needs systemd ≥ 254
  (trixie). bookworm's systemd 252 ignores them and restarts every 5 s.
- `SystemCallFilter=` is not set, pending a run under `systemd-analyze security`.
- `audit.jsonl` grows without rotation.
