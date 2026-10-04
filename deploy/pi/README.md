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
| Network | The Ethernet port goes to the robot (directly or through the cell switch). `install.sh` gives it **192.168.3.20/24** and serves the robot **192.168.3.3** over DHCP, so a robot left on DHCP needs no setup and the URCap needs nothing typed (§3). Office access, if any, over Wi-Fi. |
| Robot | UR e-Series on PolyScope 5 with the Perceptronic URCap (`urcap/dist/perceptronic-ps5-*.urcap`, see `urcap/perceptronic-ps5/README.md`). |

**Power:** the D435 is powered from the USB port. Raspberry Pi's documentation gives a
Pi 4 **1.2 A total** for USB peripherals on the recommended 3 A supply (the kit's 5 V
HDR-30-5, trimmed to 5.1 V). A Pi 5 limits USB to 600 mA unless it runs on the 5 V / 5 A
supply (or `usb_max_current_enable=1` is set in `config.txt`). Neither has **been tested
with a D435 in this repo**. If the camera drops out under load, check the supply first.

**Faster: flash the prebuilt image.** `image/README.md` builds one image with all of this
already installed, flashes it and seeds each board (host name, key, address, robot) without
a compile or internet on the PC. The steps below are the from-scratch path, and the update
path for a running board.

## 1. Flash and first boot

1. Flash **Debian arm64** (for a Raspberry Pi or a CM4/CM5 box: Debian's Raspberry Pi
   image), create a user in the `sudo` group with your laptop's SSH key in
   `~/.ssh/authorized_keys`, and set the host name, e.g. `pickpc`. (Raspberry Pi Imager's
   **Raspberry Pi OS Lite (64-bit)** is Debian too: set the user, public-key SSH and host
   name in its OS customisation settings.)
2. Plug in Ethernet on the robot's network and the D435 (USB 3), then boot.
3. The cell address is the installer's job (§2, *cell port*): nothing to do here. Reach the
   PC for the deploy over Wi-Fi or a second network — `install.sh` never re-addresses a port
   that is already on another network, so deploying over the Ethernet port on the office LAN
   leaves it there; re-run on the cell to give it 192.168.3.20. By hand, on Raspberry Pi OS
   (NetworkManager): `sudo nmcli con mod <name> ipv4.method manual ipv4.addresses
   192.168.3.20/24 ipv4.never-default yes` (on the 2026-10-02 Pi 5 image the profile was
   `netplan-eth0`; it persisted through the netplan backend and a reboot).
4. The first install needs internet access on the PC. apt and the librealsense build
   fetch from Debian mirrors and GitHub (plus sqlite.org), see *Open items*.

## 2. Deploy (one command, from your laptop)

From a checkout of this repo on the laptop (it needs `python3` with `pip`, and `ssh`):

    scripts/deploy-pi.sh pi@192.168.3.20 --cell ur3 --robot-host 192.168.3.3

This builds the wheel (`python3 -m pip wheel`), copies it and `deploy/pi/`
to the PC, runs `install.sh` there under `sudo`, and prints `perceptronics doctor` from the
PC. The first run compiles librealsense, which took 10.5 minutes on a Pi 5 4 GB (2026-10-02, `-j2`) and longer on a Pi 4 (not yet timed). Later runs
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
| firewall | `/etc/nftables.conf` (the original is kept as `.pre-perceptronics`). Inbound traffic is dropped except loopback, replies, ICMP, SSH, :7621/:7622 from the cell subnet (`--allow-from CIDR`, default `UR_HOST`'s /24) and DHCP requests on the cell port. |
| cell port | `--cell-if` (default `eth0`; `none` skips this row) gets `--cell-address` (default `192.168.3.20/24`, no gateway) as the NetworkManager profile `perceptronics-cell` — unless the port is already on another network, which is left alone. `perceptronics-cell-dhcp.service` (dnsmasq, already on Raspberry Pi OS Lite) hands **one** lease, the cell's `UR_HOST` (192.168.3.3), with no route and no DNS; it starts only when `python -m perceptronics.cellnet probe <port>` heard **no other DHCP server** there, and a NetworkManager hook re-probes every time the port comes up — so plugged into a plant network by mistake it stays quiet. A `UR_HOST` off the cell subnet: no DHCP, the robot needs a static address. |
| service | `perceptronics-cockpit.service` enabled and restarted, plus `/usr/local/bin/perceptronics-doctor` |

## 3. Point the pendant at it

Out of the box, nothing: an empty **Cockpit** field (**Installation** tab → **URCaps** →
**Perceptronic**) means `192.168.3.20`, the address the installer gives the PC's cell port,
and the field shows it. The robot has to be on that network: **Settings → System → Network →
DHCP → Apply** (UR's recommended setting — the PC hands it 192.168.3.3), or a static
`192.168.3.x`, mask `255.255.255.0`. A PC deployed with another `--cell-address`: type that
address in the field → **Save**. The Pick node uses the same host. It learns the pick port (:7622) from the cockpit. No `--cors` is needed,
because the node is Java on the controller, not a web page.

## Day-to-day

| | |
| --- | --- |
| health | `sudo perceptronics-doctor` (add `--json`, `--no-robot`; `--stream` opens the camera, so stop the service first) |
| logs | `journalctl -u perceptronics-cockpit -f` |
| stop / start | `sudo systemctl stop perceptronics-cockpit` / `sudo systemctl start perceptronics-cockpit` |
| the cockpit UI from a laptop | `ssh -L 7621:127.0.0.1:7621 pi@192.168.3.20`, then open http://127.0.0.1:7621 |
| config | edit `/etc/perceptronics/cell.env`, then `sudo systemctl restart perceptronics-cockpit` |
| calibration | `perceptronics calibrate --apply` saves to `/var/lib/perceptronics/captures/calibration/handeye.json` and takes effect at once. `install.sh` keeps `PERCEPTRONICS_T_FLANGE_CAMERA` out of `cell.env` (it seeds that file from the profile instead), because an environment value would win over every calibration at the next restart. Command: `PLUG-AND-PLAY.md` §6. |
| audit | every robot action: `/var/lib/perceptronics/audit.jsonl` |

**Update:** run the same `scripts/deploy-pi.sh pi@<ip>` from a newer checkout. It
installs a new release beside the old one, moves `current`, and restarts the service.
`cell.env` is kept. A PC with no SSH login (a card flashed from the image with no seed)
updates through the setup portal instead (below).

**Rollback:** `scripts/deploy-pi.sh pi@<ip> --rollback`, or on the PC
`sudo /opt/perceptronics/deploy/install.sh --rollback`. This swaps `current` and `previous`
and restarts the service.

**Uninstall:** `sudo /opt/perceptronics/deploy/install.sh --uninstall` removes the service,
the firewall table and `/opt/perceptronics`. It keeps `/etc/perceptronics`, the calibrations
in `/var/lib/perceptronics`, librealsense and the user. Add `--purge` to remove those too.

## Setup portal: network and updates without SSH

Every PC installed or flashed from this directory serves **http://192.168.3.20:7621/setup**
(log in `admin` / `admin` for now; `PERCEPTRONICS_ADMIN_USER` / `PERCEPTRONICS_ADMIN_PASSWORD`
in `cell.env` change it). Connect a laptop to the PC's Ethernet port and give the laptop a
**static** `192.168.3.10`, mask `255.255.255.0`. Don't use DHCP: the PC's one-lease DHCP
server would hand the laptop the robot's address.

**Network.** Type the PC's new address, mask, optional gateway and DNS, and the robot's
address, then press Apply. The PC moves at once and answers at the new address. **It keeps
`192.168.3.20` as a second, backup address** unless the new network holds that address, so a
wrong entry never locks you out: plug back in at `192.168.3.10` and fix it. Tick "Give the
robot its address" only for a network with no DHCP server of its own (a cable straight to the
robot). It never serves while another DHCP server answers anyway. The firewall follows: the
cockpit and pick server accept the new network, the backup subnet, and a robot routed in from
another subnet.

**Update.** Choose the `perceptronics-update-<version>-<rev>.tar` you were sent and press
Install. The PC checks every file against the bundle's manifest (sha256; a truncated or
damaged upload is refused before anything changes), installs it as a new release with the
same `install.sh`, waits up to 2 min for the cockpit to answer, and **rolls back by itself**
if it doesn't. `cell.env`, calibrations and the network settings are kept. The page shows
the install log. Make a bundle and send it from a checkout:

```bash
scripts/pi-update.sh bundle                       # target/pi-update/perceptronics-update-*.tar
scripts/pi-update.sh push target/pi-update/perceptronics-update-*.tar 192.168.3.20
```

A cell PC has no internet. If a bundle pins a newer librealsense than the PC has, add the
prebuilt library: `scripts/pi-update.sh bundle --librealsense
target/pi-image/cache/librealsense-<ver>.tar`. Without it, `install.sh` fails before it
switches anything, and the page says so.

**How it works.** The cockpit (unprivileged, sandboxed) only checks the request and drops it
in `/var/lib/perceptronics/admin/queue`. `perceptronics-admin.path` then starts
`perceptronics-admin.service`, which runs `/usr/local/sbin/perceptronics-admin` (stdlib
Python, as root; it doesn't depend on the release it is replacing). The helper re-checks
everything the cockpit wrote (no symlinks, size caps, every field), runs `install.sh
--network …` or the bundle's `install.sh --wheel …`, and writes
`/var/lib/perceptronics-admin/status.json`, which the page shows. The network settings in use
are saved to `/etc/perceptronics/network.env` and are the defaults of every later install, so
an update never undoes the portal. Logs: `journalctl -u perceptronics-admin`.

**Trust.** Bundles carry checksums, not a signature (decided 2026-10-03). Anyone who can
reach `:7621` from the allowed subnets and knows the login can install software as root. The
firewall limits that to the cell. Change the default login on a PC that leaves the bench.

## Ports

| Port | Direction | What | Who may connect |
| --- | --- | --- | --- |
| 22/tcp | in | SSH | anyone (key auth; tighten in `nftables.conf` if the PC is on a wider network) |
| 7621/tcp | in | cockpit HTTP API (`perceptronics gui --port`), incl. `/api/color.png`, and the setup portal `/setup` (login) | cell subnet only (+ the backup subnet after a network change) |
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

## Verified on a board (2026-10-02)

Raspberry Pi 5 Model B 4 GB, Raspberry Pi OS Lite (64-bit) **trixie**, kernel 6.18.50,
Python 3.13.5, systemd 257, official 27 W supply (`usb_max_current_enable=1`,
`get_throttled=0x0` throughout), D435 fw 5.12.7.100 on a blue port (USB 3.2).

- **librealsense v2.58.4 builds** on arm64 Debian outside Docker: 10.5 min (`-j2`, the
  RAM / 1536 MiB rule on 4 GB), 55 °C with the active cooler.
- **`nft -c` accepts the rendered firewall**; the table loads and survives a reboot.
- **The unit is valid** (`systemd-analyze verify` silent); `systemd-analyze security`:
  3.3 OK.
- **The hardened, non-root unit opens the D435** through the udev rules: 848×480 @ 30,
  aligned, High Density preset applied. No `DeviceAllow=` change needed.
- **Stream cost:** `/api/color.png` 27 ms median on the Pi (92 ms from a laptop over
  Wi-Fi); the cockpit uses ~70 % of one core streaming.
- **Reboot:** the static cell address, the firewall and the service (camera open,
  frames advancing) all come back on their own.
- **The robot side**, with the Pi on the office LAN: against the PolyScope X sim
  (Robot-API state, RTDE pose at 30 Hz, the sim's controller container opening :7621 and
  :7622 on the Pi) and against the e-Series sim's URControl in the x86 VM (RTDE pose and
  Primary TCP readback with the `ur3` cell profile, through an SSH tunnel; that VM's
  PolyScope crashed twice, so the Dashboard path was not exercised from the Pi).

## Open items

- **The real UR3e from the Pi**: Dashboard, Remote-mode motion and the pendant's URCap
  reaching `http://192.168.3.20:7621`. Needs the robot powered and the Pi on the cell switch.
- **A sustained run**: the longest unbroken stream so far is ~7 min.
- **Build needs internet.** librealsense's CMake fetches nlohmann/json, fastcdr, yaml-cpp
  (GitHub) and sqlite (sqlite.org) during the build (`BUILD_ROSBAG2` defaults ON; left as
  the Dockerfile's verified build does). For an air-gapped PC, build once on an identical
  networked board and copy `/opt/librealsense-2.58.4` across. `install.sh` skips the build
  when its stamp file matches.
- **Restart back-off** (`RestartSteps=`, `RestartMaxDelaySec=`) needs systemd ≥ 254
  (trixie). bookworm's systemd 252 ignores them and restarts every 5 s.
- `SystemCallFilter=` is not set, pending a run under `systemd-analyze security`.
- `audit.jsonl` grows without rotation.
