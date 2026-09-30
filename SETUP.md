# SETUP.md — the hardware and network this repo runs against

What exists, where it is on the network, and what is connected today. `CLAUDE.md` has
the protocols and gotchas; the cell files (`perceptronics/cells/*.env`) are the machine-
readable half of this page and win where they differ. Dated lines are the field log
(`docs/realsense-cell.html` has the longer one). **Update this page when the cell
changes** — a robot moved, an address changed, something unplugged.

**Status 2026-09-29:** the UR3e is **not connected** (Nick). Nothing else in the cell has
changed since 2026-09-28. Everything below that says "verified" was verified before that.

## 1. Cells

| Cell (`--cell`) | Robot | Platform | Address | Camera bracket | Tool | Status |
| --- | --- | --- | --- | --- | --- | --- |
| `ur3` | **UR3e** (URSoftware 5.25.1 on 2026-09-27) | PolyScope 5, e-Series | `192.168.3.3` — Dashboard :29999, Primary :30001, RTDE :30004 | `eseries` print, D435 on the flange | Robotiq **Hand-E** (e-Series kit) | the demo cell; **unplugged 2026-09-29** |
| `ur20` | UR20 | PolyScope X | not filled in (`UR_HOST=` empty) — Robot-API :80, Primary :30001, RTDE :30004 | `ur20` print, clocked 45° | none yet (`PERCEPTRONICS_TIP_M` unset) | test cell, never driven from this repo |
| `sim` | PolyScope X simulator, `ROBOT_TYPE=UR3` | PolyScope X 10.13.0 (arm64 image) | `localhost` — UI :8000, Primary :31001, RTDE :31004 | none (synthetic scene, `PERCEPTRONICS_FAKE=1`) | — | `HOST_ARCH=arm64 make simx-up` |

### The UR3e cell, as built

- The arm stands on a ~12 in pedestal; **the parts sit ~0.27 m below the base** (the
  09-25 hand-eye solved a block top at base z = −0.270). The table is flat and parallel
  to base XY. Reach at that depth is the controller's IK's call, not the datasheet 0.5 m.
- **Hand-E on the ISO-50 flange** through the bracket's 6 mm adapter: flange → fingertip
  **0.163 m** along flange +Z (157 + 6). The fingers travel along flange **Y**. The
  controller's active TCP is a 223 mm training offset the Hand-E does not match — never
  used; every cockpit move sets the TCP to the fingertips itself.
- **D435 on `hardware/d435-tool-bracket` (`eseries` print, PPA-CF)**, re-clocked 180° about
  the flange axis on 2026-09-25 (camera opposite the tool connector). Hand-eye
  (`PERCEPTRONICS_T_FLANGE_CAMERA` in `perceptronics/cells/ur3.env`): re-solved 2026-09-27
  22:40 with `perceptronics calibrate --apply`, RMS 2.5 mm; the fitted floor tilts 1.4–2.2°
  with it. Re-solve before trusting a pick after anything on the wrist moves.
- **Picture pose** (`PERCEPTRONICS_HOME_POSE`): 0.37 m up, looking down in front of the
  stand, set by Nick 2026-09-27 22:25.
- **Robotiq URCap daemon is loopback-only** on the controller (63352 closed from the
  network): the gripper is driven from URScript (`urctl gripper`, `POST /api/robot/gripper`).
  It deactivates in Local mode.
- **Local vs Remote:** Primary URScript *motion* runs only in Remote (top-right pendant
  indicator; no network way to flip it). State, `textmsg`, Dashboard work in Local.
- **SSH :22 on the controller times out from the Mac** (2026-09-27); file placement by
  `scp` has worked on the UR10 at `192.168.1.50` but not been exercised on this UR3e.
- **URCap on it:** RealSense Pilot 0.2.0 installed and rendered on the pendant 2026-09-27;
  0.3.0 is on the "URE MODELS" USB stick; **0.5.0 has never been on a pendant**, and the
  URCap is now **Perceptronic 0.6.0** (`urcap/dist/perceptronic-ps5-0.6.0.urcap`, bundle
  `com.nickarmenta.perceptronic` — a different URCap to PolyScope: remove RealSense Pilot
  on the pendant, its node data and Pick nodes don't carry over). Auto-install from the stick needs
  **Settings → Security → General → Run magic files** on (`scripts/urcap5-usb.sh`,
  `scripts/urmagic_perceptronic.sh`) — also never run on this robot yet.

## 2. Computers

| Machine | Role | Address / access | Notes |
| --- | --- | --- | --- |
| **Mac Studio** (M1 Max, arm64, macOS 26 / Darwin 25) | dev host; ran the cockpit for every hardware session so far | cell LAN `192.168.3.10` (bind the cockpit here, not 0.0.0.0); a second interface (`en1`) on the office LAN; Screen Sharing (VNC) on :5900 | Nick works on it **over SSH**: an SSH shell has no sudo credential, no camera access (TCC), and **cannot start Docker Desktop** (`open -a Docker` fails from a login session; start it from the console or Screen Sharing). The camera needs `sudo` (libusb must detach Apple's UVC driver) and a local Terminal; `--rs-lean` is what streams on this Mac (2026-09-25). The app firewall silently dropped pendant → cockpit :7621 until turned off (2026-09-27). `captures/` is root-owned from sudo runs. Two Logitech webcams (`HD Pro Webcam C920`, `Logi Webcam C920e`) are the extra views in `ur3.env`. |
| **Pick PC** (`deploy/pi/`) | the shipped camera computer: Pi-class arm64 Debian, cockpit as `perceptronics-cockpit.service`, :7621 HTTP + :7622 pick server | example address `192.168.3.10`; `scripts/deploy-pi.sh <user@pc> --cell ur3 --robot-host 192.168.3.3` | **Never run on a board yet** (2026-09-28). Kit hardware in `hardware/BOM.md`: KUNBUS RevPi Connect 5 (primary) or CompuLab IOT-GATE-RPI5, Mean Well HDR-60-24, Newnex screw-lock USB cable, L-com Ethernet. |
| **Windows work laptop** | demo host candidate (2026-09-27 trial); the D435 streams under WSL2 (verified 2026-09-23) | — | Native Windows path (`scripts/setup-windows.ps1`, `scripts/cockpit.ps1`) never run. |
| **Jetson** | the on-controller GPU for the PolyScope X future: Orin first, then **AGX Thor** (`feature/perception-thor`, worktree `../UR-utils-thor`, NGC PyTorch 25.08 / CUDA 13) | — | Final deployment (decided 2026-09-25): a Jetson next to a UR running PolyScope X, room for a second on other arms. Not built yet. |

## 3. Simulators on the Mac

| Simulator | Works here? | How |
| --- | --- | --- |
| PolyScope X 10.13.0 (`ursim-px`, arm64) | **yes** | `HOST_ARCH=arm64 make simx-up` → UI :8000, Primary :31001, RTDE :31004. Enable Primary/RTDE once under Settings → Security → Services; Remote mode for anything mutating. A second one as UR3 has run as a plain container on :8001 / :32001 / :32004. Each PX sim's inner Docker is ~12 GB (Docker Desktop disk raised to 160 GB, 2026-09-27); `docker rm -v` or it leaks. |
| URCap in that sim + a fake cockpit | yes | `make urcap-install` (urservice endpoint, no Remote needed) + `make urcap-cockpit` (:7621 with `--cors` for :8000). `urcap/e2e.py` does it headless (~2 min). |
| e-Series URSim 5.x (PolyScope 5) | **no** — the image is amd64-only and neither Rosetta nor QEMU user-mode keeps URControl + Xvfb alive | `scripts/ursim-e-vm.sh up` (full x86_64 QEMU VM, ~8 min to Dashboard) proves a URCap *loads*; PolyScope's JVM crashes in JIT there, so no clicking through. Every 5.x minor from 5.4 runs in CI on amd64 (`urcap5-matrix.yml`). Off-pendant screens: `uv run python urcap/preview5.py` (JDK). |

## 4. Network summary

Cell subnet `192.168.3.0/24`: robot `.3`, cockpit/pick PC `.10`. The UR10 seen earlier in
this repo's history was `192.168.1.50` (another network). Controller ports are in
`CLAUDE.md` § *Network surface of a UR controller*; the cockpit's are :7621 (HTTP),
:7622 (pick server socket the PS5 program node opens), :7631 (the retired
`pick-server` sidecar — kill it before relaunching a cockpit, both bind :7622).

## 5. Removable media

- **"URE MODELS"** — FAT32 USB stick for the pendant. `scripts/urcap5-usb.sh` puts the
  current `.urcap`, the `.urcapx` (PolyScope X, for System Manager) and the magic file on
  it without macOS `._` files, verifies the checksum and ejects.
