---
name: deploy-pick-pc
description: >-
  Deploy, update, roll back or check the pick PC: a Raspberry-Pi-class arm64 box on
  minimal Debian / Raspberry Pi OS Lite with a RealSense D435 that runs the RGB-D cockpit
  headless (perceptronics-cockpit.service, :7621 HTTP + :7622 pick server) for a UR e-Series
  on PolyScope 5 and its Perceptronic / 3D Pick URCap. Use when asked to
  "deploy / set up / install / update the pick PC (or the Pi, the camera computer) at
  <ip>", to roll it back, to check why the pendant can't reach the cockpit, or to point
  the URCap at it. Covers the preflight you answer from the environment (not by asking),
  scripts/deploy-pi.sh, reading the doctor, the pendant's Cockpit field, and the
  USB 2 / udev / firewall / :7622 sidecar traps.
---

# Deploying the pick PC

The human version, with what the installer does step by step, is
`deploy/pi/README.md`. This skill is the procedure. The pieces:

- `scripts/deploy-pi.sh <user@host> [--cell NAME] [--robot-host IP] [--allow-from CIDR]
  [--doctor-only] [--rollback]` runs **on the laptop** (this checkout). It builds the
  wheel with `python3 -m pip wheel`, scps it plus `deploy/pi/`, runs `install.sh` under sudo on the
  PC, then runs `sudo perceptronics-doctor` there.
- `deploy/pi/install.sh` runs **on the PC** as root, idempotently. It covers apt,
  librealsense v2.58.4 (RSUSB), udev rules, the `perceptronics` user, a venv per release,
  `/etc/perceptronics/cell.env`, nftables and the systemd unit. Its copy lives at
  `/opt/perceptronics/deploy/install.sh` for `--rollback` / `--uninstall [--purge]`.
- `perceptronics-doctor` on the PC is `perceptronics doctor` as the service user with the
  service's cell file.

## 1. Preflight: answer these yourself, don't ask

Take the PC's address from the request. Run every check below and report the answers
in one short block before deploying. Stop only on a hard failure; each row says what
to do.

| Question | How to answer it | If it fails |
| --- | --- | --- |
| Can I log in, with a key? | `ssh -o BatchMode=yes -o ConnectTimeout=10 <user@pc> true` | Ask for the user name or key. Never use `sshpass`, and never put a password on a command line. |
| Passwordless sudo? (my shell has no TTY) | `ssh <user@pc> sudo -n true` | Ask Nick to run `scripts/deploy-pi.sh` in his own terminal (sudo prompts there), or to add a sudoers rule. |
| arm64 + supported OS? | `ssh <pc> 'uname -m; . /etc/os-release; echo $ID $VERSION_CODENAME; python3 -V'` | Expect `aarch64`, `debian`/`raspbian`, bookworm/trixie, Python ≥ 3.10. Another distro: say so and stop. |
| Is the D435 there? | `ssh <pc> 'grep -l 0b07 /sys/bus/usb/devices/*/idProduct'` (works before `usbutils` is installed); after install, `lsusb -d 8086:0b07` | Nothing found: the camera isn't plugged in or isn't powered. You can still deploy; the doctor will flag it. |
| USB 3 link? | `ssh <pc> 'for d in /sys/bus/usb/devices/*; do [ "$(cat $d/idVendor 2>/dev/null)$(cat $d/idProduct 2>/dev/null)" = 80860b07 ] && echo "$d $(cat $d/speed) Mb/s"; done'` | `5000` is USB 3. `480` is USB 2: the cockpit still works (it negotiates 640×480 @ 15), but tell Nick to move it to a blue port with a short cable and no hub. |
| Robot IP? | The request, else the cell's value: `python3 -m perceptronics cells --export ur3` (`UR_HOST`), else `/etc/perceptronics/cell.env` on an already deployed PC (`ssh <pc> sudo grep UR_HOST /etc/perceptronics/cell.env`) | No address anywhere: ask for it. It is on the pendant under Settings → System → Network. |
| Is it an e-Series on PolyScope 5, and which model? | From the PC: `ssh <pc> 'exec 3<>/dev/tcp/<robot>/29999; head -1 <&3; printf "PolyscopeVersion\nget robot model\nquit\n" >&3; head -2 <&3'` | No Dashboard banner means the address is wrong, or it is PolyScope X (no Dashboard). The URCap for this path is the PolyScope 5 one. |
| Cell name? | `python3 -m perceptronics cells`. `ur3` is the only shipped e-Series cell (UR3e, `eseries` bracket, Hand-E `PERCEPTRONICS_TIP_M=0.163`). | Another model: deploy with `--cell ur3 --robot-host <ip>`, then edit `UR_ROBOT_MODEL`, `PERCEPTRONICS_TIP_M`, `PERCEPTRONICS_BRACKET` and `PERCEPTRONICS_HOME_POSE` in `/etc/perceptronics/cell.env`, and tell Nick which values are guesses. |
| Cell subnet for the firewall? | The installer defaults to `UR_HOST`'s /24 | Only if the PC and robot sit on different subnets: pass `--allow-from <CIDR>`. |
| Internet on the PC? (the first build fetches from GitHub/sqlite.org) | `ssh <pc> 'python3 -c "import urllib.request as u; u.urlopen(\"https://github.com\", timeout=10); print(\"ok\")"'` | No internet: apt and the librealsense build can't run. Say so; the README's *Open items* has the copy-a-built-tree path. |
| Enough room? | `ssh <pc> 'df -h /var/tmp; free -m'` | The build wants ≥ 5 GiB free. Low RAM is handled (temporary swapfile). |
| Something already on :7621 / :7622? | `ssh <pc> "ss -ltnp 'sport = :7621 or sport = :7622'"` | A `perceptronics pick-server` sidecar or a hand-started cockpit is running: stop it first (`pkill -f "pick-server --bind"`). Two processes on :7622 means the cockpit runs without its pick server. |
| Recent activity by someone else? | `ssh <pc> 'journalctl -u perceptronics-cockpit --since -2h -n 20 --no-pager; ls -l /etc/perceptronics 2>/dev/null'` | A cockpit is in use (a pick in progress): check with Nick before restarting it. |

## 2. Deploy

```bash
scripts/deploy-pi.sh <user@pc> --cell ur3 --robot-host <robot-ip>
```

- The **first run compiles librealsense**, which takes tens of minutes on a Pi 5. Run it
  with `run_in_background` and poll; don't block on the default 2 min timeout. The
  build's logs are on the PC in `/var/tmp/perceptronics-build/cmake-*.log`.
- `--cell` / `--robot-host` rewrite `/etc/perceptronics/cell.env`, keeping the old one as
  `cell.env.<timestamp>`. On an **update**, leave them out so the operator's edits
  survive: `scripts/deploy-pi.sh <user@pc>`.
- Re-running is safe: the same wheel is a no-op, and librealsense is skipped when its
  stamp matches.

## 3. Read the doctor

`deploy-pi.sh` ends with `sudo perceptronics-doctor`. To re-run it alone:
`scripts/deploy-pi.sh <user@pc> --doctor-only`. What the lines mean:

- `sdk` ok, with `/opt/librealsense/lib/librealsense2.so`: the build and `REALSENSE_LIB`
  are fine.
- `camera ... usb 3.2`: good. `usb 2.1` (warn): see the USB 3 row above.
- `cockpit ... fps N, seq N`: the service is streaming. `no cockpit at ...` means the
  unit isn't up: `ssh <pc> journalctl -u perceptronics-cockpit -n 50 --no-pager`.
- `robot.reach` / `robot.primary` / `robot.rtde`: the PC's path to the controller's
  ports. `robot.control` fails in **Local** mode: state reads work, but Primary motion
  doesn't on a real e-Series until the pendant is set to Remote. That is expected, not a
  deploy fault.
- `approach`: must name the tool length Nick measured (`PERCEPTRONICS_TIP_M`).
- `handeye`: `env:` means `PERCEPTRONICS_T_FLANGE_CAMERA` from `cell.env`. After a new
  `perceptronics calibrate --apply`, delete that line and restart, or the old value wins.

Report the doctor's failures verbatim, each with its fix. Don't paraphrase them into
"mostly fine".

## 4. Point the pendant at it

Tell Nick (it's a pendant action, with no network path to it): **Installation** tab →
**URCaps** → **Perceptronic** → **Cockpit** = `http://<pc-ip>:7621` → **Save**. The
**3D Pick** program node uses the same host and learns :7622 from the cockpit.
If the URCap isn't installed, point to `urcap/perceptronic-ps5/README.md` (USB stick,
Settings → System → URCaps → +).

Verify from the robot's side of the network as far as you can:
`curl -s http://<pc-ip>:7621/api/info | head -c 300` from the laptop works only if the
laptop is inside the cell subnet (the firewall). Otherwise tunnel:
`ssh -L 7621:127.0.0.1:7621 <user@pc>`.

## 5. Update, roll back, remove

- Update: `scripts/deploy-pi.sh <user@pc>` from the newer checkout.
- Roll back: `scripts/deploy-pi.sh <user@pc> --rollback` (swaps `current`/`previous`
  and restarts).
- Remove: `ssh -t <user@pc> sudo /opt/perceptronics/deploy/install.sh --uninstall`. This
  keeps calibrations, cell.env and librealsense. `--purge` removes those too, so ask
  before using it.

## Troubleshooting

| Symptom | Cause → fix |
| --- | --- |
| `no RealSense device enumerated` while `lsusb -d 8086:0b07` lists it | udev rules not applied to a camera that was already plugged in: have it re-plugged. Check `/etc/udev/rules.d/99-realsense-libusb.rules` exists and `id perceptronics` shows `plugdev`. |
| `RS2_USB_STATUS_ACCESS` / `claim usb interface` in the journal | Same as above. If `sudo perceptronics-doctor` can't open it either, check nothing else holds it (`ss`, `ps aux \| grep perceptronics`). One process owns the camera. |
| Camera on USB 2 | The cockpit negotiates 640×480 @ 15 by itself (`describe()["negotiated"]`). The fix is physical: a blue port, a short cable, no hub, and adequate PSU current on a Pi 5. |
| Pendant: cockpit unreachable / feed blank | `ssh <pc> sudo nft list ruleset`: the controller's IP must fall inside `CELL_NET`. Re-deploy with `--allow-from <subnet>`. Also check the Cockpit field says `http://<pc-ip>:7621`, not `:7621` (that means the controller itself). |
| Pick node loops on status −4 | The D435 dropped out (frames frozen). The cockpit re-opens it. Check the journal for `Frame didn't arrive`, and re-plug if it doesn't recover. It is not a hang. |
| Cockpit log: pick port busy / runs without pick server | Something else holds :7622 (a sidecar). `pkill -f "pick-server --bind"`, then `sudo systemctl restart perceptronics-cockpit`. |
| Service in a restart loop | `journalctl -u perceptronics-cockpit -b --no-pager \| tail -50`. Usually a bad `cell.env` value. Compare with the shipped cell, fix it, restart. |
| librealsense build failed | `/var/tmp/perceptronics-build/cmake-build.log` on the PC. Out of memory shows up as `Killed` / `internal compiler error`: re-run (fewer jobs are chosen from RAM), and check `free -m`. |

## Don't

- Don't run the cockpit as root on the PC (`sudo perceptronics gui`). That is macOS
  folklore; on Linux the udev rules make it unnecessary, and a root run leaves
  `captures/` root-owned.
- Don't start a `perceptronics pick-server` sidecar next to the service (:7622 clash).
- Don't `nft flush ruleset` to "fix" reachability. Widen `--allow-from` instead.
- Don't edit the unit in `/etc/systemd/system` by hand. Change
  `deploy/pi/perceptronics-cockpit.service`, commit, and redeploy, so the repo stays the
  source of truth.
