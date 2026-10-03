# PI.md — bringing up the Raspberry Pi 5 test board

The Pi 5 arrives **Thursday 2026-10-01**. It is the **test board** for the pick PC
(`deploy/pi/`); the kit board stays the Pi 4 in `hardware/BOM.md`, with an industrial Pi 4
a possibility later. Part 1 is for Nick (hands on the board, ~15 min). Part 2 is for the
Claude session (Fable) that does the rest over SSH from the Mac Studio.

Taking the board to the UR3e afterwards: **`PLUG-AND-PLAY.md`**. The reference for what the installer does is `deploy/pi/README.md`; the procedure is the
`deploy-pick-pc` skill. **Nothing in `deploy/pi/` has run on a board yet** (written
2026-09-28, CI-checked only) — Thursday is the first run, so the job is as much to record
what happens as to get it working.

Keeping it simple (Nick, 2026-09-30): a typical Raspberry Pi OS install, the existing
deploy script, a static address set by hand. No custom image, no setup hotspot — those are
under *Later*.

| | Thursday (bench) | On the cell |
| --- | --- | --- |
| Ethernet | office LAN `10.0.0.0/24`, DHCP: internet for the build, SSH from the Mac Studio (`10.0.0.16`) | cell switch, static `192.168.3.20/24`, offline |
| Wi-Fi | not configured | not configured |
| D435 | blue USB 3 port | same |
| UR3e (`192.168.3.3`) | unplugged since 2026-09-29: the doctor's `robot.*` lines fail, expected | on |

`192.168.3.20`, not the `.10` in `SETUP.md` and the README's examples: `.10` is the Mac
Studio's cell address.

---

## Part 1 — Nick, by hand

### On the bench

- The Pi 5, its **active cooler** (the librealsense compile runs all cores for a long
  time), a microSD ≥ 32 GB and a reader.
- **The 27 W (5.1 V / 5 A) USB-C supply.** On a lesser supply a Pi 5 caps all USB ports at
  600 mA total, and the D435 is powered from that budget.
- The D435 and a short **USB 3** C-to-A cable, no hub.
- An Ethernet cable to the **office** network.
- No monitor or keyboard.

### 1. Flash

Raspberry Pi Imager → device **Raspberry Pi 5** → **Raspberry Pi OS (other) → Raspberry Pi
OS Lite (64-bit)** → the card. In the customisation step:

| Setting | Value |
| --- | --- |
| Hostname | `pickpc` |
| Username / password | `nick` / anything (console login only) |
| Wireless LAN | leave empty |
| SSH | enabled, **public-key only**, key = the Mac Studio's |

The key is the Mac Studio's, because the deploy runs from there:

    cat ~/.ssh/id_ed25519.pub        # on the Mac Studio; fingerprint SHA256:Xz1K1SaS…nzzDI

### 2. Assemble and boot

1. Cooler on, card in, **D435 into a blue (USB 3) port**, **Ethernet to the office
   network**.
2. Power on. First boot resizes the card and reboots: give it two minutes.

### 3. Check you can get in (from the Mac Studio)

    ssh nick@pickpc.local 'hostname -I; uname -m; sudo -n true && echo sudo-ok'

Expect a `10.0.0.x` address, `aarch64`, `sudo-ok`.

- Name doesn't resolve: the router's lease list, or
  `ping -c1 10.0.0.255 >/dev/null; arp -a | grep -i -E '2c:cf:67|d8:3a:dd|88:a2:9e|dc:a6:32|e4:5f:01'`.
- No `sudo-ok` (sudo wants a password): once, on the Pi —
  `echo "nick ALL=(ALL) NOPASSWD:ALL" | sudo tee /etc/sudoers.d/010-nick-nopasswd && sudo chmod 0440 /etc/sudoers.d/010-nick-nopasswd && sudo visudo -c`
- Host-key warning after a re-flash: `ssh-keygen -R pickpc.local` and `-R <ip>`.

### 4. Hand over

    Pi address on the office LAN: 10.0.0.___

Prompt for Fable: *"The Pi 5 is up at nick@10.0.0.___ — follow PI.md part 2."*

Don't upgrade, install or configure anything on it first.

---

## Part 2 — Fable, over SSH

You are on the Mac Studio in this checkout. Load the **`deploy-pick-pc`** skill and follow
it; the steps below are the first-run specifics it doesn't know. Use the Pi's **IP
address**, never `pickpc.local`: the installer's firewall drops inbound mDNS, so the name
stops resolving when the deploy finishes.

### 0. Before touching the Pi

- Session-start audit. Nick edits `TODO.md` by hand in this tree — an `M TODO.md` you
  didn't make is his; leave it, stage by path.
- Fresh branch off `origin/dev`; fixes to `deploy/pi/` are expected.
- `env -u UR_CELL python3 -m pytest tests/test_deploy_pi.py -q` green before you start.

### 1. Preflight

The skill's table, plus:

    ssh nick@<ip> '. /etc/os-release; echo $ID $VERSION_CODENAME; uname -r; python3 -V; systemctl --version | head -1'
    ssh nick@<ip> 'cat /proc/device-tree/model; free -m; df -h / /var/tmp; nproc'
    ssh nick@<ip> 'vcgencmd get_throttled; vcgencmd measure_temp; vcgencmd get_config usb_max_current_enable'
    ssh nick@<ip> 'nmcli -t -f NAME,TYPE,DEVICE con show; ip -br addr; ip route'

- Record the codename and systemd version. The image published 2026-09-15 is trixie; on
  trixie (systemd ≥ 254) the unit's restart back-off works.
- `get_throttled` must be `0x0`. `usb_max_current_enable=0` means the supply did not
  negotiate 5 A and USB is capped at 600 mA: tell Nick before blaming the camera.
- D435 link speed (the skill's sysfs one-liner): `5000` = USB 3; `480` = wrong port or
  cable.

### 2. Deploy

    scripts/deploy-pi.sh nick@<ip> --cell ur3 --robot-host 192.168.3.3

- **Background, 2 h limit, poll.** The build has never been timed on a board. `install.sh`
  sizes jobs as RAM / 1536 MiB. Watch:
  `ssh nick@<ip> 'tail -n 3 /var/tmp/perceptronics-build/cmake-build.log; vcgencmd measure_temp'`.
- A killed run is safe to repeat but **starts the build from scratch** (`install.sh`
  removes the build tree before cloning). Two failed attempts → stop and describe.
- `--robot-host 192.168.3.3` makes the firewall admit :7621/:7622 from `192.168.3.0/24`
  only, so on the office LAN the cockpit is reached through a tunnel (step 3). SSH stays
  open.
- Note the wall-clock build time from the `[install HH:MM:SS]` lines.

### 3. Verify

    scripts/deploy-pi.sh nick@<ip> --doctor-only
    ssh nick@<ip> 'systemctl is-active perceptronics-cockpit; journalctl -u perceptronics-cockpit -b -n 40 --no-pager'
    ssh nick@<ip> 'sudo nft list table inet perceptronics; ss -ltn | grep -E ":762[12]"'
    ssh -f -N -L 7621:127.0.0.1:7621 nick@<ip> && curl -s http://127.0.0.1:7621/api/info | head -c 600

(Kill the tunnel after: `pkill -f "7621:127.0.0.1:7621"`. Use another local port if a
cockpit on the Mac holds :7621.)

Good, with the robot unplugged: `sdk` ok at `/opt/librealsense/lib/librealsense2.so`;
`camera … usb 3.x`; `cockpit … fps ~30` with `seq` advancing between two reads. The
`robot.*` lines **fail** — expected, report them verbatim. `handeye` reads `env:` (the
2026-09-27 solve from `ur3.env`); leave it.

Each of the README's *Open items* gets a yes/no from this run:

| Open item | How to answer it |
| --- | --- |
| librealsense builds on arm64 Debian outside Docker | the deploy finished; note the minutes |
| `nft -c` accepts the rendered firewall | `nft list table` shows the table |
| the unit is valid | `systemd-analyze verify /etc/systemd/system/perceptronics-cockpit.service` |
| the **hardened, non-root** unit opens the D435 | frames advancing. If access fails while `sudo perceptronics-doctor --stream` (service stopped) works, look at `DeviceAllow=` / `DevicePolicy=` |
| hardening score | `systemd-analyze security perceptronics-cockpit` |
| the D435 holds on the Pi 5's USB budget | 10 min streaming: `seq` advancing, `get_throttled` still `0x0`, no `Frame didn't arrive` in the journal |
| per-frame cost | `curl -s -o /dev/null -w '%{time_total}\n' http://127.0.0.1:7621/api/color.png` ×10; `top` for the cockpit process |

### 4. The cell address (when Nick is ready to move the cable)

Set it last: this cuts the office-LAN session, and the Pi is only reachable again once
its cable is on the cell switch and the Mac Studio's cell interface (`192.168.3.10`) is up.

    ssh nick@<ip> 'nmcli -t -f NAME,TYPE,DEVICE con show'      # the ethernet connection's name
    ssh nick@<ip> 'sudo nmcli con mod "<name>" ipv4.method manual ipv4.addresses 192.168.3.20/24 && sudo reboot'

No gateway: the cell network has no route out, and in service the box is offline. Then
Nick moves the cable, and:

    ssh nick@192.168.3.20 'ip -br addr; sudo perceptronics-doctor'

Check the address survived the reboot. If the Pi doesn't come back on `.20`, it is a
10-minute re-flash, or a monitor and keyboard and `nmcli con mod "<name>" ipv4.method auto`.
To bring it back to the office LAN for an update that needs internet, the same command
over SSH from the cell side.

### 5. If something breaks

- Fix it **in the repo** (`deploy/pi/`, `scripts/deploy-pi.sh`), regression test first in
  `tests/test_deploy_pi.py`, commit, redeploy. Never hand-edit the unit or the firewall on
  the Pi.
- Build failed: `/var/tmp/perceptronics-build/cmake-{configure,build}.log`. `Killed` /
  `internal compiler error` = memory; a thermal crawl shows in `measure_temp`.
- The Pi is expendable. The UR3e's controller is not — nothing in this runbook sends
  motion, and nothing should.

### 6. Write it down, then PR

- `deploy/pi/README.md`: each verified *Open item* moves out with its date and number
  (build minutes, OS codename, USB link, frame timings); the `192.168.3.10` examples →
  `.20`; drop the gateway from the static-address example.
- `SETUP.md` §2 and §4: the Pi 5 test board, its address, "first run 2026-10-0x".
- `TODO.md`: close the pick-PC OS question with the codename you saw.
- Memory: one entry for the Pi (address, user, what was verified).
- Draft PR into `dev` if anything under `deploy/` or `scripts/` changed; docs-only can
  auto-merge.

---

## Later (not Thursday, not designed yet)

Nick's direction for the shipped product, 2026-09-30. Recorded so it isn't lost; none of it
is built, and it is far off.

- **Offline at runtime, hardwired to the robot.** Internet only during setup.
- **A pre-built SD image**: flash, boot, commission. Nothing compiles or downloads on a
  shipped unit.
- **SSH off** on shipped units.
- **Commissioning over a hotspot**: the box raises a Wi-Fi network during commissioning
  only, with a web page to set the wired address (DHCP or static). Stuck → power-cycle
  brings it back.
- **Board**: possibly an industrial Pi 4.
- **librealsense**: whichever build reduces dependencies and increases portability
  (`TODO.md`).

### Not in this session (needs the robot and Nick at the pendant)

1. UR3e powered, `sudo perceptronics-doctor` green on `robot.*`.
2. Pendant: remove RealSense Pilot, install Perceptronic 0.9.0 from the USB stick;
   **Installation → URCaps → Perceptronic** shows the picture with the Cockpit field at its
   default `192.168.3.20` (`PLUG-AND-PLAY.md` §3).
3. Hand-eye on the Pi if the wrist moved since 2026-09-27 (`PLUG-AND-PLAY.md` §6). The solve
   is saved to the hand-eye file and survives restarts; nothing to delete.
4. The pick kit's first run on the cell (`TODO.md`, 2026-09-28 entry).
