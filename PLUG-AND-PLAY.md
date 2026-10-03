# PLUG-AND-PLAY.md — the Pi pick PC on the UR3e

How to put the Raspberry Pi pick PC (`deploy/pi/`) on the **UR3e cell** — the robot at
`192.168.3.3` that ran the cockpit picks on 2026-09-27 (`SETUP.md` §1) — with as little to
set as possible. Three cables, one URCap install, nothing typed.

Generic version, for any PolyScope 5 robot: `docs/pick-kit.md`. Bringing up a new Pi
board: `PI.md`. What the installer does, line by line: `deploy/pi/README.md`.

## What is already true (nothing to set)

| | Out of the box | Where it comes from |
| --- | --- | --- |
| Pi's address on the robot's cable | **192.168.3.20/24**, no gateway | `install.sh --cell-address` (default); a port already on another network is left alone |
| Robot's address | **192.168.3.3** — the UR3e is set to it statically, and a robot on **DHCP** gets it from the Pi | `perceptronics/cells/ur3.env` `UR_HOST`; `perceptronics-cell-dhcp` (one lease, only when no other DHCP server answers; dnsmasq pings first, so it never hands out an address in use) |
| Pendant's **Cockpit** field | **empty = 192.168.3.20** (the field shows it) | URCap 0.9.0 `Cockpit.DEFAULT_HOST` |
| Cockpit + pick server | start at boot (:7621, :7622), restart on failure, re-open a camera that drops out | `perceptronics-cockpit.service` |
| Robot link | read over RTDE in **Local** mode; reconnects by itself when the robot comes up after the Pi | the cockpit |
| Firewall | the robot's network may reach :7621/:7622; SSH from anywhere; nothing else | `/etc/nftables.conf` |
| Tool | Hand-E fingertips 0.163 m past the flange; picture pose 0.37 m up | `ur3.env` |
| Hand-eye | the 2026-09-27 solve, in `/var/lib/perceptronics/captures/calibration/handeye.json` — a calibration made on the Pi replaces it and **survives reboots and redeploys** | `install.sh` (`handeye_out_of_env`) |

## 1. At the desk (once)

1. **Deploy the Pi** from the Mac. It can be on any network that reaches the Pi; Wi-Fi is fine:

       scripts/deploy-pi.sh nick@10.0.0.56 --cell ur3

   It ends with the doctor's verdict. Away from the robot, `robot.*` lines fail; that's
   expected. The line that matters here is `network`. On the cell it reads
   `this machine is 192.168.3.20 on 192.168.3.0/24; the pendant's Cockpit field can stay empty`.
   (A flashable image is in progress, `scripts/pi-image.sh`, PR #49. When it lands, it
   replaces this step with flash + seed.)
2. **Make the USB stick:**

       scripts/urcap5-usb.sh

   It puts `perceptronic-ps5-0.9.0.urcap` and its auto-install file on the "URE MODELS"
   stick, without macOS `._` files, then ejects.

## 2. At the robot: three cables

| Cable | From | To |
| --- | --- | --- |
| USB 3 (the short Newnex one, no hub) | D435 on the wrist | a **blue** USB port on the Pi |
| Ethernet | Pi | the cell switch (where the UR3e and the Mac's `.10` are), or straight into the controller's Ethernet port |
| Power | Pi's USB-C | the **27 W (5.1 V / 5 A)** supply. On less, the Pi 5 caps USB at 600 mA and the camera browns out |

Power-up order doesn't matter. Give the Pi a minute after power-on.

## 3. On the pendant (once)

1. **URCap.** If ☰ → Settings → System → URCaps lists **RealSense Pilot**, select it, tap
   **–** and restart. It's a different bundle; its node data doesn't carry over. Then plug
   in the stick:
   - **automatic:** Settings → Security → General → **Run magic files** on, arm powered
     **off**, plug the stick in, and the robot installs and restarts by itself;
   - **by hand:** ☰ → Settings → System → URCaps → **+** → `perceptronic-ps5-0.9.0.urcap`
     → Open → Restart.

   (Step by step with pictures: `urcap/perceptronic-ps5/README.md` §Install on the robot.)
2. **Network: leave it.** The UR3e is already `192.168.3.3`. Check under ☰ → Settings →
   System → Network that it reads 192.168.3.3 / 255.255.255.0. A robot that says
   *Disabled network* or sits on another subnet: select **DHCP** → **Apply**, and the Pi
   gives it 192.168.3.3.
3. **Installation tab → URCaps → Perceptronic.** The Cockpit field says `192.168.3.20` and
   the live picture appears within seconds. Nothing to type. Then ☰ → **Save Installation**
   so the node's data (pick areas, a typed address) is kept.

## 4. The first pick program

Program tab, top to bottom:

    Gripper: open        ← Robotiq's node (Hand-E)
    3D Pick              ← URCaps → 3D Pick: part size, one picture point (+), pick order
    Gripper: close       ← Robotiq's node
    If rs_pick_found     ← lift, place

**3D Pick** drives no gripper. It ends with the fingertips at the grip, and runs in
**Local** mode (it is the robot's own program). Picture point: move the arm so the camera
sees the parts from ≥ 0.3 m (the picture pose, 0.37 m up, is right) and tap **+**. The
node's screen draws the parts it will pick in green, numbered, and near misses in yellow
with the reason (`urcap/perceptronic-ps5/README.md` §3D Pick).

## 5. Check it

From the Mac, on any network that reaches the Pi:

    scripts/deploy-pi.sh nick@10.0.0.56 --doctor-only

What READY looks like on this cell:

| Line | Reads |
| --- | --- |
| `network` | `this machine is 192.168.3.20 on 192.168.3.0/24; the pendant's Cockpit field can stay empty` |
| `robot.reach` / `robot.primary` / `robot.rtde` | dashboard, 30001 and 30004 at 192.168.3.3 open |
| `camera` | the D435, held by the cockpit, ~30 fps, USB 3.x |
| `handeye` | `file:/var/lib/perceptronics/captures/calibration/handeye.json` |
| `approach` | fingertip, 0.163 m |
| `robot.control` | `LOCAL` gates the **cockpit's** moves and `calibrate` only (the verdict says STATE ONLY); the 3D Pick node doesn't care |

The cockpit's own page is at http://192.168.3.20:7621 from the Mac's cell interface (`.10`),
or from anywhere through `ssh -L 7621:127.0.0.1:7621 nick@10.0.0.56`.

## 6. Calibrate when the camera has moved

The Pi starts with the 2026-09-27 solve (RMS 2.5 mm). **Re-run it if anything on the wrist
moved since** (the bracket re-printed or re-seated, the camera re-screwed). It's also worth
doing if picks land consistently off by more than a few mm, or if the cockpit's LEVEL lamp
reads over ~0.5° (the table is flat, so any tilt it shows is calibration error).

1. Pendant to **Remote** (top right). Calibrating moves the arm, and on hardware moves over
   the network need Remote.
2. One white block under the camera, arm at the picture pose.
3. From the Mac:

       ssh nick@10.0.0.56 sudo -u perceptronics /opt/perceptronics/current/bin/perceptronics \
           --cell /etc/perceptronics/cell.env calibrate --cockpit http://127.0.0.1:7621 --apply

   The arm orbits the block (3 ranges × 13 views, a few minutes) and the solve is applied at
   once and saved on the Pi. It stays through reboots and redeploys.
4. Pendant back to **Local** if that is how the cell runs.

## 7. When it doesn't work

What the pendant says (the URCap names the cause first, most likely first):

| On the pendant | Check |
| --- | --- |
| *No answer from the camera computer at 192.168.3.20* + *Robot network: …DHCP…* | The Pi's Ethernet link lights. Pi powered. Robot's Network screen on 192.168.3.x/24. `ssh nick@10.0.0.56 ip -br addr` shows `eth0 … 192.168.3.20/24` |
| *A computer answers at 192.168.3.20, but not the camera program* | `ssh nick@10.0.0.56 systemctl status perceptronics-cockpit`; `journalctl -u perceptronics-cockpit -n 50` |
| *The camera computer is on, but its camera gives no picture* | The D435 in a **blue** port, no hub; the 27 W supply; re-plug it (the picture comes back by itself) |
| Parts drawn but picks miss by cm | §6, calibrate |
| `3D Pick: no pick - …` popup | It names the reason; the node's screen shows the parts in green / yellow |
| The robot got no address (DHCP) | `journalctl -u perceptronics-cell-dhcp`: *another DHCP server answered* means the Pi is on a network that has one, so it stays quiet. Give the robot a static 192.168.3.3 |

## Verified 2026-10-02 (pickpc, no cable to the robot yet)

`scripts/deploy-pi.sh nick@10.0.0.56 --cell ur3` from this branch: the installer kept
Raspberry Pi OS's own `netplan-eth0` profile (it already gives eth0 192.168.3.20/24) instead of
adding a second, seeded the hand-eye file from the profile (the cockpit's `/api/info` says
`file:/var/lib/perceptronics/captures/calibration/handeye.json`), started the DHCP server
waiting for eth0, installed the link-up hook; the D435 streams at 30 fps on USB 3.2; the
doctor's `network` line reads *reached from 10.0.0.56, not on its network* — right, no cable.

## Not verified yet (2026-10-02)

- The Pi has never run against the real UR3e: Dashboard / RTDE from the Pi, the pendant
  reaching 192.168.3.20. Every robot-side check so far was against simulators.
- URCap 0.9.0 (and 0.8.0's 3D Pick) has never been on a pendant.
- The DHCP lease path has never served a robot. The probe and dnsmasq's config were
  checked on the Pi; a robot taking the lease has not been.

When one of these is seen working, strike it here and add a dated line to `SETUP.md`.
