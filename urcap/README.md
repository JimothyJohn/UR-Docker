# <img src="perceptronic.svg" width="28" align="top"> Perceptronic — the wrist camera inside PolyScope X

*URCap `advin/perceptronic` since 0.4.0 (2026-09-30; vendor ID from advin.io, owner Nick Armenta).
PolyScope X treats an earlier identity as a different URCap — delete it before installing this:
`urcapx.py delete nickarmenta perceptronic …` (0.3.0) or `urcapx.py delete olympus-controls
realsense-pilot …` (RealSense Pilot).*

Copyright © 2026 Nick Armenta.

> **e-Series robot on PolyScope 5?** Use the Installation-node build instead:
> [`perceptronic-ps5/`](perceptronic-ps5/README.md) (`dist/perceptronic-ps5-0.9.0.urcap`).

A URCap for **PolyScope X** (PolyScope 10) robots. It adds a **Perceptronic**
node under **Application** that shows the live colour feed from an Intel
RealSense D435 on the tool flange, right in PolyScope's own screen:

- **hover** over the picture to read the distance to any point,
- **click** an object to outline it and get its position in the robot's base
  frame, an approach pose above it, and whether the arm can reach it,
- **Move (PolyScope)** hands that pose to PolyScope's own hold-to-move screen;
  **Move (cockpit)** moves the arm there through the safety-checked cockpit.

The URCap is only the window. The camera is plugged into a computer next to the
robot (laptop, Jetson, mini-PC) that runs the **cockpit** (`perceptronics gui` from
this repository). The pendant shows the cockpit's picture over the cell network.

```
 ┌───────────── PolyScope X pendant ─────────────┐          ┌──── computer next to the robot ────┐
 │ Application → Perceptronic  (this URCap)      │  HTTP    │ perceptronics gui  (the cockpit)       │
 │   feed · hover · click · Move                 │ ───────► │   :7621   D435 on USB               │
 └───────────────────────────────────────────────┘          │   talks to the robot on 30001/30004 │
                                                            └─────────────────────────────────────┘
```

> **PolyScope X only.** PolyScope 5 e-Series controllers (UR3e/UR5e/UR10e… on
> 5.x software) can't run URCap X packages. On those, use the cockpit in a browser
> instead (`perceptronics gui`, [docs/realsense.md](../docs/realsense.md)).

---

## Quick start

1. **Download** [`dist/perceptronic-0.6.0.urcapx`](dist/perceptronic-0.6.0.urcapx)
   (on GitHub: open the file, then the download button). The single file is the
   whole URCap.
2. **Install it** on the robot (see [Install the URCap](#1-install-the-urcap)).
3. **Start the cockpit** on the camera computer, with the robot's address:
   ```bash
   python3 -m perceptronics --cell mycell.env gui --bind 0.0.0.0 --cors http://<robot-ip>
   ```
4. On the pendant: **Application → Perceptronic**, type
   `http://<camera-computer-ip>:7621` in **Cockpit**, press **Save**. The feed
   appears.

If the feed doesn't appear, the node says what is wrong in plain words and, when
it's the `--cors` value, prints the exact flag to restart the cockpit with. See
[When it doesn't work](#when-it-doesnt-work).

---

## What you need

<!-- urcap-target -->Built and tested for **PolyScope X 10.13.1** (held at 10.13, 10.13; simulator `universalrobots/ursim_polyscopex:10.13.0`, URCap SDK 6.5.65 / contribution-api 21.3.266) — pinned in [`target.json`](target.json) and kept current by `.github/workflows/urcap-track.yml`.<!-- /urcap-target -->

| | |
| --- | --- |
| **Robot** | A PolyScope X controller on the release above (see [What has been verified](#what-has-been-verified)). The admin password (UR's factory default is `easybot`; the first use asks you to change it). |
| **Camera** | An Intel RealSense **D435** on the tool flange. A printable bracket is in [`hardware/d435-tool-bracket/`](../hardware/d435-tool-bracket/) (e-Series ISO-50 and UR20 prints). |
| **Computer** | Windows, macOS or Linux (amd64/arm64) on the **same network as the robot**, with the D435 on a **USB 3** port, Python 3.10 or newer and librealsense ([docs/realsense.md](../docs/realsense.md) covers each OS; macOS needs `sudo`). |

---

## 1. Install the URCap

Pick one of the two ways. You only do this once per robot (and again to update).

### A. On the pendant, from a USB stick

1. Copy `perceptronic-0.6.0.urcapx` onto a USB stick and plug it into the
   teach pendant.
2. Open the **☰ menu** (top-left) → **System Manager** → **URCaps**, and
   unlock it with the **admin password**.
3. Add a URCap and pick `perceptronic-0.6.0.urcapx` from the USB stick.
4. When it's listed, open **☰ → Reload**.
   **Perceptronic** now appears under **Application**.

### B. Over the network, from any computer

[`urcapx.py`](urcapx.py) is one file that needs only Python 3, with no packages
to install. Download it next to the `.urcapx` and run:

```bash
python3 urcapx.py install perceptronic-0.6.0.urcapx --host <robot-ip> --port 80
```

It posts the package to the same endpoint PolyScope's System Manager uses, which
**doesn't need Remote mode**. `--port 80` is the robot's web port (the
simulator in this repo publishes it on `8000`, the default). When it prints
`installed`, reload PolyScope (**☰ → Reload**, or refresh the browser tab).

Other commands from the same file:

```bash
python3 urcapx.py list --host <robot-ip> --port 80                                          # what's installed
python3 urcapx.py install perceptronic-0.6.0.urcapx --host <robot-ip> --port 80 --replace   # update
python3 urcapx.py delete advin perceptronic --host <robot-ip> --port 80        # uninstall
```

---

## 2. Prepare the robot (once)

The cockpit reads the arm's pose and sends moves over the robot's **Primary**
interface. PolyScope X ships with it off, and runs scripts from it only in
**Remote** control mode:

1. **☰ → Settings → Security → Services** (admin password): enable **Primary
   Client Interface** and **RTDE**.
2. Switch the robot to **Remote** control on the Safety screen (operational-mode
   password; UR's factory default is `operator`). There's no way to do this
   over the network.

In **Local** mode the feed and hover still work, but a click can't be turned into
a base-frame point. The robot silently ignores the pose query, and the node shows
`locate failed: no TCP pose/offset surfaced on the Primary broadcast`.

---

## 3. Start the cockpit on the camera computer

Get this repository and its environment:

```bash
git clone https://github.com/JimothyJohn/perceptronics.git && cd perceptronics
python3 -m pip install -e ".[vision]"   # optional: numpy/OpenCV for the classic-CV backend; the cockpit itself needs nothing installed
```

Describe your cell in one small file (copy [`perceptronics/cells/ur20.env`](../perceptronics/cells/ur20.env), the
PolyScope X template) and fill in the robot:

```bash
# mycell.env
UR_HOST=192.168.1.20          # the robot's IP address
UR_PLATFORM=polyscopex
UR_ROBOT_API_PORT=80
UR_PRIMARY_PORT=30001
UR_RTDE_PORT=30004
UR_ROBOT_MODEL=UR20           # sizes the reach check: UR3e … UR30
PERCEPTRONICS_BRACKET=ur20       # which bracket print: eseries (ISO-50) or ur20
```

Check everything before the first run (it prints a fix next to each failure):

```bash
python3 -m perceptronics --cell mycell.env doctor
```

Then start the cockpit so the pendant can reach it:

```bash
python3 -m perceptronics --cell mycell.env gui --bind 0.0.0.0 --cors http://<robot-ip>
```

- `--bind 0.0.0.0` lets the pendant reach the computer (the default is this
  computer only).
- `--cors` names the page that's allowed to call the cockpit: the PolyScope X
  page on the pendant. Its exact value depends on the controller. If yours
  differs, the node tells you the right one (next section). You can also set
  `PERCEPTRONICS_CORS` in `mycell.env` instead of passing the flag.
- The cockpit listens on port **7621**. Open `http://localhost:7621` on the
  computer itself to see the full cockpit.
- macOS: prefix with `sudo` (the camera needs it) and launch from a local
  Terminal, not over SSH.

**Calibrate once per camera mounting.** Without it, positions come from the
bracket's nominal geometry and can be off by a few centimetres. With the cockpit
running, put a block under the camera and run
`python3 -m perceptronics --cell mycell.env calibrate --apply` in a second terminal (the
arm orbits the block and the result is saved), or use the cockpit's **Calibrate
hand-eye**. See [docs/realsense.md §Hand-eye](../docs/realsense.md).

---

## 4. Use it

1. On the pendant: **Application → Perceptronic** (add it to a program like
   any other node).
2. **Cockpit**: `http://<camera-computer-ip>:7621`, then **Save**. It's stored in
   the node, so you do this once. Left empty, the node looks for a cockpit on the
   robot itself, which only works if the cockpit runs there.
3. The feed appears. Hover to read depth. **Click** an object: it's outlined, and
   the panel shows the base-frame point, the approach pose and whether it's
   reachable.
4. **Move (PolyScope)** opens PolyScope's hold-to-move screen for the approach
   pose. **Move (cockpit)** drives there directly through the cockpit's safety
   envelope (speed, reach and step limits; every action is logged). **Bring
   up** powers on and releases the brakes. **STOP** stops the arm.

---

## 5. Pick with the **3D Pick** node

The same node as the PolyScope 5 kit's, for PolyScope X, at functional parity (0.6.0 here
= 0.8.0 there; a test holds the two nodes' URScript line for line): **one move sequence
with no children** that starts with the survey and ends with the tool at the grip on the
next part in the order you choose. **It does not control the gripper** — your program
opens it before the node and closes it after ([`../docs/pick-kit.md`](../docs/pick-kit.md) for the
whole kit, [`perceptronic-ps5/README.md`](perceptronic-ps5/README.md) for every screen in
detail). The camera computer's address and the **pick areas** live in **Application →
Perceptronic** (above), on its **Pick areas** tab: touch the table with the fingertips at a
corner, along one edge and on the far side for each area. The map beside them draws the
base, how far this arm reaches (its rated reach, named on its circle) and every area in
it. There are no reach margins: which parts can be picked is the arm's kinematics' answer.

1. **Program → + → URCaps → 3D Pick.** The tree row shows the part, the picture points
   and the order; **Teach & options…** opens the node's screen as a PolyScope dialog.
   Nothing in it scrolls.
2. In it: the live picture, with the parts that will be picked in **green, numbered** in
   the pick order, and the candidates that are nearly the part and will not be in
   **yellow, each with why** (`too long`, `out of reach (no joint solution)`, `no room for
   a finger beside it`, …); the **Picture / Depth** toggle in its top right corner (the depth
   as a heatmap); the picture points as a grid of numbered buttons — **+** adds one from
   PolyScope's joint positions, **Go** opens PolyScope's move screen to the selected one,
   its second line chooses the pick area it looks at; the **pick order** tiles; **Check
   approach** (PolyScope's IK + move screen over the first part, fingertips *Approach* mm
   over its top); and **Options**, two tabs: **Part** (Box or Cylinder — standing on its end —
   its size, the tolerance) and **Approach** (the approach, the grip depth, the **Grip
   check** with its **Finger room** — 20 mm clear on each side the fingers come down on —
   **Grip across the long side** for a box, the **Closer look**). Speeds have no settings,
   and there is nothing about the gripper.
3. Open the gripper **before** the node; **after** it, close the gripper, then
   `If rs_pick_found` → lift, place.
   `rs_pick_found` and `rs_pick_loc` (which picture point it came from) are program
   variables. A node saved by 0.4.0 keeps its picture points and part; whatever was inside
   it moves after it, and its **After picture N** children are gone with the node type.

When the camera computer does not answer, the picture's place says what to check — cables,
the IP address, the firewall, or the camera's USB cable — and the long story (the
exception, the `--cors` origin to start the cockpit with) goes to the browser console.

The node's URScript runs inside your program (Local mode, no Primary): it talks to the
cockpit's pick server on `:7622` (protocol 2, the same as the PolyScope 5 node — one
cockpit serves both robots). Anything that stops a run says why in a popup and in the
robot's log.

---

## When it doesn't work

The node tells a cockpit that **refuses** the pendant apart from one that
**isn't there**. Read its message first.

| The node says | What's wrong | Fix |
| --- | --- | --- |
| *running but refuses this page (origin X)* … *Restart it with --cors X* | The cockpit is up but wasn't started with this page's origin | Restart the cockpit with exactly the `--cors X` it shows. Origins are exact: `http://localhost:8000` and `http://127.0.0.1:8000` are different. |
| *nothing answers at …* · *listens on loopback only unless started with --bind 0.0.0.0* | Cockpit not running, started without `--bind 0.0.0.0`, wrong IP/port, or a firewall | Start it with `--bind 0.0.0.0`; check the IP; allow port 7621 through the computer's firewall. |
| *"localhost" is the machine running this browser* | `localhost` in the node means the robot, not your computer | Use the camera computer's IP. |
| *answered HTTP 404 on /api/color.png* | The cockpit is older than the URCap | Update the repository (`git pull`) and restart it. |
| *locate failed: no TCP pose/offset surfaced on the Primary broadcast* | The robot ignored the pose query: Local mode, Primary interface off, or the controller was busy for a moment | Enable Primary (§2), switch to Remote, and click again. |
| Feed works but **OUT OF REACH** on every click | The object is farther than the arm reaches, or `UR_ROBOT_MODEL` is wrong | Move the part closer; check the model in `mycell.env`. |
| Point lands a few cm off | No hand-eye calibration yet | Calibrate (§3). |
| Perceptronic isn't under Application after installing | PolyScope hasn't reloaded | **☰ → Reload**, or refresh the browser tab on a simulator. |
| `urcapx.py install` prints `409` / *already installed* | That version is installed | Add `--replace`. |

The cockpit's own log names every refused origin (`CORS: refused a page from …`),
and `http://<camera-computer-ip>:7621/api/info` lists them under `cors.refused`.

**Security.** The cockpit has no login and can move the robot. `--bind 0.0.0.0`
belongs on a trusted cell network only, and `--cors` should name only your
pendant's origin.

---

## Try it without a robot

The repository's PolyScope X simulator runs the whole thing on one machine, and
a synthetic camera stands in for the D435:

```bash
make simx-up          # PolyScope X 10.13.0 → http://localhost:8000 (HOST_ARCH=arm64 on Apple Silicon)
make urcap-install    # installs dist/perceptronic-*.urcapx into it
make urcap-cockpit    # a fake cockpit on :7621 with --cors for the simulator
```

Open `http://localhost:8000`, **Application → Perceptronic**, leave
**Cockpit** empty, and the synthetic scene appears. Locate/Move need a real robot
link behind the cockpit.

---

## What has been verified

| | Status |
| --- | --- |
| Package builds reproducibly; `dist/` equals a fresh build | CI (`tests/test_urcap.py`) |
| Installs through the System Manager endpoint in Local mode (201; 409 on duplicate; delete) | PolyScope X **10.13.0 simulator**, 2026-09-26 |
| Node loads under Application; feed, hover depth, click → segment → locate call; cockpit URL persists across reloads | 10.13.0 simulator, headless Chromium, against a synthetic cockpit |
| Node tells a CORS refusal from a dead port and prints the fix | 10.13.0 simulator |
| All of the above plus PolyScope's IK / FK services answering the node, on **10.14.0** (verified; the target is held at 10.13 since — `target.json` `hold`) | `urcap/e2e.py`, 2026-09-27; re-run on every change and every new UR release by CI |
| Install from a USB stick on a physical pendant (§1A) | **not yet**: the steps follow PolyScope X's System Manager but haven't been walked on hardware |
| Locate + Move (cockpit) against a real PolyScope X arm | **not yet** |
| `perceptronics calibrate` (orbit hand-eye) as a command on hardware | **not yet**: the same orbit, scripted, gave RMS 4.4 mm on a UR3e (2026-09-25); the cockpit's touch-and-click calibration is the proven path |
| Move (PolyScope): IK + hold-to-move accept the pose | **not yet**: compare against the cockpit's approach pose on first use |
| **3D Pick** program node (named Perceptronic Pick until 0.5.0): in the toolbox, its row in the tree, the dialog (feed, a picture point from PolyScope's joints, Options), the two program variables declared, and its URScript **compiled and run by PolyScope X** (Play): NEXT → movej → FIND against a synthetic cockpit over the network, the failure popup naming the pick server's answer | 10.13.0 simulator (UR3), 2026-09-29; the toolbox → dialog → verdict part is in `urcap/e2e.py` |
| Pick areas taught from PolyScope's joint positions + DH; robot model read from PolyScope for the reach map and the kinematics | 10.13.0 simulator, 2026-09-29 |
| A real pick with the Pick node on a PolyScope X arm (and the Robotiq Hand-E URCap for PolyScope X exposing `:63352` the way the e-Series one does) | **not yet** |

## Tested PolyScope X releases

`urcap/psx_matrix.py` runs the whole e2e (install, the application node, the Pick node's
row → dialog → a picture point → the verdict) against the **newest PolyScope X releases
from 10.8** (the floor the URCap is built for) that UR publishes a simulator image for;
`.github/workflows/urcapx-matrix.yml` runs it on every change to the URCap and weekly
(`make urcapx-matrix`). 2026-09-30, all eight pass: **10.14.0, 10.13.0, 10.12.1, 10.12.0,
10.11.0, 10.10.0, 10.9.0, 10.8.0** (10.6 and 10.7 were dropped: their simulators' web app
does not start on GitHub's runners, and nobody will run them). What the older ones needed:

- **10.8–10.9** have no `convertJointPositionsToTcpPose`: Move (PolyScope) and Check
  approach can't learn PolyScope's active TCP, so they hand PolyScope the flange target
  as the TCP and say so — set PolyScope's TCP to the flange, or use Move (cockpit).
- **10.8–10.11** have no `variableService`: the two program variables are declared
  through the older `symbolService.generateVariable` instead (same names).

## Releases

Pushing a `urcapx-v<version>` tag (`git tag urcapx-v0.3.0 && git push fork urcapx-v0.3.0`)
publishes the committed `dist/perceptronic-<version>.urcapx` and its sha256 as a GitHub
Release (`.github/workflows/release-urcapx.yml`) after `urcapx.py release-check` proves it
is the tag's version and exactly what the tagged sources package to, and the URCap's tests
pass. So: bump `version` in `perceptronic/manifest.yaml`, `make urcap-package`, commit
`dist/`, then tag. The PolyScope 5 URCap's `urcap5-v*` tags are a separate line.

Building or changing the URCap: [DEVELOPING.md](DEVELOPING.md).
