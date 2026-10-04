# The RealSense pick kit for UR e-Series (PolyScope 5)

A wrist-mounted RealSense D435, a small camera computer beside the robot, and a URCap. The
operator tells the robot the part's size and where to look; the robot finds every part of
that size, numbers them in the order it will pick them, and picks them one by one —
approaching with the fingertips 25 mm over the part's top, fingers fully open, closing
across its short side.

| Piece | Where | What it is |
| ----- | ----- | ---------- |
| Bill of materials | [`hardware/BOM.md`](../hardware/BOM.md) | Every part with links, the price seen 2026-09-28/29 and its history (kit ≈ $785 per cell with a Raspberry Pi 4; ≈ $1,420 with a RevPi Connect 5, ≈ $1,112 with a CompuLab IOT-GATE-RPI5 for a 24 V cabinet PC) |
| Camera bracket | [`hardware/d435-tool-bracket/`](../hardware/d435-tool-bracket/) | Parametric CadQuery source; print-ready STL and STEP for e-Series (ISO 9409-1-50-4-M6) and UR20 flanges in `out/`; the spec, fasteners and the nominal camera pose in its README |
| Camera computer | [`deploy/pi/`](../deploy/pi/README.md) | A Raspberry-Pi-class arm64 box on minimal Debian: one command installs librealsense, the cockpit as a hardened systemd service (:7621 HTTP, :7622 pick server) and a firewall |
| URCap | [`urcap/perceptronic-ps5/`](../urcap/perceptronic-ps5/README.md) | `urcap/dist/perceptronic-ps5-0.9.0.urcap`: the **3D Pick** program node and the **Perceptronic** installation node |
| URCap for PolyScope X | [`urcap/README.md`](../urcap/README.md) §5 | `urcap/dist/perceptronic-0.7.0.urcapx`: the same **3D Pick** node for a PolyScope X robot (the two write the same URScript); the pick areas on the **Perceptronic** application node; one cockpit serves both robots |
| Self-deploy | [`.claude/skills/deploy-pick-pc/`](../.claude/skills/deploy-pick-pc/SKILL.md) | Claude Code deploys, checks, updates or rolls back the camera computer: "deploy the pick PC at 192.168.3.10" |
| Tested PolyScope | [`.github/workflows/urcap5-matrix.yml`](../.github/workflows/urcap5-matrix.yml) | Every change: the URCap loads and the node's own URScript picks on PolyScope 5.24, 5.25 and 5.26 (URSim) |

**See it first, on a desktop:** `python3 urcap/preview5.py` opens the pendant's screens
in a window, fed by a simulated camera computer (or `--cockpit http://<ip>:7621`, a real one).

## Setting up a cell

1. **Mount the camera.** Print `d435_tool_bracket_eseries.stl` (the BOM names the
   material), fit it between the flange and the gripper with the tool's bolts, the D435
   dropped into the bracket's lip on its one 1/4-20, the cable clicked into the bracket's clip
   and along the arm (strain relief at the wrist).
2. **Deploy the camera computer.** Flash Raspberry Pi OS Lite 64-bit (or Debian arm64),
   plug the D435 into a **USB 3** port and its Ethernet into the robot (the deploy gives
   that port 192.168.3.20 and serves the robot 192.168.3.3 over DHCP), then
   from a laptop with this repo — or ask Claude Code to "deploy the pick PC":

       scripts/deploy-pi.sh pi@192.168.3.10 --cell ur3 --robot-host 192.168.3.3

   It ends with `perceptronics doctor`'s verdict: camera, robot ports, hand-eye, tool length.
3. **Install the URCap** from a USB stick: Settings → System → URCaps → **+** →
   `perceptronic-ps5-0.9.0.urcap` → Restart.
   Or leave it to the stick: `scripts/urcap5-usb.sh` writes a magic file that installs it by
   itself when **Settings → Security → General → Run magic files** is on
   (`urcap/perceptronic-ps5/README.md`, *Or let the stick install it*).
4. **Robot network → DHCP** (Settings → System → Network → DHCP → Apply), then
   **Installation → URCaps → Perceptronic**: the Cockpit field already says `192.168.3.20`
   and the live picture appears. A camera computer at another address: type it there.
5. **Calibrate the camera to the flange** once (`perceptronics calibrate` on the camera
   computer: the arm orbits a part and solves where the camera sits; `docs/realsense.md`
   §Hand-eye without a mark). Re-run it if the bracket is ever re-mounted.
6. **Pick areas** (same Installation node, second tab): for each place parts are picked
   from, **New area** and touch the table with the fingertips at its corner, along one
   edge and on the far side. The map shows each area against how far the arm reaches;
   which parts can be picked is the arm's kinematics' answer, part by part — there are no
   reach margins to set.

## Programming a pick

**Program tab → URCaps → 3D Pick**, inside a loop. The node is one line — a move sequence
with no children: it starts with the survey and ends with the tool at the grip, the fingers
around a part. It does not control the gripper: put your **Open gripper** before it and
your **Close gripper** after it.

1. **+** adds a picture point where the arm is — as many as needed (up to 12); each looks
   at a pick area (tap its second line to choose) or the live table. The arm visits them
   in turn.
2. Choose the **pick order**.
3. **Options** — two tabs. **Part**: box or cylinder (standing on its end), its size, the
   tolerance. **Approach**: how far over the top (25 mm), the grip depth, the grip check
   and its finger room (20 mm clear on each side the fingers come down on), whether a box
   is gripped across its long side, the closer look.
4. Put what happens to the part **after** the node: close the gripper, then
   `If rs_pick_found` → lift, place.
   `rs_pick_loc` says which picture point it came from.
5. **Check approach** opens PolyScope's move screen over the first part: hold Move to see
   the fingers arrive 25 mm over it.

The picture shows the parts that will be picked in green, each with its number in the
pick order, and what nearly is the part and will not be — a little off its size, out of the
arm's reach, no room for the fingers — in yellow with why; the toggle in its corner
switches it to the depth as a heatmap. A run: the survey (or, when the last one already
showed more parts, straight to the next one), a closer look to re-measure it, the approach,
the descent to the grip. Anything that stops a run says why in a popup and on the camera computer's log
(`journalctl -u perceptronics-cockpit -f`).

## What is verified, and what is not (2026-09-28)

- **Verified in CI, every change:** the detector against ray-cast ground truth (sizes,
  axes, order, reach, areas, finger room, holes, tilted views; property tests); the pick
  server protocol over a real socket, including concurrent requests; the URCap's script
  and settings under a JDK, cross-checked against the Python; the bundle loading and the
  node's own URScript running a two-part pick on PolyScope 5.24.0, 5.25.2 and 5.26.1.
- **Not yet seen:** the 0.5.0 screens on a real pendant (they render in a harness —
  `urcap/perceptronic-ps5/screens/`); the node driving the Hand-E on the UR3e; the
  camera computer on real Pi-class hardware (the installer and service are tested as
  files, not on a board). Those are the first things to do on the cell.
