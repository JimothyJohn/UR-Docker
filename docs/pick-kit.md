# The RealSense pick kit for UR e-Series (PolyScope 5)

A wrist-mounted RealSense D435, a small camera computer beside the robot, and a URCap. The
operator tells the robot the part's size and where to look; the robot finds every part of
that size, numbers them in the order it will pick them, and picks them one by one —
approaching with the fingertips 25 mm over the part's top, fingers fully open, closing
across its short side.

| Piece | Where | What it is |
| ----- | ----- | ---------- |
| Bill of materials | [`hardware/BOM.md`](../hardware/BOM.md) | Every part with links, the price seen 2026-09-28 and its history (kit ≈ $1,420 per cell with a RevPi Connect 5; ≈ $1,112 with a CompuLab IOT-GATE-RPI5) |
| Camera bracket | [`hardware/d435-tool-bracket/`](../hardware/d435-tool-bracket/) | Parametric CadQuery source; print-ready STL and STEP for e-Series (ISO 9409-1-50-4-M6) and UR20 flanges in `out/`; the spec, fasteners and the nominal camera pose in its README |
| Camera computer | [`deploy/pi/`](../deploy/pi/README.md) | A Raspberry-Pi-class arm64 box on minimal Debian: one command installs librealsense, the cockpit as a hardened systemd service (:7621 HTTP, :7622 pick server) and a firewall |
| URCap | [`urcap/perceptronic-ps5/`](../urcap/perceptronic-ps5/README.md) | `urcap/dist/perceptronic-ps5-0.6.0.urcap`: the **Perceptronic Pick** program node and the **Perceptronic** installation node |
| URCap for PolyScope X | [`urcap/README.md`](../urcap/README.md) §5 | `urcap/dist/perceptronic-0.3.0.urcapx`: the same **Perceptronic Pick** node (and **After picture N**) for a PolyScope X robot; the pick areas and reach on the **Perceptronic** application node; one cockpit serves both robots |
| Self-deploy | [`.claude/skills/deploy-pick-pc/`](../.claude/skills/deploy-pick-pc/SKILL.md) | Claude Code deploys, checks, updates or rolls back the camera computer: "deploy the pick PC at 192.168.3.10" |
| Tested PolyScope | [`.github/workflows/urcap5-matrix.yml`](../.github/workflows/urcap5-matrix.yml) | Every change: the URCap loads and the node's own URScript picks on PolyScope 5.24, 5.25 and 5.26 (URSim) |

**See it first, on a desktop:** `uv run python urcap/preview5.py` opens the pendant's screens
in a window, fed by a simulated camera computer (or `--cockpit http://<ip>:7621`, a real one).

## Setting up a cell

1. **Mount the camera.** Print `d435_tool_bracket_eseries.stl` (the BOM names the
   material), fit it between the flange and the gripper with the tool's bolts, the D435
   on its 1/4-20 and M3 screws, the cable along the arm (strain relief at the wrist).
2. **Deploy the camera computer.** Flash Raspberry Pi OS Lite 64-bit (or Debian arm64),
   give it an address on the robot's network, plug the D435 into a **USB 3** port, then
   from a laptop with this repo — or ask Claude Code to "deploy the pick PC":

       scripts/deploy-pi.sh pi@192.168.3.10 --cell ur3 --robot-host 192.168.3.3

   It ends with `perceptronics doctor`'s verdict: camera, robot ports, hand-eye, tool length.
3. **Install the URCap** from a USB stick: Settings → System → URCaps → **+** →
   `perceptronic-ps5-0.6.0.urcap` → Restart.
   Or leave it to the stick: `scripts/urcap5-usb.sh` writes a magic file that installs it by
   itself when **Settings → Security → General → Run magic files** is on
   (`urcap/perceptronic-ps5/README.md`, *Or let the stick install it*).
4. **Installation → URCaps → Perceptronic**: type the camera computer's address
   (`http://192.168.3.10:7621`); the live picture appears.
5. **Calibrate the camera to the flange** once (`perceptronics calibrate` on the camera
   computer: the arm orbits a part and solves where the camera sits; `docs/realsense.md`
   §Hand-eye without a mark). Re-run it if the bracket is ever re-mounted.
6. **Pick areas + reach** (same Installation node, second tab): for each place parts are
   picked from, **New area** and touch the table with the fingertips at its corner, along
   one edge and on the far side. Check the reach ring: parts are picked only between the
   base's outer radius + 150 mm and the rated reach − 150 mm (both margins editable).

## Programming a pick

**Program tab → URCaps → Perceptronic Pick**, inside a loop:

1. **Add picture point here** — as many as needed; each looks at a pick area (tap its
   second line to choose) or the live table. The arm visits them in turn.
2. Choose the **pick order**; the picture renumbers the parts at once.
3. **Options** — the part's length × width × height, the approach (25 mm), the grip
   depth, the gripper (Robotiq Hand-E driven by the node, a digital output, or your own
   nodes), speed, and whether each picture point gets its own routine.
4. Put what happens to the part — the place — **inside** the node (or inside each
   **After picture N**). `rs_pick_found` and `rs_pick_loc` are there for your own logic.
5. **Check approach** opens PolyScope's move screen over part #1: hold Move to see the
   fingers arrive open, 25 mm over it.

A run: the picture (or, when the last picture already showed more parts, straight to the
next one), a close look to re-measure it, the approach, the grip, the lift, then your
routine. Anything that stops a run says why in a popup and on the camera computer's log
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
