#!/usr/bin/env python3
"""Preview the PolyScope 5 URCap's screens in action, on a desktop — no robot, no pendant.

Opens the Perceptronic Pick node's and the Installation node's real Swing screens in a
1280 x 800 window (the pendant's size) and feeds them from a camera computer:

* by default a **simulated** one started here: the cockpit's HTTP API over a ray-cast box
  scene (seven parts of the node's default size, one too long, one too small) seen from a
  fixed flange pose, its picture stamped NO CAMERA CONNECTED. The parts, their pick
  numbers, sizes and reasons are the real detector's answers;
* or, with ``--cockpit http://192.168.3.10:7621``, the real one (its camera, its robot pose).

Everything you can tap works: add / select / remove picture points, the eight pick-order
tiles (the numbers change on the picture), the Options view with its drawings, the
Installation's pick areas and reach map. What only a robot can do is stood in for — a
picture point is added without moving an arm, a pick area's three touches are a sample
rectangle on the demo table, typed values come from a dialog instead of PolyScope's keypad.

    uv run python urcap/preview5.py                   # simulated camera computer
    uv run python urcap/preview5.py --shuffle 6       # ... that re-scatters the parts every 6 s
    uv run python urcap/preview5.py --cockpit http://192.168.3.10:7621
    uv run python urcap/preview5.py --snapshot out.png   # render once and exit (no display)

Needs a JDK (``javac``).
"""

from __future__ import annotations

import argparse
import math
import random
import shutil
import subprocess
import sys
import tempfile
import threading
import time
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO))

from tests.test_urcap5 import JAVA, PURE_JAVA  # noqa: E402

SCREENS = ("PickScreen.java", "LiveView.java", "LocationsScreen.java")
PREVIEW = Path(__file__).resolve().parent / "preview5" / "Preview.java"
FLANGE = [0.29, -0.01, 0.10, math.pi, 0.0, 0.0]  # the picture point: tool down, 0.37 m over the table
TABLE_Z = -0.27  # the UR3e cell: parts ~0.27 m below the base
W, H, FX = 848, 480, 610.0  # the D435's colour stream


def scene_boxes(rng: random.Random | None = None):
    from perceptronics.synthscene import Box

    if rng is None:  # two rows of three, a spare, and three that are not to be picked
        boxes = [
            Box(0.25 + 0.065 * r, -0.08 + 0.07 * c, 0.05, 0.03, 0.03, 0.12 * (c - 1) - 0.06 * r)
            for r in range(2)
            for c in range(3)
        ]
        return boxes + [
            Box(0.235, 0.12, 0.05, 0.03, 0.03, 1.2),
            Box(0.385, -0.085, 0.068, 0.035, 0.03, 0.4),  # too long
            Box(0.385, 0.045, 0.025, 0.025, 0.02, 0.0),  # too short
            Box(0.40, -0.02, 0.05, 0.03, 0.03, 0.3),  # beyond the UR3e's reach ring
        ]
    boxes: list = []
    while len(boxes) < 7:
        x, y = rng.uniform(0.22, 0.36), rng.uniform(-0.11, 0.09)
        if all(math.dist((x, y), (b.x, b.y)) > 0.07 for b in boxes):
            boxes.append(Box(x, y, 0.05, 0.03, 0.03, rng.uniform(-math.pi / 2, math.pi / 2)))
    return boxes


def simulated_cockpit(port: int, shuffle_s: float):
    """The cockpit's HTTP API over the box scene, with a stand-in robot link (a fixed flange
    pose, the camera at the flange) so reach and pick areas are checked as on a robot."""
    from http.server import ThreadingHTTPServer

    from perceptronics.config import PerceptionConfig
    from perceptronics.synthscene import BoxSceneCamera
    from perceptronics.webapp import ViewerApp, ViewerHandler
    from urctl.pose import Transform

    cam = BoxSceneCamera(scene_boxes(), Transform.from_pose(FLANGE), w=W, h=H, fx=FX, table_z=TABLE_Z)

    class Eye:
        def as_dict(self):
            return {"flange_to_color_pose": [0.0] * 6}

    class Link:  # what the scene route reads of a robot link
        handeye = Eye()
        tip_m = 0.163

        def flange_pose(self):
            return {
                "ok": True,
                "flange": FLANGE,
                "tcp_offset": [0, 0, 0.163, 0, 0, 0],
                "tcp_offset_consistent": True,
            }

    app = ViewerApp(cam, config=PerceptionConfig())
    srv = ThreadingHTTPServer(("127.0.0.1", port), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 10
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.05)
    app.robot = Link()  # after the camera loop is up: it needs nothing of the robot

    if shuffle_s > 0:

        def shuffle():
            rng = random.Random(1)
            while True:
                time.sleep(shuffle_s)
                fresh = BoxSceneCamera(
                    scene_boxes(rng), Transform.from_pose(FLANGE), w=W, h=H, fx=FX, table_z=TABLE_Z
                )
                cam.depth, cam.rgb = fresh.depth, fresh.rgb

        threading.Thread(target=shuffle, daemon=True).start()
    return f"http://127.0.0.1:{srv.server_address[1]}", lambda: (srv.shutdown(), app.stop())


def build() -> Path:
    javac = shutil.which("javac")
    if not javac:
        raise SystemExit("the preview needs a JDK (javac): brew install openjdk, or apt install default-jdk")
    root = Path(tempfile.mkdtemp(prefix="urcap5-preview-"))
    pkg = root / "src" / "com" / "nickarmenta" / "perceptronic"
    pkg.mkdir(parents=True)
    for name in (*PURE_JAVA, *SCREENS):
        shutil.copy(JAVA / name, pkg / name)
    shutil.copy(PREVIEW, pkg / PREVIEW.name)
    subprocess.run(
        [javac, "--release", "8", "-Xlint:-options", "-encoding", "UTF-8", "-d", str(root / "c"),
         *map(str, pkg.glob("*.java"))],
        check=True,
    )  # fmt: skip
    return root / "c"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--cockpit", help="a running camera computer (default: a simulated one started here)")
    ap.add_argument(
        "--port", type=int, default=0, help="the simulated camera computer's port (default: any free)"
    )
    ap.add_argument("--shuffle", type=float, default=0.0, help="re-scatter the simulated parts every N s")
    ap.add_argument(
        "--snapshot", help="render the window once to this PNG and exit (works without a display)"
    )
    args = ap.parse_args(argv)

    stop = None
    base = args.cockpit
    if not base:
        print("rendering the simulated scene…", flush=True)
        base, stop = simulated_cockpit(args.port, args.shuffle)
        print(f"simulated camera computer on {base} (no camera connected: its picture says so)", flush=True)
    classes = build()
    cmd = ["java", "-cp", str(classes)]
    if args.snapshot:
        cmd.append("-Djava.awt.headless=true")
    cmd += ["com.nickarmenta.perceptronic.Preview", base]
    if args.snapshot:
        cmd += ["--snapshot", str(Path(args.snapshot).resolve())]
    try:
        return subprocess.run(cmd).returncode
    finally:
        if stop:
            stop()


if __name__ == "__main__":
    raise SystemExit(main())
