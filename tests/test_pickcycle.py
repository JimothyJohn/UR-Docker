"""The pick cycle's heuristics on synthetic data, and a dry-run survey through a live cockpit."""

from __future__ import annotations

import json
import math
import threading
import time
from http.server import ThreadingHTTPServer

import pytest

from perceptronics.config import PerceptionConfig
from perceptronics.frame import Frame
from perceptronics.pickcycle import (
    Block,
    Cockpit,
    PickCycle,
    grasp_rotation,
    grasp_yaw_deg,
    plane_axes_xy,
    reject_off_surface,
    tip_pose,
    top_face,
    white_blobs,
)
from perceptronics.rgbd import DepthImage, Intrinsics, RgbdFrame
from perceptronics.robotlink import RobotLink
from perceptronics.webapp import ViewerApp, ViewerHandler
from urctl.config import RobotConfig
from urctl.pose import Transform

W, H = 160, 120
K = {"fx": 200.0, "fy": 200.0, "ppx": W / 2, "ppy": H / 2}


def scene(blocks, *, floor_z=0.40, height=0.04):
    """A beige floor at ``floor_z`` with white rectangles (x0, y0, x1, y1 pixels) ``height`` closer."""
    rgb = bytearray(b"\x80\x70\x50" * (W * H))
    depth = bytearray(W * H * 2)
    for y in range(H):
        for x in range(W):
            z = floor_z
            for x0, y0, x1, y1 in blocks:
                if x0 <= x < x1 and y0 <= y < y1:
                    z = floor_z - height
                    i = (y * W + x) * 3
                    rgb[i : i + 3] = b"\xf0\xf0\xf2"
            d = int(round(z / 0.001))
            depth[2 * (y * W + x)] = d & 0xFF
            depth[2 * (y * W + x) + 1] = d >> 8
    return bytes(rgb), bytes(depth)


def test_white_blobs_and_top_face_on_a_synthetic_block():
    rgb, depth = scene([(40, 30, 70, 54), (100, 70, 126, 92)])
    blobs = white_blobs(W, H, 3, rgb, step=2, min_px=50)
    assert len(blobs) == 2 and blobs[0]["px"] > blobs[1]["px"]
    assert abs(blobs[0]["cx"] - 55) <= 3 and abs(blobs[0]["cy"] - 42) <= 3
    tf = top_face(W, H, 3, rgb, depth, 0.001, K, blobs[0]["bbox"])
    assert tf is not None and tf["n"] > 500
    assert tf["centre"][2] == pytest.approx(0.36, abs=0.002)
    assert abs(abs(tf["normal"][2]) - 1.0) < 0.05  # a face square to the camera


def test_plane_axes_and_grasp_yaw():
    pts = [
        (
            0.05 * math.cos(0.3) * u - 0.02 * math.sin(0.3) * v,
            0.05 * math.sin(0.3) * u + 0.02 * math.cos(0.3) * v,
            0.0,
        )
        for u in (-1, -0.5, 0, 0.5, 1)
        for v in (-1, 0, 1)
    ]
    ax = plane_axes_xy(pts)
    assert ax["theta"] == pytest.approx(0.3, abs=0.02)
    assert ax["major_m"] > ax["minor_m"] > 0
    # The invariant that matters: after tip_pose applies the yaw, the finger axis
    # heads along the minor axis (either sense) — for a tool pointing down, tilted,
    # and for either finger axis.
    down = [0.3, 0.2, 0.1, math.pi, 0.0, 0.0]
    tilted = [
        0.3,
        0.2,
        0.1,
        *Transform.from_pose(down).compose(Transform.from_pose([0, 0, 0, 0.2, -0.1, 0.4])).to_pose()[3:],
    ]
    for flange in (down, tilted):
        for axis, vec in (("x", (1.0, 0.0, 0.0)), ("y", (0.0, 1.0, 0.0))):
            for minor_deg in (0, 30, 75, 120, 170, -50):
                yaw = grasp_yaw_deg(flange, math.radians(minor_deg), axis)
                assert -90 <= yaw <= 90
                pose = tip_pose([0.3, 0.2, -0.1], flange, 0.163, yaw)
                fa = Transform.from_pose(pose).rotate(vec)
                heading = math.degrees(math.atan2(fa[1], fa[0]))
                err = (heading - minor_deg + 90.0) % 180.0 - 90.0
                assert abs(err) < 3.0, (axis, minor_deg, yaw, heading)


def test_tip_pose_places_the_fingertips_not_the_flange():
    tip = [0.30, 0.20, -0.25]
    down = [0.0, 0.0, 0.0, math.pi, 0.0, 0.0]
    pose = tip_pose(tip, down, 0.163)
    assert pose[:3] == pytest.approx([0.30, 0.20, -0.25 + 0.163], abs=1e-9)
    tilted = [
        0.0,
        0.0,
        0.0,
        *Transform.from_pose([0, 0, 0, math.pi, 0, 0])
        .compose(Transform.from_pose([0, 0, 0, 0.2, 0, 0]))
        .to_pose()[3:],
    ]
    pose = tip_pose(tip, tilted, 0.163, yaw_deg=45)
    zax = Transform.from_pose(pose).rotate((0, 0, 1))
    back = [pose[i] + zax[i] * 0.163 for i in range(3)]
    assert back == pytest.approx(tip, abs=1e-9)


class SceneCamera:
    """An RgbdCamera whose frame is the synthetic block scene."""

    def __init__(self):
        self.rgb, self.depth = scene([(40, 30, 70, 54)])
        self._open = False

    def open(self):
        self._open = True

    def close(self):
        self._open = False

    def read(self):
        time.sleep(0.02)
        return RgbdFrame(
            color=Frame(width=W, height=H, data=self.rgb, channels=3),
            depth=DepthImage(width=W, height=H, data=self.depth, scale_m=0.001),
            intrinsics=Intrinsics(width=W, height=H, fx=K["fx"], fy=K["fy"], ppx=K["ppx"], ppy=K["ppy"]),
            timestamp_ms=0.0,
            frame_number=1,
            aligned=True,
            extra={"serial": "SCENE"},
        )

    def describe(self):
        return {"kind": "scene", "open": self._open, "device": {"serial": "SCENE"}, "intrinsics": {}}


@pytest.fixture
def cockpit():
    app = ViewerApp(
        SceneCamera(),
        config=PerceptionConfig(),
        robot=RobotLink(RobotConfig(host="fake-ur.invalid"), dry_run=True),
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield Cockpit(f"http://127.0.0.1:{srv.server_address[1]}")
    srv.shutdown()
    srv.server_close()
    app.stop()


def test_dry_run_survey_and_cycle_through_a_cockpit(cockpit):
    cycle = PickCycle(cockpit, dry_run=True, say_fn=lambda ev: None)
    blocks = cycle.survey()
    assert len(blocks) == 1
    b = blocks[0]
    assert b.n > 500 and 0.02 < b.major_m < 0.07 and 0.01 < b.minor_m < 0.06
    # the dry-run link reports a stand-in flange pose (tool down, 0.5 m out and up); the block is below it
    assert b.centre_base[2] < 0.5
    out = cycle.run(drop=True)
    assert out["ok"] and [r["stage"] for r in out["results"]] == ["replaced", "dropped"]
    assert any("one program" in ev["text"] for ev in cycle.log) and cycle.log[-1]["text"] == "Done"
    assert json.dumps(out, default=str)


def test_off_surface_candidates_are_dropped():
    mk = lambda i, z: Block(i, [0.1 * i, 0.2, z], 0.0, 0.045, 0.028, (0, 0), 500)  # noqa: E731
    kept = reject_off_surface([mk(0, -0.27), mk(1, -0.20), mk(2, -0.275), mk(3, -0.268)])
    assert [b.centre_base[2] for b in kept] == [-0.27, -0.275, -0.268] and [b.index for b in kept] == [
        0,
        1,
        2,
    ]
    two = [mk(0, -0.27), mk(1, -0.10)]
    assert reject_off_surface(two) == two  # too few to vote


def test_cli_parser_carries_every_option_the_runner_reads():
    """Every attribute run_pick_cycle reads must exist on the parsed namespace —
    a missing add_argument crashed the command on start once (2026-09-25)."""
    import argparse
    import inspect
    import re

    from perceptronics.pickcycle import add_pick_cycle_args, run_pick_cycle

    ap = argparse.ArgumentParser()
    add_pick_cycle_args(ap)
    ns = ap.parse_args(["--dry-run"])
    used = set(re.findall(r"args\.([a-z_]+)", inspect.getsource(run_pick_cycle)))
    missing = sorted(a for a in used if not hasattr(ns, a))
    assert not missing, missing


# -- the flat-table grasp (pick-cycle) --------------------------------------------------

TILTED = [-0.19, 0.19, 0.21, -2.583, 0.568, 0.0]  # the 09-27 survey pose: tool 28.5 deg off vertical


@pytest.mark.parametrize("top", [[-0.221, 0.381, -0.268], [0.3, 0.0, -0.1], [0.0, -0.35, -0.27]])
@pytest.mark.parametrize("lean", [0.0, 12.0, 24.0])
def test_grasp_rotation_points_down_and_leans_outward(top, lean):
    T = Transform.from_pose(grasp_rotation(TILTED, top, lean))
    x, y, z = (T.rotate(a) for a in ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    assert math.degrees(math.acos(-z[2])) == pytest.approx(lean, abs=1e-6)
    for a, b in ((x, y), (y, z), (x, z)):
        assert sum(p * q for p, q in zip(a, b, strict=True)) == pytest.approx(0.0, abs=1e-9)
    if lean:  # the fingertips tip away from the column: z has a positive radial part
        r = math.hypot(top[0], top[1])
        assert (z[0] * top[0] + z[1] * top[1]) / r > 0
    # the heading stays the flange's: X projected, not spun
    x0 = Transform.from_pose(TILTED).rotate((1, 0, 0))
    assert sum(p * q for p, q in zip(x, x0, strict=True)) > 0.8
