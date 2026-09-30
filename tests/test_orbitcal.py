"""The orbit hand-eye calibration end to end on a fake cell: a fake camera renders a
block from wherever the fake arm's flange is (through a TRUE hand-eye the routine
never sees), a fake cockpit runs a real CalibrationSession from a WRONG seed, and
the routine must recover the truth — then the fault campaign around it."""

from __future__ import annotations

import argparse
import math
import re

import pytest

from perceptronics.calibrate import CalibrationSession
from perceptronics.handeye import HandEye
from perceptronics.orbitcal import (
    OrbitCalibration,
    add_calibrate_args,
    centred_pose,
    orbit_pose,
    plan_orbit,
    predict_pixel,
    run_calibrate,
)
from perceptronics.pickcycle import CockpitError
from tests.test_urctl import FakeController
from urctl.config import RobotConfig
from urctl.pose import Transform, matrix_to_rotvec, pose_inv, pose_trans
from urctl.robot import Robot

W, H = 160, 120
K = {"fx": 200.0, "fy": 200.0, "ppx": W / 2, "ppy": H / 2}
TRUE_FC = [0.012, 0.071, -0.004, 0.02, -0.015, 3.10]  # camera 70 mm off the flange, looking along the tool
SEED_NEAR = [0.032, 0.086, -0.014, 0.05, 0.03, 3.08]  # ~25 mm and ~3° off
SEED_FAR = [0.075, 0.115, 0.020, 0.09, 0.06, 3.02]  # ~70 mm and ~6° off
MARK = [0.35, 0.10, 0.0]  # a block's top-face centre on the table, base frame
HALF = (0.025, 0.015)  # 50 x 30 mm block
DOWN = [0.0, math.pi, 0.0]  # tool pointing down


def render(flange, fc_pose, mark, *, block=True):
    """The colour + depth image a camera at flange∘fc sees: a beige floor 40 mm
    under the block's top face and the white top face itself."""
    t_bc = Transform.from_pose(flange).compose(Transform.from_pose(fc_pose))
    o = t_bc.translation
    r = t_bc.rotation
    rgb = bytearray(b"\x80\x70\x50" * (W * H))
    depth = bytearray(W * H * 2)
    z_top, z_floor = mark[2], mark[2] - 0.04
    for v in range(H):
        dy = (v - K["ppy"]) / K["fy"]
        for u in range(W):
            dx = (u - K["ppx"]) / K["fx"]
            d = (
                r[0][0] * dx + r[0][1] * dy + r[0][2],
                r[1][0] * dx + r[1][1] * dy + r[1][2],
                r[2][0] * dx + r[2][1] * dy + r[2][2],
            )
            if abs(d[2]) < 1e-6:
                continue
            white = False
            t = (z_top - o[2]) / d[2]
            if block and t > 0:
                px, py = o[0] + t * d[0], o[1] + t * d[1]
                white = abs(px - mark[0]) <= HALF[0] and abs(py - mark[1]) <= HALF[1]
            if not white:
                t = (z_floor - o[2]) / d[2]
                if t <= 0:
                    continue
            mm = min(65535, int(round(t * 1000)))
            i = v * W + u
            depth[2 * i] = mm & 0xFF
            depth[2 * i + 1] = mm >> 8
            if white:
                rgb[3 * i : 3 * i + 3] = b"\xf0\xf0\xf2"
    return bytes(rgb), bytes(depth)


class MovingController(FakeController):
    """The fake controller, with a flange that actually goes where movel says, an
    optional protective stop on chosen moves, and an unlock that clears it."""

    def __init__(self, **kw):
        super().__init__(**kw)
        self.model = "UR5"  # 0.85 m reach: every planned view of the test cell is reachable
        self.pstop_on: set[int] = set()
        self.moves = 0

    def _dashboard(self, host, port, payload, timeout=10.0):
        if "unlock protective stop" in payload.decode():
            self.safety_mode = "NORMAL"
        return super()._dashboard(host, port, payload, timeout)

    def _primary_collect(self, host, port, payload, **kw):
        body = payload.decode()
        m = re.search(r"movel\(p\[([^\]]+)\]", body)
        if m and "urctl/flange" not in body:
            self.moves += 1
            if self.moves in self.pstop_on:
                self.primary_sends.append(body)
                self.safety_mode = "PROTECTIVE_STOP"
                return b""
            self.tcp_pose = [float(v) for v in m.group(1).split(",")]
            if "set_tcp(p[0.0,0.0,0.0" in body.replace(" ", "") or "set_tcp(p[0,0,0" in body.replace(" ", ""):
                self.tcp_offset = [0.0] * 6
        return super()._primary_collect(host, port, payload, **kw)


class FakeCockpit:
    """Vision + the calibration session, as the real cockpit serves them."""

    base = "fake://cockpit"

    def __init__(self, ctl, seed, *, bad_views=(), block=True):
        self.ctl = ctl
        self.seed = seed
        self.block = block
        self.bad_views = set(bad_views)
        self.session = CalibrationSession(seed=HandEye.from_pose(seed, source="bracket-nominal:test"))
        self.posts: list[tuple[str, dict]] = []
        self.applied = None
        self.frames = 0
        self.last = None

    def _flange(self):
        return pose_trans(self.ctl.tcp_pose, pose_inv(self.ctl.tcp_offset))

    def frame(self):
        self.frames += 1
        rgb, depth = render(self._flange(), TRUE_FC, MARK, block=self.block)
        self.last = (rgb, depth)
        hdr = {"intrinsics": K, "depth_scale_m": 0.001, "width": W, "height": H}
        return hdr, W, H, 3, rgb, depth

    def get(self, path):
        if path == "/api/robot":
            return {
                "ok": True,
                "robot": {
                    "host": "fake-ur.invalid",
                    "handeye": {"flange_to_color_pose": self.seed, "source": "bracket-nominal:test"},
                },
            }
        raise AssertionError(path)

    def post(self, path, body=None):
        body = body or {}
        self.posts.append((path, body))
        if path == "/api/cal/reset":
            return {**self.session.reset(), "ok": True}
        if path == "/api/cal/view":
            x, y = int(body["x"]), int(body["y"])
            _, depth = self.last
            i = y * W + x
            z = (depth[2 * i] | (depth[2 * i + 1] << 8)) * 0.001
            if z <= 0:
                return {"ok": False, "error": "no valid depth around that pixel"}
            if len(self.session.views) in self.bad_views:
                z += 0.03  # a bad click: 30 mm of depth error
            pc = [(x - K["ppx"]) * z / K["fx"], (y - K["ppy"]) * z / K["fy"], z]
            out = self.session.add_view(self._flange(), pc, pixel=(x, y))
            return {**out, "ok": True}
        if path == "/api/cal/solve":
            try:
                return dict(self.session.solve())
            except Exception as exc:
                return {"ok": False, "error": str(exc)}
        if path == "/api/cal/remove":
            return {**self.session.remove_view(int(body["index"])), "ok": True}
        if path == "/api/cal/apply":
            if not self.session.result:
                return {"ok": False, "error": "not solved"}
            self.applied = self.session.result["flange_to_depth_pose"]
            return {"ok": True, "saved": "captures/calibration/handeye_test.json"}
        if path in ("/api/robot/stop", "/api/robot/bring_up"):
            return {"ok": True}
        return {"ok": False, "error": f"unexpected {path}"}


def start_pose(fc_pose=TRUE_FC, range_m=0.30):
    return centred_pose(Transform.from_pose(fc_pose), MARK, [0.0, 0.0, 0.0, *DOWN], range_m)


@pytest.fixture
def rig(monkeypatch):
    def make(seed=SEED_NEAR, **cockpit_kw):
        ctl = MovingController().install(monkeypatch)
        ctl.remote = True
        ctl.tcp_pose = start_pose()
        ctl.tcp_offset = [0.0] * 6
        robot = Robot(RobotConfig(host="fake-ur.invalid"))
        cockpit = FakeCockpit(ctl, seed, **cockpit_kw)
        events: list[str] = []
        cal = OrbitCalibration(
            cockpit, robot=robot, min_blob_px=100, settle_s=0.0, say_fn=lambda ev: events.append(ev["text"])
        )
        return ctl, cockpit, cal, events

    return make


def _delta(pose_a, pose_b):
    """(translation mm, rotation deg) between two poses."""
    a, b = Transform.from_pose(pose_a), Transform.from_pose(pose_b)
    rel = a.inverse().compose(b)
    ang = math.degrees(math.sqrt(sum(c * c for c in matrix_to_rotvec(rel.rotation))))
    return math.dist(a.translation, b.translation) * 1000.0, ang


# -- geometry ---------------------------------------------------------------------------------


def test_centred_pose_puts_the_mark_on_the_optical_axis():
    track = Transform.from_pose(TRUE_FC)
    pose = start_pose(range_m=0.30)
    u, v, p_cam = predict_pixel(track, pose, MARK, K)
    assert abs(u - K["ppx"]) < 1e-6 and abs(v - K["ppy"]) < 1e-6
    assert abs(p_cam[2] - 0.30) < 1e-9


def test_orbit_pose_keeps_the_range_and_turns_by_the_angle():
    pose = start_pose()
    turned = orbit_pose(pose, MARK, (1, 0, 0), 15)
    assert abs(math.dist(turned[:3], MARK) - math.dist(pose[:3], MARK)) < 1e-9
    rel = Transform.from_pose(pose).inverse().compose(Transform.from_pose(turned))
    assert abs(math.degrees(math.sqrt(sum(c * c for c in matrix_to_rotvec(rel.rotation)))) - 15) < 1e-6
    # the mark stays on the optical axis
    u, v, _ = predict_pixel(Transform.from_pose(TRUE_FC), turned, MARK, K)
    assert abs(u - K["ppx"]) < 1e-6 and abs(v - K["ppy"]) < 1e-6


def test_plan_has_thirteen_views_per_range_and_the_ranges_asked_for():
    plan = plan_orbit(Transform.from_pose(TRUE_FC), MARK, start_pose(), ranges_m=(0.25, 0.35))
    assert len(plan) == 26 and {p["range_m"] for p in plan} == {0.25, 0.35}
    assert [p["name"] for p in plan[:3]] == ["centre@0.25", "X+@0.25", "X-@0.25"]


# -- the solve ----------------------------------------------------------------------------------


def test_recovers_the_hand_eye_from_a_wrong_seed(rig):
    ctl, cockpit, cal, events = rig()
    seed_err = _delta(SEED_NEAR, TRUE_FC)
    assert seed_err[0] > 20  # the seed really is off
    out = cal.run()
    assert out["ok"] and out["kept"] >= 20 and out["trimmed"] == 0
    got = cockpit.session.result["flange_to_color_pose"]
    mm, deg = _delta(got, TRUE_FC)
    assert mm < 2.0 and deg < 0.8, (mm, deg)  # roll about the mark's ray is the weak axis
    assert out["rms_mm"] < 2.0 and out["env_line"].startswith("PERCEPTRONICS_T_FLANGE_CAMERA=")
    assert cockpit.applied is None and any("Not applied" in e for e in events)
    # every move was the flange (tcp override) at the calibration speed, and the arm came home
    moves = [b for b in ctl.primary_sends if "movel(" in b and "urctl/flange" not in b]
    assert moves and all("set_tcp(p[0" in b.replace(" ", "") for b in moves)
    mm, deg = _delta(ctl.tcp_pose, start_pose())
    assert mm < 0.1 and deg < 0.1  # back at the start pose (rotvec representation may differ)


def test_apply_goes_through_the_cockpit(rig):
    ctl, cockpit, cal, events = rig()
    out = cal.run(apply=True)
    assert out["applied"] and cockpit.applied is not None and out["saved"].endswith(".json")
    assert _delta(cockpit.session.result["flange_to_color_pose"], TRUE_FC)[0] < 2.0


def test_a_seed_centimetres_off_still_locks_on_to_the_block(rig):
    ctl, cockpit, cal, events = rig(seed=SEED_FAR)
    assert _delta(SEED_FAR, TRUE_FC)[0] > 60
    out = cal.run()
    assert out["ok"] and out["kept"] >= 12
    mm, deg = _delta(cockpit.session.result["flange_to_color_pose"], TRUE_FC)
    assert mm < 3.0 and deg < 0.8, (mm, deg)


def test_a_bad_click_is_trimmed_out(rig):
    ctl, cockpit, cal, events = rig(bad_views={5})
    out = cal.run()
    assert out["ok"] and out["trimmed"] == 1 and out["rms_mm"] < 2.0
    assert any("Trim: dropped view 6" in e for e in events)
    assert ("/api/cal/remove", {"index": 5}) in cockpit.posts


# -- faults -------------------------------------------------------------------------------------


def test_local_control_refuses_before_any_motion(rig):
    ctl, cockpit, cal, events = rig()
    ctl.remote = False
    with pytest.raises(CockpitError, match="Local"):
        cal.run()
    assert not [b for b in ctl.primary_sends if "movel(" in b]


def test_no_block_in_view_is_an_error_before_any_motion(rig):
    ctl, cockpit, cal, events = rig(block=False)
    with pytest.raises(CockpitError, match="no white block"):
        cal.run()
    assert not [b for b in ctl.primary_sends if "movel(" in b]


def test_protective_stop_unlocks_and_moves_on(rig):
    ctl, cockpit, cal, events = rig()
    ctl.pstop_on = {3}
    out = cal.run()
    assert out["ok"]
    assert [s["status"] for s in out["steps"]].count("protective_stop") == 1
    assert any("unlock protective stop" in s for s in ctl.dashboard_sends)
    assert _delta(cockpit.session.result["flange_to_color_pose"], TRUE_FC)[0] < 2.0


def test_a_refused_move_is_skipped_and_the_run_continues(rig, monkeypatch):
    ctl, cockpit, cal, events = rig()
    real = cal._move
    calls = {"n": 0}

    def flaky(pose):
        calls["n"] += 1
        if calls["n"] == 2:
            return {"ok": False, "error": "velocity", "safety": {"violations": ["velocity"]}}
        return real(pose)

    monkeypatch.setattr(cal, "_move", flaky)
    out = cal.run()
    assert out["ok"] and [s["status"] for s in out["steps"]].count("refused") == 1
    assert any("refused" in e for e in events)


def test_views_beyond_the_arm_reach_are_skipped_not_sent(rig):
    ctl, cockpit, cal, events = rig()
    ctl.model = "UR3"  # the Dashboard reports a UR3e: the envelope sizes itself to 0.5 m on first use
    out = cal.run()
    beyond = [s for s in out["steps"] if s["status"] == "skipped" and "reach" in s["why"]]
    assert beyond and out["ok"]
    assert all("0.45 m" in s["why"] for s in beyond)
    # exactly one program per view that was not skipped, plus the return to the start pose
    sent = [b for b in ctl.primary_sends if "movel(" in b and "urctl/flange" not in b]
    assert len(sent) == sum(1 for s in out["steps"] if s["status"] != "skipped") + 1


def test_dry_run_plans_and_moves_nothing(rig):
    ctl, cockpit, cal, events = rig()
    cal.dry_run = True
    out = cal.run()
    assert out["dry_run"] and out["planned"] == 39
    assert {s["status"] for s in out["steps"]} <= {"planned", "skipped"}
    assert not [b for b in ctl.primary_sends if "movel(" in b and "urctl/flange" not in b]
    assert not [p for p, _ in cockpit.posts if p.startswith("/api/cal")]


def test_interrupt_reaches_the_bail_out_and_exits_130(monkeypatch):
    from perceptronics import orbitcal as oc

    calls = []
    monkeypatch.setattr(
        oc.OrbitCalibration, "run", lambda self, **kw: (_ for _ in ()).throw(KeyboardInterrupt())
    )
    monkeypatch.setattr(oc.OrbitCalibration, "bail_out", lambda self: calls.append("bail"))
    ap = argparse.ArgumentParser()
    add_calibrate_args(ap)
    assert run_calibrate(ap.parse_args(["--robot-host", "fake-ur.invalid"])) == 130 and calls == ["bail"]


def test_parser_and_runner_agree_on_every_option(monkeypatch, capsys):
    """Every attribute the runner reads exists on the parsed namespace (the pick-cycle
    shipped once with an option the runner read and the parser lacked)."""
    from perceptronics import orbitcal as oc

    monkeypatch.setattr(
        oc.OrbitCalibration,
        "run",
        lambda self, **kw: {"ok": True, "env_line": "PERCEPTRONICS_T_FLANGE_CAMERA=x"},
    )
    ap = argparse.ArgumentParser()
    add_calibrate_args(ap)
    args = ap.parse_args(["--via-cockpit", "--range-m", "0.25", "0.35", "--apply", "--no-reset", "--json"])
    assert run_calibrate(args) == 0
    assert '"ok": true' in capsys.readouterr().out
