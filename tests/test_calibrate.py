"""Touch-and-click hand-eye: recover a known transform from synthetic views."""

from __future__ import annotations

import json
import math
import random

import pytest

from perception.calibrate import CalibrationError, CalibrationSession, load_calibration_pose
from perception.handeye import BRACKET_NOMINAL_ESERIES, HandEye
from urctl.pose import Transform, rotvec_to_matrix


def _true_transform() -> Transform:
    """The bracket seed perturbed by 8 mm and ~2° — what a real print looks like."""
    seed = BRACKET_NOMINAL_ESERIES
    tweak = Transform(rotvec_to_matrix((0.02, -0.03, 0.015)), (0.005, -0.004, 0.003))
    return seed.compose(tweak)


def _views(true_fc: Transform, mark, n=6, noise_m=0.0, rng=None):
    """Flange poses looking at the mark from varied orientations; the camera point
    is the mark seen through the *true* transform (+ optional noise)."""
    rng = rng or random.Random(1)
    out = []
    for i in range(n):
        # a flange pose ~0.45 m above the mark, tilted and yawed differently each time
        # tool pointing down (Rx(pi)), then a different yaw about the tool axis and a
        # tilt each time, so the flange orientations are genuinely diverse
        rz = Transform(rotvec_to_matrix((0.0, 0.0, i * 0.7)))
        rx = Transform(rotvec_to_matrix((math.pi, 0.0, 0.0)))
        tilt = Transform(rotvec_to_matrix((0.25 * math.sin(i * 1.3), 0.15 * math.cos(i), 0.0)))
        rot = rz.compose(tilt).compose(rx).rotation
        offset = (0.03 * math.cos(i), 0.03 * math.sin(i), 0.40 + 0.05 * (i % 3))
        base_from_flange = Transform(rot, (mark[0] + offset[0], mark[1] + offset[1], mark[2] + offset[2]))
        p_flange = base_from_flange.inverse().apply(mark)
        p_cam = true_fc.inverse().apply(p_flange)
        assert p_cam[2] > 0.2, "generator bug: the camera must look at the mark"
        if noise_m:
            p_cam = tuple(v + rng.gauss(0, noise_m) for v in p_cam)
        out.append((base_from_flange.to_pose(), list(p_cam)))
    return out


def test_recovers_transform_with_touched_mark():
    true_fc = _true_transform()
    mark = [0.45, -0.10, 0.02]
    s = CalibrationSession(seed=HandEye())  # bracket nominal seed, identity extrinsics
    s.record_mark(mark)
    for fp, pc in _views(true_fc, mark, n=5):
        s.add_view(fp, pc, pixel=(1, 2), seq=3)
    res = s.solve()
    got = Transform.from_pose(res["flange_to_depth_pose"])
    assert all(abs(a - b) < 1e-6 for a, b in zip(got.translation, true_fc.translation, strict=True))
    for axis in ((1, 0, 0), (0, 1, 0), (0, 0, 1)):
        assert all(abs(a - b) < 1e-6 for a, b in zip(got.rotate(axis), true_fc.rotate(axis), strict=True))
    assert res["rms_m"] < 1e-7 and res["views"] == 5 and not res["warnings"]
    assert res["env_line"].startswith('PERCEPTION_T_FLANGE_CAMERA="[')
    assert 4 < max(abs(v) for v in res["delta_from_seed_mm"]) < 20  # it moved off the seed, sensibly
    assert s.handeye().calibrated and s.handeye().source.startswith("calibrated")


def test_recovers_transform_with_noise_and_extrinsics():
    true_fc = _true_transform()
    mark = [0.50, 0.05, 0.0]
    ext = {"rotation": [[1, 0, 0], [0, 1, 0], [0, 0, 1]], "translation": [0.015, 0.0, 0.0]}  # D435-ish
    s = CalibrationSession(seed=HandEye(), extrinsics=ext)
    s.record_mark(mark)
    for fp, pc in _views(true_fc, mark, n=8, noise_m=0.0015):
        s.add_view(fp, pc)
    res = s.solve()
    got_color = Transform.from_pose(res["flange_to_color_pose"])
    err = math.dist(got_color.translation, true_fc.translation)
    assert err < 0.005, err  # 1.5 mm click noise over 8 views → a few mm at worst
    # depth pose = colour pose composed with depth→colour
    got_depth = Transform.from_pose(res["flange_to_depth_pose"])
    expect = got_color.compose(Transform.from_pose([0.015, 0, 0, 0, 0, 0]))
    assert math.dist(got_depth.translation, expect.translation) < 1e-9


def test_solves_mark_jointly_without_a_touch():
    true_fc = _true_transform()
    mark = [0.40, 0.00, 0.05]
    s = CalibrationSession(seed=HandEye())
    for fp, pc in _views(true_fc, mark, n=6):
        s.add_view(fp, pc)
    res = s.solve()
    assert res["mark_solved"] and math.dist(res["mark_base"], mark) < 1e-5
    assert math.dist(Transform.from_pose(res["flange_to_color_pose"]).translation, true_fc.translation) < 1e-5


def test_too_few_or_degenerate_views():
    s = CalibrationSession()
    s.record_mark([0.4, 0, 0])
    with pytest.raises(CalibrationError, match="at least 3"):
        s.solve()
    s2 = CalibrationSession()
    for fp, pc in _views(_true_transform(), [0.4, 0, 0], n=3):
        s2.add_view(fp, pc)
    with pytest.raises(CalibrationError, match="at least 4"):
        s2.solve()
    # identical poses → no rotation diversity → warned (and translation unobservable)
    s3 = CalibrationSession()
    s3.record_mark([0.4, 0, 0])
    fp, pc = _views(_true_transform(), [0.4, 0, 0], n=1)[0]
    for _ in range(3):
        s3.add_view(fp, pc)
    res = s3.solve()
    assert any("poorly observed" in w for w in res["warnings"])


def test_bad_inputs_and_bookkeeping(tmp_path):
    s = CalibrationSession()
    with pytest.raises(CalibrationError):
        s.record_mark([float("nan"), 0, 0])
    with pytest.raises(CalibrationError, match="no depth"):
        s.add_view([0] * 6, [0, 0, 0])
    with pytest.raises(CalibrationError):
        s.add_view([0] * 5, [0, 0, 1])
    s.add_view([0, 0, 0, 0, 0, 0], [0, 0, 1])
    assert len(s.views) == 1 and s.as_dict()["min_views"] == 4
    s.remove_view(0)
    with pytest.raises(CalibrationError):
        s.remove_view(0)
    with pytest.raises(CalibrationError, match="solve first"):
        s.save(tmp_path / "x.json")
    s.reset()
    assert s.as_dict()["views"] == [] and s.as_dict()["mark_base"] is None


def test_save_and_load_roundtrip(tmp_path):
    true_fc = _true_transform()
    mark = [0.45, -0.10, 0.02]
    s = CalibrationSession()
    s.record_mark(mark)
    for fp, pc in _views(true_fc, mark, n=4):
        s.add_view(fp, pc)
    s.solve()
    p = s.save(tmp_path / "cal" / "handeye.json")
    body = json.loads(p.read_text())
    assert body["source"] == "touch-and-click" and len(body["session"]["views"]) == 4
    assert load_calibration_pose(p) == s.result["flange_to_depth_pose"]
