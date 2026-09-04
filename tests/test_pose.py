"""urctl.pose — UR-convention pose math (rotation vectors, compose, invert)."""

from __future__ import annotations

import math
import random

import pytest

from urctl.pose import (
    IDENTITY,
    Transform,
    matrix_to_rotvec,
    pose_inv,
    pose_trans,
    rotvec_to_matrix,
)


def _close(a, b, tol=1e-9):
    return all(abs(x - y) < tol for x, y in zip(a, b, strict=True))


def _flat(m):
    return [v for row in m for v in row]


def test_rotvec_matrix_round_trip_random():
    rng = random.Random(7)
    for _ in range(200):
        axis = [rng.uniform(-1, 1) for _ in range(3)]
        n = math.sqrt(sum(a * a for a in axis)) or 1.0
        theta = rng.uniform(-math.pi + 1e-3, math.pi - 1e-3)
        rv = [a / n * theta for a in axis]
        back = matrix_to_rotvec(rotvec_to_matrix(rv))
        assert _close(rv, back, 1e-7), (rv, back)


def test_rotvec_round_trip_near_pi():
    rng = random.Random(11)
    for _ in range(200):
        axis = [rng.uniform(-1, 1) for _ in range(3)]
        n = math.sqrt(sum(a * a for a in axis)) or 1.0
        theta = math.pi - rng.uniform(0, 2e-3)
        rv = [a / n * theta for a in axis]
        back = matrix_to_rotvec(rotvec_to_matrix(rv))
        assert _flat(rotvec_to_matrix(back)) == pytest.approx(_flat(rotvec_to_matrix(rv)), abs=1e-7)


def test_rotvec_edge_cases():
    assert rotvec_to_matrix((0, 0, 0)) == IDENTITY
    assert matrix_to_rotvec(IDENTITY) == (0.0, 0.0, 0.0)
    # 90° about z maps x → y
    m = rotvec_to_matrix((0, 0, math.pi / 2))
    assert _close(Transform(m).rotate((1, 0, 0)), (0, 1, 0), 1e-12)
    # π about x (the classic "tool pointing down" UR orientation) survives the round trip
    for rv in ((math.pi, 0, 0), (0, math.pi, 0), (0, 0, math.pi), (2.2214, 2.2214, 0)):
        back = matrix_to_rotvec(rotvec_to_matrix(rv))
        # at exactly π the sign of the axis is a convention; the rotation must match
        # (1e-8 ≈ the acos precision floor near π — 0.01 µm at a metre)
        assert _flat(rotvec_to_matrix(back)) == pytest.approx(_flat(rotvec_to_matrix(rv)), abs=1e-8)


def test_compose_inverse_and_apply():
    a = Transform.from_pose([0.1, 0.2, 0.3, 0.4, -0.5, 0.6])
    b = Transform.from_pose([-0.3, 0.05, 0.7, 1.0, 0.2, -0.4])
    p = (0.11, -0.22, 0.33)
    # (a·b)(p) == a(b(p))
    assert _close(a.compose(b).apply(p), a.apply(b.apply(p)), 1e-12)
    # a⁻¹(a(p)) == p, and a·a⁻¹ == identity
    assert _close(a.inverse().apply(a.apply(p)), p, 1e-12)
    ident = a.compose(a.inverse())
    assert _close(ident.translation, (0, 0, 0), 1e-12)
    assert _close([v for row in ident.rotation for v in row], [v for row in IDENTITY for v in row], 1e-12)
    # to_pose/from_pose round-trip
    assert _close(Transform.from_pose(a.to_pose()).apply(p), a.apply(p), 1e-9)


def test_pose_trans_and_pose_inv_match_urscript_semantics():
    # pose_trans(tcp, pose_inv(offset)) recovers the flange from the TCP: build
    # a flange, a tool offset, the resulting TCP, and undo it.
    flange = [0.4, -0.1, 0.5, 2.0, -1.1, 0.3]
    offset = [0.0, 0.0, 0.15, 0.0, 0.0, 0.5]  # 150 mm tool, twisted 0.5 rad
    tcp = pose_trans(flange, offset)
    back = pose_trans(tcp, pose_inv(offset))
    assert _close(back[:3], flange[:3], 1e-9)
    assert _flat(rotvec_to_matrix(back[3:])) == pytest.approx(_flat(rotvec_to_matrix(flange[3:])), abs=1e-9)


def test_from_axes_is_column_major():
    # child x = parent +Y, child y = parent −X, child z = parent +Z (the bracket's camera map)
    t = Transform.from_axes((0, 1, 0), (-1, 0, 0), (0, 0, 1), (0.075, -0.0175, 0.035))
    assert _close(t.apply((0, 0, 0.3)), (0.075, -0.0175, 0.335), 1e-12)  # on-axis point goes straight out +Z
    assert _close(t.apply((0.1, 0, 0)), (0.075, 0.0825, 0.035), 1e-12)  # camera +x is flange +Y
    assert _close(t.rotate((0, 1, 0)), (-1, 0, 0), 1e-12)


def test_bad_pose_length():
    with pytest.raises(ValueError):
        Transform.from_pose([1, 2, 3])
