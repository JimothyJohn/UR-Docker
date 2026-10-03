"""The volume detector's contract (:mod:`perceptronics.volume`), against ray-cast ground truth
(:mod:`perceptronics.synthscene`): the right size and nothing else, the rectangle's axes, the
surface (fitted, taught, nudged), the reach annulus, the pick order the pendant numbers."""

from __future__ import annotations

import math
import random

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from perceptronics.partspec import PartSpec
from perceptronics.synthscene import Box, camera_looking_down, render_depth
from perceptronics.volume import (
    Reach,
    Surface,
    convex_hull,
    find_parts,
    min_area_rect,
    order_parts,
    parse_order,
)
from urctl.pose import Transform

W, H = 320, 180
K = {"fx": 230.0, "fy": 230.0, "ppx": W / 2, "ppy": H / 2}
TABLE = -0.27  # the UR3e cell: parts ~0.27 m below the base
SPEC = PartSpec.from_mm(60, 40, 30)
CAM = camera_looking_down(0.35, 0.0, 0.12)  # 0.39 m over the table


def scene(boxes, T=CAM, **kw):
    return render_depth(W, H, K, T, boxes, table_z=TABLE, **kw)


def detect(boxes, T=CAM, **kw):
    depth = scene(boxes, T, holes=kw.pop("holes", ()))
    return find_parts(W, H, depth, 0.001, K, T, **kw)


def ang_diff(a, b):
    """Difference between two axis directions (mod π)."""
    d = (a - b) % math.pi
    return min(d, math.pi - d)


# -- the part, and only the part ------------------------------------------------------------


def test_the_part_is_found_where_it_is_its_size_and_its_axes():
    box = Box(0.34, 0.02, 0.060, 0.040, 0.030, math.radians(25))
    sc = detect([box], spec=SPEC)
    assert len(sc.parts) == 1 and not sc.rejected
    p = sc.parts[0]
    assert math.dist(p.centre[:2], (box.x, box.y)) < 0.003
    assert p.centre[2] == pytest.approx(TABLE + 0.030, abs=0.002)
    assert p.length_m == pytest.approx(0.060, abs=0.004) and p.width_m == pytest.approx(0.040, abs=0.004)
    assert p.height_m == pytest.approx(0.030, abs=0.002)
    assert ang_diff(p.theta, box.theta) < math.radians(3)  # the long side: the fingers close across it
    assert sc.surface.source == "fitted" and sc.surface.origin[2] == pytest.approx(TABLE, abs=0.002)


def test_things_the_wrong_size_are_rejected_with_the_reason():
    boxes = [
        Box(0.30, -0.07, 0.060, 0.040, 0.030),  # the part
        Box(0.40, -0.07, 0.085, 0.040, 0.030),  # too long (not two parts end to end)
        Box(0.30, 0.07, 0.060, 0.040, 0.060),  # too tall
        Box(0.40, 0.07, 0.030, 0.030, 0.030),  # too short
    ]
    sc = detect(boxes, spec=SPEC)
    assert [(round(p.centre[0], 2), round(p.centre[1], 2)) for p in sc.parts] == [(0.30, -0.07)]
    why = {(round(p.centre[0], 2), round(p.centre[1], 2)): p.why for p in sc.rejected}
    assert why[(0.40, -0.07)] == "too long"
    assert why[(0.30, 0.07)] == "too tall"
    assert why[(0.40, 0.07)] == "too short"


def test_two_parts_touching_end_to_end_are_named_as_such():
    sc = detect([Box(0.32, 0.0, 0.060, 0.040, 0.030), Box(0.38, 0.0, 0.060, 0.040, 0.030)], spec=SPEC)
    assert not sc.parts and [p.why for p in sc.rejected] == ["2 parts touching?"]


def test_parts_close_but_apart_stay_apart():
    sc = detect([Box(0.33, 0.0, 0.060, 0.040, 0.030), Box(0.33, 0.046, 0.060, 0.040, 0.030)], spec=SPEC)
    assert len(sc.parts) == 2  # a 6 mm gap has the table's depth in it


def test_holes_in_the_depth_of_a_top_face_are_filled():
    box = Box(0.35, 0.0, 0.060, 0.040, 0.030)
    # a few pixels of dropout in the middle of the top (white foam under an IR speckle)
    sc = detect([box], spec=SPEC, holes=[(155, 85, 163, 93)])
    assert len(sc.parts) == 1
    assert sc.parts[0].width_m == pytest.approx(0.040, abs=0.004)


def test_a_part_cut_off_by_the_picture_edge_is_not_picked():
    sc = detect([Box(0.35, 0.15, 0.060, 0.040, 0.030)], spec=SPEC)
    assert not sc.parts and sc.rejected[0].why == "cut off by the edge of the picture"


def test_an_empty_table_is_no_parts_and_no_complaint():
    sc = detect([], spec=SPEC)
    assert sc.parts == [] and sc.rejected == [] and sc.surface is not None


def test_no_depth_at_all_says_so():
    sc = find_parts(W, H, bytes(W * H * 2), 0.001, K, CAM, spec=SPEC)
    assert sc.parts == [] and sc.notes == ["almost no depth in this frame"]


def test_a_tilted_camera_measures_the_same_part():
    tilt = Transform.from_pose([0, 0, 0, math.radians(20), 0, 0])  # 20° about the camera's X
    T = camera_looking_down(0.30, -0.08, 0.10, yaw=0.4).compose(tilt)
    axis, o = T.rotate((0, 0, 1)), T.translation
    t = (TABLE + 0.03 - o[2]) / axis[2]
    hit = (o[0] + t * axis[0], o[1] + t * axis[1])  # where the optical axis meets the part's top
    box = Box(hit[0] + 0.01, hit[1] - 0.01, 0.060, 0.040, 0.030, 0.2)
    sc = detect([box], T=T, spec=SPEC)
    assert len(sc.parts) == 1, sc.as_dict()
    p = sc.parts[0]
    assert math.dist(p.centre[:2], (box.x, box.y)) < 0.004
    assert p.length_m == pytest.approx(0.06, abs=0.005) and p.width_m == pytest.approx(0.04, abs=0.005)
    assert ang_diff(p.theta, box.theta) < math.radians(4)


def test_a_camera_only_preview_fits_the_table_in_the_camera_frame():
    box = Box(0.35, 0.0, 0.060, 0.040, 0.030, 0.5)
    depth = scene([box])
    sc = find_parts(W, H, depth, 0.001, K, None, spec=SPEC)
    assert len(sc.parts) == 1 and sc.surface.source == "fitted"
    assert sc.parts[0].height_m == pytest.approx(0.03, abs=0.002)
    assert abs(sc.parts[0].pixel[0] - W / 2) <= 2 and abs(sc.parts[0].pixel[1] - H / 2) <= 2


def test_the_overlay_corners_are_the_parts_corners_in_the_picture():
    sc = detect([Box(0.35, 0.0, 0.060, 0.040, 0.030)], spec=SPEC)
    xs = sorted(c[0] for c in sc.parts[0].corners_px)
    ys = sorted(c[1] for c in sc.parts[0].corners_px)
    # the camera looks straight down from 0.36 m over the top: 60 mm is ~38 px, 40 mm ~26 px
    assert xs[-1] - xs[0] == pytest.approx(0.060 * K["fx"] / 0.36, abs=3)
    assert ys[-1] - ys[0] == pytest.approx(0.040 * K["fx"] / 0.36, abs=3)


def test_the_same_frame_gives_the_same_answer():
    boxes = [Box(0.30, -0.05, 0.06, 0.04, 0.03, 0.3), Box(0.40, 0.05, 0.06, 0.04, 0.03, -1.0)]
    depth = scene(boxes)
    a = find_parts(W, H, depth, 0.001, K, None, spec=SPEC).as_dict()
    b = find_parts(W, H, depth, 0.001, K, None, spec=SPEC).as_dict()
    assert a == b


# -- the surface ----------------------------------------------------------------------------


def test_a_taught_plane_follows_a_table_a_few_mm_off_and_reports_it():
    taught = Surface.level(TABLE - 0.004).__class__(
        (0.2, -0.15, TABLE - 0.004), (1, 0, 0), (0, 1, 0), (0, 0, 1), (0.0, 0.3, 0.0, 0.3), "taught"
    )
    sc = detect([Box(0.35, 0.0, 0.06, 0.04, 0.03)], spec=SPEC, surface=taught)
    assert sc.surface.source == "taught" and sc.surface.offset_m == pytest.approx(0.004, abs=0.0015)
    assert sc.parts[0].height_m == pytest.approx(0.03, abs=0.002)


def test_a_taught_plane_far_off_the_table_is_flagged_not_trusted_blindly():
    taught = Surface((0.2, -0.15, TABLE - 0.025), (1, 0, 0), (0, 1, 0), (0, 0, 1), None, "taught")
    sc = detect([Box(0.35, 0.0, 0.06, 0.04, 0.03)], spec=SPEC, surface=taught)
    assert any("re-teach" in n for n in sc.notes)


def test_parts_outside_the_taught_area_are_not_picked():
    area = Surface.from_points((0.25, -0.10, TABLE), (0.45, -0.10, TABLE), (0.30, 0.0, TABLE))
    assert area.area == pytest.approx((0.0, 0.20, 0.0, 0.10))
    sc = detect(
        [Box(0.33, -0.05, 0.06, 0.04, 0.03), Box(0.33, 0.07, 0.06, 0.04, 0.03)], spec=SPEC, surface=area
    )
    assert [round(p.centre[1], 2) for p in sc.parts] == [-0.05]
    assert [p.why for p in sc.rejected] == ["outside the pick area"]


def test_three_points_make_an_upward_plane_whichever_way_they_were_taught():
    a = Surface.from_points((0, 0, 0), (0.2, 0, 0), (0.0, 0.1, 0))
    b = Surface.from_points((0, 0, 0), (0.2, 0, 0), (0.0, -0.1, 0))  # the far side on the other hand
    for s in (a, b):
        assert s.normal == pytest.approx((0, 0, 1))
        assert s.inside((0.1, math.copysign(0.05, s.local((0, 0.1 if s is a else -0.1, 0))[1]), 0))
    with pytest.raises(ValueError, match="collinear|coincide"):
        Surface.from_points((0, 0, 0), (0.1, 0, 0), (0.2, 0, 0))


@given(
    tilt=st.floats(0, 8),
    heading=st.floats(-math.pi, math.pi),
    sx=st.floats(0.05, 0.5),
    sy=st.floats(0.05, 0.5),
)
def test_a_plane_round_trips_through_its_pose_form(tilt, heading, sx, sy):
    pts = [(0.2, 0.1, -0.2), (0.2 + sx * math.cos(heading), 0.1 + sx * math.sin(heading), -0.2)]
    far = (
        0.2 - sy * math.sin(heading),
        0.1 + sy * math.cos(heading),
        -0.2 + sy * math.sin(math.radians(tilt)),
    )
    s = Surface.from_points(pts[0], pts[1], far)
    T = Transform.from_axes(s.x_axis, s.y_axis, s.normal, s.origin)
    back = Surface.from_pose(T.to_pose(), (s.area[1], s.area[3]))
    assert back.normal == pytest.approx(s.normal, abs=1e-6)
    assert back.area == pytest.approx(s.area, abs=1e-6)
    assert s.tilt_deg() == pytest.approx(math.degrees(math.atan(math.sin(math.radians(tilt)))), abs=0.5)


# -- reach ----------------------------------------------------------------------------------


def test_reach_is_the_base_radius_plus_150_to_the_reach_minus_150():
    r = Reach.for_model("UR3e")
    assert r.min_m == pytest.approx(0.064 + 0.150) and r.max_m == pytest.approx(0.500 - 0.150)
    assert Reach.for_model("UR3e", inner_margin_m=0.2).min_m == pytest.approx(0.264)
    assert Reach.for_model("KUKA") is None
    assert r.why_not((0.30, 0.0, 0)) is None
    assert r.why_not((0.10, 0.10, 0)).startswith("too close to the base")
    assert r.why_not((0.30, 0.25, 0)).startswith("out of reach")


def test_parts_outside_the_reach_annulus_are_reported_not_picked():
    sc = detect(
        [Box(0.30, -0.05, 0.06, 0.04, 0.03), Box(0.42, 0.05, 0.06, 0.04, 0.03)],
        spec=SPEC,
        reach=Reach(0.214, 0.35),
    )
    assert [round(p.centre[0], 2) for p in sc.parts] == [0.30]
    assert sc.rejected[0].why.startswith("out of reach")


# -- the fingers' room ----------------------------------------------------------------------


def test_a_neighbour_where_a_finger_goes_down_blocks_the_pick():
    fingers = {"grasp_below_m": 0.015, "stroke_m": 0.050}
    # the part's short side is along base Y; the fingers come down ~25-39 mm either side of it
    alone = detect([Box(0.35, -0.04, 0.06, 0.04, 0.03)], spec=SPEC, fingers=fingers)
    assert len(alone.parts) == 1
    crowded = detect(
        [Box(0.35, -0.04, 0.06, 0.04, 0.03), Box(0.35, 0.012, 0.06, 0.04, 0.03)],
        spec=SPEC,
        fingers=fingers,
    )
    assert all(p.why and p.why.startswith("no room for a finger") for p in crowded.rejected)
    assert not crowded.parts


# -- clear the way (Nick, 2026-10-02: pick the parts that make other parts easier to pick) ----

FINGERS = {"grasp_below_m": 0.015, "stroke_m": 0.050}
# B lies along X, so its fingers come down beside it along Y — where A stands. A lies along Y,
# its fingers come down along X, clear of B: A can go now, B once A is gone.
PINNED = Box(0.35, 0.0, 0.06, 0.04, 0.03)
BLOCKER = Box(0.35, 0.06, 0.06, 0.04, 0.03, math.pi / 2)  # 10 mm off B, 9 mm into its finger zone


def test_a_part_pinned_only_by_a_pickable_neighbour_is_picked_after_it():
    sc = detect([PINNED, BLOCKER], spec=SPEC, fingers=FINGERS)
    assert not sc.rejected and len(sc.parts) == 2
    first, second = sorted(sc.parts, key=lambda p: p.order)
    assert math.dist(first.centre[:2], (BLOCKER.x, BLOCKER.y)) < 0.005  # the blocker goes first
    assert math.dist(second.centre[:2], (PINNED.x, PINNED.y)) < 0.005
    assert [p.order for p in sc.parts] == [1, 2]  # the list is in pick order


def test_the_same_holds_with_the_nodes_finger_room():
    room = {"grasp_below_m": 0.015, "room_m": 0.020}
    sc = detect([PINNED, BLOCKER], spec=SPEC, fingers=room)
    assert not sc.rejected and [round(p.centre[1], 2) for p in sc.parts] == [0.06, 0.0]


def test_a_part_pinned_by_something_that_is_not_a_part_stays_out():
    post = Box(0.35, 0.04, 0.02, 0.02, 0.03)  # too small to be the part: it never leaves
    sc = detect([PINNED, post], spec=SPEC, fingers=FINGERS)
    assert not sc.parts
    pinned = [p for p in sc.rejected if math.dist(p.centre[:2], (PINNED.x, PINNED.y)) < 0.005]
    assert pinned and pinned[0].why.startswith("no room for a finger")


def test_the_part_that_frees_another_goes_before_the_pictures_first():
    # FREE is first in the picture's order (front row, left) but frees nobody
    free = Box(0.27, -0.06, 0.06, 0.04, 0.03)
    sc = detect([free, PINNED, BLOCKER], spec=SPEC, fingers=FINGERS, order=("LR", "FB"))
    assert len(sc.parts) == 3 and not sc.rejected
    first = min(sc.parts, key=lambda p: p.order)
    assert math.dist(first.centre[:2], (BLOCKER.x, BLOCKER.y)) < 0.005
    # without the finger check there is nothing to clear, and the picture's order stands
    plain = detect([free, PINNED, BLOCKER], spec=SPEC, order=("LR", "FB"))
    assert math.dist(min(plain.parts, key=lambda p: p.order).centre[:2], (free.x, free.y)) < 0.005


# -- pick order -----------------------------------------------------------------------------

GRID = [Box(0.28 + 0.07 * c, -0.07 + 0.07 * r, 0.05, 0.035, 0.03) for r in range(3) for c in range(3)]
GRID_SPEC = PartSpec.from_mm(50, 35, 30)


def numbered(order, T=CAM):
    sc = detect(GRID, T=T, spec=GRID_SPEC, order=order)
    assert len(sc.parts) == 9
    return {(round(p.centre[0], 2), round(p.centre[1], 2)): p.order for p in sc.parts}, sc


def test_pick_order_numbers_follow_the_picture_left_to_right_rows_front_to_back():
    # CAM looks down with image right = base +X and image down = base -Y: the picture's
    # bottom (front) is base -Y
    got, sc = numbered(("LR", "FB"))
    rows = [
        [got[(round(0.28 + 0.07 * c, 2), round(-0.07 + 0.07 * r, 2))] for c in range(3)] for r in range(3)
    ]
    assert rows == [[1, 2, 3], [4, 5, 6], [7, 8, 9]]
    px = sorted(sc.parts, key=lambda p: p.order)
    assert px[0].pixel[0] < px[1].pixel[0] < px[2].pixel[0]  # 1 2 3 left to right on screen
    assert px[0].pixel[1] > px[3].pixel[1]  # row 1 below row 2 on screen


@pytest.mark.parametrize(
    ("order", "first_three_on_screen"),
    [
        (("LR", "FB"), "bottom row, left to right"),
        (("RL", "FB"), "bottom row, right to left"),
        (("LR", "BF"), "top row, left to right"),
        (("FB", "LR"), "left column, bottom to top"),
        (("BF", "RL"), "right column, top to bottom"),
    ],
)
def test_every_order_numbers_the_screen_the_way_it_says(order, first_three_on_screen):
    _, sc = numbered(order)
    a, b, c = sorted(sc.parts, key=lambda p: p.order)[:3]
    us, vs = [p.pixel[0] for p in (a, b, c)], [p.pixel[1] for p in (a, b, c)]
    where, direction = first_three_on_screen.split(", ")
    same = vs if where.endswith("row") else us
    assert max(same) - min(same) <= 3  # one row / one column
    seq = us if where.endswith("row") else vs
    want_up = direction in ("left to right", "top to bottom")
    assert seq == sorted(seq, reverse=not want_up)
    edge = {"bottom row": max, "top row": min, "left column": min, "right column": max}[where]
    all_same_axis = [p.pixel[1] if where.endswith("row") else p.pixel[0] for p in sc.parts]
    assert abs(same[0] - edge(all_same_axis)) <= 3


def test_the_order_is_the_pictures_whatever_the_wrist_heading():
    T = camera_looking_down(0.35, 0.0, 0.12, yaw=math.radians(90))
    _, sc = numbered(("LR", "FB"), T=T)
    a, b, c = sorted(sc.parts, key=lambda p: p.order)[:3]
    assert a.pixel[0] < b.pixel[0] < c.pixel[0]


def test_a_slightly_crooked_row_is_still_one_row():
    boxes = [Box(0.28 + 0.07 * c, 0.0 + (0.006 if c == 1 else 0.0), 0.05, 0.035, 0.03) for c in range(3)]
    sc = detect(boxes, spec=GRID_SPEC, order=("LR", "FB"))
    assert [round(p.centre[0], 2) for p in sc.parts] == [0.28, 0.35, 0.42]


def test_orders_must_cross():
    assert parse_order("lr,fb") == ("LR", "FB")
    for bad in ("LR,RL", "FB", "LR,FB,LR", "UP,DOWN"):
        with pytest.raises(ValueError):
            parse_order(bad)
    with pytest.raises(ValueError):
        order_parts([], CAM, Surface.level(0), ("FB", "BF"))


# -- geometry, adversarially -----------------------------------------------------------------


@settings(max_examples=150)
@given(
    cx=st.floats(-1, 1),
    cy=st.floats(-1, 1),
    length=st.floats(0.01, 0.4),
    ratio=st.floats(0.1, 1.0),
    ang=st.floats(-math.pi, math.pi),
    n=st.integers(4, 40),
)
def test_min_area_rect_recovers_a_sampled_rectangle(cx, cy, length, ratio, ang, n):
    width = length * ratio
    c, s = math.cos(ang), math.sin(ang)
    pts = []
    for i in range(n + 1):
        for j in range(n + 1):
            u, v = (i / n - 0.5) * length, (j / n - 0.5) * width
            pts.append((cx + u * c - v * s, cy + u * s + v * c))
    rx, ry, ra, rl, rw = min_area_rect(pts)
    assert rl >= rw
    assert rl == pytest.approx(length, rel=1e-6, abs=1e-9) and rw == pytest.approx(width, rel=1e-6, abs=1e-9)
    assert math.dist((rx, ry), (cx, cy)) < 1e-6
    if ratio < 0.98:
        assert ang_diff(ra, ang) < 1e-5
    assert -math.pi / 2 < ra <= math.pi / 2


@given(st.lists(st.tuples(st.floats(-1, 1), st.floats(-1, 1)), min_size=0, max_size=60))
def test_the_hull_holds_every_point(points):
    hull = convex_hull(points)
    if len(hull) < 3:
        return
    for p in points:
        for k in range(len(hull)):
            a, b = hull[k], hull[(k + 1) % len(hull)]
            assert (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0]) >= -1e-9


@settings(max_examples=25, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(
    x=st.floats(0.28, 0.42),
    y=st.floats(-0.06, 0.06),
    theta=st.floats(-math.pi / 2, math.pi / 2),
    yaw=st.floats(-math.pi, math.pi),
    lift=st.floats(0.08, 0.16),
)
def test_any_placement_under_any_wrist_heading_measures_the_part(x, y, theta, yaw, lift):
    box = Box(x, y, 0.060, 0.040, 0.030, theta)
    T = camera_looking_down(x + 0.01, y - 0.01, lift, yaw=yaw)
    sc = detect([box], T=T, spec=SPEC)
    assert len(sc.parts) == 1, sc.as_dict()
    p = sc.parts[0]
    assert math.dist(p.centre[:2], (x, y)) < 0.004
    assert ang_diff(p.theta, theta) < math.radians(4)
    assert p.width_m == pytest.approx(0.040, abs=0.005)


@settings(max_examples=40, deadline=None)
@given(st.binary(min_size=0, max_size=64))
def test_garbage_depth_never_raises(noise):
    depth = (noise * (W * H * 2 // max(1, len(noise)) + 1))[: W * H * 2] if noise else bytes(W * H * 2)
    sc = find_parts(W, H, depth, 0.001, K, CAM, spec=SPEC)
    assert isinstance(sc.parts, list)


def packed(rng, n):
    """Up to ``n`` parts, each lying along X or Y, 8 mm or more apart: never touching (one
    blob), often inside each other's finger zones."""
    rects = []
    for _ in range(200):
        if len(rects) == n:
            break
        turned = rng.random() < 0.5
        hx, hy = (0.02, 0.03) if turned else (0.03, 0.02)
        x, y = rng.uniform(0.24, 0.46), rng.uniform(-0.09, 0.09)
        gap = min(
            (max(abs(x - a) - hx - ahx, abs(y - b) - hy - ahy) for a, b, ahx, ahy, _ in rects), default=1
        )
        if gap >= 0.008:
            rects.append((x, y, hx, hy, turned))
    return [Box(x, y, 0.06, 0.04, 0.03, math.pi / 2 if t else 0.0) for x, y, _, _, t in rects]


@settings(max_examples=30, deadline=None, suppress_health_check=[HealthCheck.too_slow])
@given(st.integers(0, 2**32 - 1), st.integers(2, 7))
def test_every_part_in_the_plan_is_free_once_the_ones_before_it_are_gone(seed, n):
    """The contract of clearing the way, checked against fresh pictures: take the planned
    part out of the scene, re-render, and the next planned part must be pickable *then*.
    (These layouts pin parts often: over 40 of them the old rule turned away 10 parts
    that clearing the way picks.)"""
    boxes = packed(random.Random(seed), n)

    def box_of(part, among):
        return min(among, key=lambda b: math.dist(part.centre[:2], (b.x, b.y)))

    sc = detect(boxes, spec=SPEC, fingers=FINGERS)
    plan = [box_of(p, boxes) for p in sorted(sc.parts, key=lambda p: p.order)]
    left = list(boxes)
    for box in plan:
        now = detect(left, spec=SPEC, fingers=FINGERS)
        assert box in [box_of(p, left) for p in now.parts], (
            "a planned part is still pinned when its turn comes"
        )
        left.remove(box)
