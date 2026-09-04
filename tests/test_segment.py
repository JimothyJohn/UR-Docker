"""Click-to-segment + feature extraction on synthetic RGB-D scenes."""

from __future__ import annotations

import math

import pytest

from perception.frame import Frame
from perception.rgbd import DepthImage, RgbdFrame, synthetic_disks, synthetic_intrinsics, synthetic_rgbd
from perception.segment import Mask, Segmenter, StubSegmenter, extract_features, normalize_box


def _scene(width, height, rects, background=(20, 20, 24), wall=1.2):
    """Axis-aligned rectangles ``(x0, y0, x1, y1, rgb, depth_m)`` on a wall."""
    buf = bytearray(bytes(background) * (width * height))
    metres = [wall] * (width * height)
    for x0, y0, x1, y1, rgb, z in rects:
        for y in range(y0, y1):
            for x in range(x0, x1):
                i = y * width + x
                buf[i * 3 : i * 3 + 3] = bytes(rgb)
                metres[i] = z
    return RgbdFrame(
        color=Frame(width, height, bytes(buf)),
        depth=DepthImage.from_metres(width, height, metres),
        intrinsics=synthetic_intrinsics(width, height),
    )


def test_mask_basics():
    m = Mask(4, 2, bytes([0, 1, 1, 0, 0, 0, 1, 0]))
    assert m.area == 3 and m.bbox() == (1, 0, 2, 1) and m.contains(2, 1) and not m.contains(9, 9)
    assert m.pixels() == [1, 2, 6]
    assert Mask.empty(3, 3).bbox() is None
    with pytest.raises(ValueError):
        Mask(2, 2, b"\x01")
    png = m.to_png()
    assert png[:8] == b"\x89PNG\r\n\x1a\n"


def test_stub_segments_each_disk_to_its_analytic_area():
    f = synthetic_rgbd(320, 240)
    seg = StubSegmenter()
    assert isinstance(seg, Segmenter)
    for cx, cy, r, _rgb, z in synthetic_disks(320, 240):
        m = seg.segment(f, (cx, cy))
        assert abs(m.area - math.pi * r * r) / (math.pi * r * r) < 0.02
        assert m.contains(cx, cy) and not m.contains(0, 0)
        ft = extract_features(m, f)
        assert ft is not None
        assert ft.depth_median_m == pytest.approx(z)
        assert ft.centroid_px == pytest.approx((cx, cy), abs=0.6)
        assert ft.extent_m[0] == pytest.approx(2 * r * z / f.intrinsics.fx, rel=0.03)
        assert ft.elongation == pytest.approx(1.0, abs=0.05)
        assert ft.point_m[2] == pytest.approx(z)


def test_seed_outside_frame_is_rejected():
    f = synthetic_rgbd(32, 24)
    with pytest.raises(ValueError):
        StubSegmenter().segment(f, (32, 0))
    with pytest.raises(ValueError):
        StubSegmenter().segment(f, (0, -1))


def test_background_click_is_capped():
    f = synthetic_rgbd(160, 120)
    m = StubSegmenter(max_area_fraction=0.25).segment(f, (2 * 160 // 4 + 40, 5))
    assert m.area <= 0.25 * 160 * 120 + 1


def test_depth_discontinuity_splits_same_colored_objects():
    red = (220, 40, 40)
    # two touching red rectangles: left at 0.5 m, right at 0.9 m
    f = _scene(120, 60, [(10, 10, 60, 50, red, 0.5), (60, 10, 110, 50, red, 0.9)])
    with_depth = StubSegmenter().segment(f, (30, 30))
    assert with_depth.area == 50 * 40 and with_depth.bbox() == (10, 10, 59, 49)
    colour_only = StubSegmenter(use_depth=False, max_area_fraction=1.0).segment(f, (30, 30))
    assert colour_only.area == 100 * 40  # without depth they merge


def test_depth_holes_inside_an_object_do_not_split_it():
    red = (220, 40, 40)
    f = _scene(80, 40, [(10, 10, 70, 30, red, 0.5)])
    # punch a column of invalid depth through the middle
    m = f.depth.values()
    for y in range(40):
        m[y * 80 + 40] = 0
    holed = RgbdFrame(color=f.color, depth=DepthImage(80, 40, m.tobytes()), intrinsics=f.intrinsics)
    assert StubSegmenter().segment(holed, (20, 20)).area == 60 * 20
    assert StubSegmenter(allow_invalid_depth=False).segment(holed, (20, 20)).area == 30 * 20


def test_nearest_object_is_the_closest_disk():
    f = synthetic_rgbd(320, 240)
    m = StubSegmenter().nearest_object(f)
    nearest = min(synthetic_disks(320, 240), key=lambda d: d[4])
    cx, cy, r, rgb, _z = nearest
    assert m.contains(cx, cy) and abs(m.area - math.pi * r * r) / (math.pi * r * r) < 0.02
    assert extract_features(m, f).mean_rgb == rgb
    empty = RgbdFrame(
        color=f.color, depth=DepthImage(320, 240, bytes(320 * 240 * 2)), intrinsics=f.intrinsics
    )
    assert StubSegmenter().nearest_object(empty).area == 0


def test_features_of_an_elongated_rectangle():
    f = _scene(200, 100, [(20, 40, 140, 60, (60, 90, 230), 0.8)])  # 120 x 20 px, horizontal
    m = StubSegmenter().segment(f, (80, 50))
    ft = extract_features(m, f)
    assert ft.area_px == 120 * 20 and ft.bbox == (20, 40, 139, 59)
    assert ft.orientation_deg == pytest.approx(0.0, abs=0.5)
    assert ft.elongation == pytest.approx(6.0, rel=0.02)
    assert ft.fill_ratio == pytest.approx(1.0)
    assert ft.extent_m[0] == pytest.approx(120 * 0.8 / f.intrinsics.fx)
    assert ft.thickness_m == pytest.approx(0.0)
    assert ft.grasp["gripper_yaw_deg"] == pytest.approx(90.0, abs=0.5)
    assert ft.grasp["opening_m"] == pytest.approx(ft.minor_axis_m)
    d = ft.as_dict()
    assert d["depth"]["valid_px"] == 2400 and d["point_m"][2] == pytest.approx(0.8)


def test_rotated_rectangle_orientation():
    # a solid bar at 45 degrees (image axes: +x right, +y down)
    w, h = 120, 120
    buf = bytearray(bytes((20, 20, 24)) * (w * h))
    metres = [1.0] * (w * h)
    for y in range(h):
        for x in range(w):
            u = ((x - 60) + (y - 60)) / math.sqrt(2)
            v = ((x - 60) - (y - 60)) / math.sqrt(2)
            if abs(u) <= 40 and abs(v) <= 4:
                i = y * w + x
                buf[i * 3 : i * 3 + 3] = bytes((220, 40, 40))
                metres[i] = 0.6
    f = RgbdFrame(
        color=Frame(w, h, bytes(buf)),
        depth=DepthImage.from_metres(w, h, metres),
        intrinsics=synthetic_intrinsics(w, h),
    )
    ft = extract_features(StubSegmenter().segment(f, (60, 60)), f)
    assert ft.orientation_deg == pytest.approx(45.0, abs=2.0)
    assert ft.grasp["gripper_yaw_deg"] == pytest.approx(-45.0, abs=2.0)
    assert ft.elongation == pytest.approx(10.0, rel=0.15)


def test_empty_mask_and_size_mismatch():
    f = synthetic_rgbd(32, 24)
    assert extract_features(Mask.empty(32, 24), f) is None
    with pytest.raises(ValueError):
        extract_features(Mask.empty(16, 12), f)


def test_features_without_depth_data():
    f = synthetic_rgbd(64, 48)
    nodepth = RgbdFrame(color=f.color, depth=DepthImage(64, 48, bytes(64 * 48 * 2)), intrinsics=f.intrinsics)
    ft = extract_features(StubSegmenter().segment(nodepth, (16, 24)), nodepth)
    assert ft is not None and ft.depth_median_m is None and ft.point_m is None and ft.extent_m is None
    assert ft.grasp["opening_m"] is None and ft.area_px > 0


# ---- box prompts -------------------------------------------------------------------


def test_box_prompt_segments_the_thing_inside_and_clips_to_it():
    # two same-coloured touching rectangles at the same depth: a point grows across both,
    # a box returns only what's inside it
    f = _scene(80, 60, [(10, 10, 40, 50, (200, 40, 40), 0.8), (40, 10, 70, 50, (200, 40, 40), 0.8)])
    seg = StubSegmenter()
    whole = seg.segment(f, (20, 30))
    assert whole.area == 2 * 30 * 40
    left = seg.segment(f, box=(10, 10, 40, 50))
    assert left.area == 30 * 40 and left.bbox() == (10, 10, 39, 49)
    # box + point: the point seeds, the box clips
    both = seg.segment(f, (15, 15), box=(12, 12, 20, 20))
    assert both.area == 8 * 8 and both.bbox() == (12, 12, 19, 19)


def test_box_centre_seed_on_background_still_clips():
    f = _scene(80, 60, [(10, 10, 30, 30, (200, 40, 40), 0.8)])
    # box around the object but centred on background: region grows over the background,
    # is capped, and is clipped to the box — never the whole frame
    m = StubSegmenter().segment(f, box=(0, 0, 60, 60))
    assert m.area <= 60 * 60 and m.bbox() is not None and m.bbox()[2] < 60 and m.bbox()[3] < 60


@pytest.mark.parametrize(
    "box",
    [
        (0, 0, 0, 0),  # empty
        (5, 5, 5, 20),  # zero width
        (-1, 0, 10, 10),  # off the left edge
        (0, 0, 81, 10),  # past the right edge
        (0, 0, 10, 61),  # past the bottom
        (0, 0, 10),  # wrong arity
        (0, 0, 10, float("nan")),
        (0, 0, 10, float("inf")),
        (0, 0, True, 10),
        "0,0,10,10",
        None,
    ],
)
def test_normalize_box_rejects_garbage(box):
    with pytest.raises((ValueError, TypeError)):
        normalize_box(box, 80, 60)


def test_normalize_box_orders_corners_and_truncates():
    assert normalize_box((40.9, 50.2, 10, 10), 80, 60) == (10, 10, 40, 50)
    assert normalize_box([0, 0, 80, 60], 80, 60) == (0, 0, 80, 60)


def test_point_outside_box_is_rejected():
    f = _scene(80, 60, [(10, 10, 30, 30, (200, 40, 40), 0.8)])
    with pytest.raises(ValueError, match="outside box"):
        StubSegmenter().segment(f, (50, 50), box=(10, 10, 30, 30))
    with pytest.raises(ValueError, match="point, a box, or both"):
        StubSegmenter().segment(f)


def test_mask_clipped():
    m = Mask(4, 3, bytes([1] * 12))
    c = m.clipped((1, 1, 3, 3))
    assert c.area == 4 and c.bbox() == (1, 1, 2, 2)
    assert m.clipped((-5, -5, 99, 99)).area == 12
