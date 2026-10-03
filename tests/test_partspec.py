"""Finding the part by its rough size: the spec, the detector's gate, the height it measures.

Synthetic scenes (``tests.test_pickcycle.scene``: a floor 0.40 m from a camera with
fx = 200 px, blocks 40 mm proud of it) put parts of known size in front of the
detector; the contract is which of them come back, and why the rest did not.
"""

from __future__ import annotations

import random

import pytest

from perceptronics import partspec
from perceptronics.partspec import PartSpec
from perceptronics.pickcycle import detect_blocks, surface_height, top_face, white_blobs
from tests.test_pickcycle import H, K, W, scene
from urctl.pose import Transform

# At 0.36 m (the top of a 40 mm block on the 0.40 m floor) one pixel is 1.8 mm. BLOCK
# measures 54 x 40 mm (30 x 22 px: the blob's 4 px grid clips its top two rows), SMALL
# 43 x 43 mm, LONG 108 x 54 mm.
BLOCK = (40, 30, 70, 54)
SMALL = (100, 68, 124, 92)
LONG = (20, 60, 80, 90)


def detect(blocks, part=None, **scene_kw):
    rgb, depth = scene(blocks, **scene_kw)
    rejects: list[dict] = []
    found = detect_blocks(W, H, 3, rgb, depth, 0.001, K, Transform(), Transform(), part=part, rejects=rejects)
    return found, rejects


# -- the spec ---------------------------------------------------------------------------------


def test_the_footprint_is_either_way_round_and_the_token_round_trips():
    a, b = PartSpec.from_mm(40, 60, 30), PartSpec.from_mm(60, 40, 30)
    assert a == b and a.length_m == pytest.approx(0.06) and a.width_m == pytest.approx(0.04)
    assert a.token() == "part=60x40x30 tol=25"
    assert partspec.parse(a.token()) == a
    assert partspec.parse("FIND p[0,0,0,0,0,0] part=60.5x40 tol=12.5") == PartSpec.from_mm(
        60.5, 40, None, 12.5
    )
    assert partspec.parse("FIND p[0,0,0,0,0,0] u=1 v=2") is None


def test_a_measurement_is_the_part_within_tolerance_and_says_why_not_otherwise():
    spec = PartSpec.from_mm(60, 40, 30, tol_pct=20)
    assert spec.why_not(0.060, 0.040, 0.030) is None
    assert spec.why_not(0.071, 0.047, 0.035) is None  # just inside 20 %
    assert spec.why_not(0.060, 0.040, None) is None  # height not seen: not held against it
    assert spec.why_not(0.080, 0.040, 0.030) == "too long"
    assert spec.why_not(0.060, 0.050, 0.030) == "too wide"
    assert spec.why_not(0.040, 0.040, 0.030) == "too short"
    assert spec.why_not(0.060, 0.015, 0.030) == "too narrow"  # narrower than every face
    # 60 x 25 is within the 60 x 30 face, which stands 40 tall: that is the face to complain about
    assert spec.why_not(0.060, 0.025, 0.030) == "too flat"
    assert spec.why_not(0.060, 0.040, 0.040) == "too tall"
    assert spec.why_not(0.060, 0.040, 0.010) == "too flat"
    assert spec.why_not(0.120, 0.040, 0.030) == "2 parts touching?"
    # a small part is never held to less than the measurement's own ±5 mm
    tiny = PartSpec.from_mm(10, 8, tol_pct=5)
    assert tiny.why_not(0.014, 0.012, None) is None


def test_a_box_is_the_part_lying_on_any_of_its_faces():
    spec = PartSpec.from_mm(60, 40, 30, tol_pct=10)
    assert spec.poses() == [(0.06, 0.04, 0.03), (0.06, 0.03, 0.04), (0.04, 0.03, 0.06)]
    assert spec.why_not(0.060, 0.040, 0.030) is None  # flat
    assert spec.why_not(0.060, 0.030, 0.040) is None  # on its long side
    assert spec.why_not(0.040, 0.030, 0.060) is None  # on its end
    assert spec.why_not(0.040, 0.030, 0.030) == "too flat"  # the end face, but only 30 tall
    assert spec.heights() == [0.03, 0.04, 0.06]
    assert spec.near_miss(0.040, 0.030, 0.050)


def test_a_cube_has_one_pose_and_a_cylinder_stands_on_its_end():
    assert PartSpec.from_mm(30, 30, 30).poses() == [(0.03, 0.03, 0.03)]
    can = PartSpec.from_mm(40, 40, 60, shape="cyl")
    assert can.poses() == [(0.04, 0.04, 0.06)]
    assert can.why_not(0.060, 0.040, 0.040) == "too long"  # a can on its side is not this part
    flat = PartSpec.from_mm(60, 40)  # no height: as given
    assert flat.poses() == [(0.06, 0.04, None)] and flat.heights() == []


@pytest.mark.parametrize(
    "text",
    [
        "part=60",
        "part=60x",
        "part=x40",
        "part=60x40x",
        "part=-60x40",
        "part=60x40x30x20",
        "part=60x40.5.5",
        "part=4x40",  # under 5 mm
        "part=600x40",  # over 500 mm
        "part=60x40 tol=0",
        "part=60x40 tol=101",
        "part=60x40 tol=abc",
        "part=60x40 tol=-5",
        "part=1e3x40",
    ],
)
def test_malformed_specs_are_refused(text):
    with pytest.raises(ValueError):
        partspec.parse(text)


def test_random_tokens_parse_to_a_valid_spec_or_a_refusal():
    rng = random.Random(20260928)
    alphabet = "0123456789.xX-e tol=part=\x00é"
    for _ in range(3000):
        text = "part=" + "".join(rng.choice(alphabet) for _ in range(rng.randint(0, 24)))
        try:
            spec = partspec.parse(text)
        except ValueError:
            continue
        assert spec is not None
        assert 0.005 <= spec.width_m <= spec.length_m <= 0.5
        assert spec.height_m is None or 0.005 <= spec.height_m <= 0.5
        assert 0.05 <= spec.tol <= 1.0
        assert partspec.parse(spec.token()) == spec


@pytest.mark.parametrize(
    ("payload", "want"),
    [
        ({}, None),
        ({"part": ""}, None),
        ({"part": "60x40x30"}, PartSpec.from_mm(60, 40, 30)),
        ({"part": "60x40", "tol": 10}, PartSpec.from_mm(60, 40, None, 10)),
    ],
)
def test_json_and_query_forms(payload, want):
    assert partspec.from_payload(payload) == want
    qs = {k: [str(v)] for k, v in payload.items()}
    assert partspec.from_query(qs) == want


@pytest.mark.parametrize(
    "payload",
    [
        {"part": 60},
        {"part": ["60x40"]},
        {"part": "big"},
        {"part": "60x40", "tol": "ten"},
        {"part": "60x40", "tol": True},
    ],
)
def test_malformed_json_specs_are_refused(payload):
    with pytest.raises(ValueError):
        partspec.from_payload(payload)


def test_json_specs_fuzz_to_a_spec_or_a_refusal():
    rng = random.Random(7631)
    pool = ["60x40", "60x40x30", "1x1", "60x40 ", " 60X40", "60x40x", 60, None, "", [], {}, True, "\x00"]
    tols = [25, 0, 5, 100, 101, -1, 1e308, 10**400, float("nan"), float("inf"), "25", None, True, 12.5]
    for _ in range(2000):
        payload = {}
        if rng.random() < 0.9:
            payload["part"] = rng.choice(pool)
        if rng.random() < 0.6:
            payload["tol"] = rng.choice(tols)
        try:
            spec = partspec.from_payload(payload)
        except ValueError:
            continue
        assert spec is None or (0.05 <= spec.tol <= 1.0 and spec.width_m <= spec.length_m)


# -- the height above the table ---------------------------------------------------------------


@pytest.mark.parametrize("height", [0.02, 0.04, 0.08])
def test_the_height_is_measured_from_the_surface_around_the_part(height):
    rgb, depth = scene([BLOCK], height=height)
    (blob,) = white_blobs(W, H, 3, rgb)
    tf = top_face(W, H, 3, rgb, depth, 0.001, K, blob["bbox"])
    assert surface_height(W, H, 3, rgb, depth, 0.001, K, blob["bbox"], tf) == pytest.approx(height, abs=0.002)


def test_neighbouring_parts_are_not_mistaken_for_the_table():
    # a big block (60 x 48 px: a 15 px ring) walled in 5 px away by a lower white frame
    # (20 mm tall) that fills two thirds of the ring: counted as the surface, the block
    # would read 20 mm tall instead of 40
    big = (40, 30, 100, 78)
    frame = [(15, 5, 125, 25), (15, 83, 125, 103), (15, 25, 35, 83), (105, 25, 125, 83)]
    rgb, depth = scene(frame, height=0.02)
    rgb, depth = bytearray(rgb), bytearray(depth)
    x0, y0, x1, y1 = big
    for y in range(y0, y1):
        for x in range(x0, x1):
            rgb[(y * W + x) * 3 : (y * W + x) * 3 + 3] = b"\xf0\xf0\xf2"
            depth[2 * (y * W + x)], depth[2 * (y * W + x) + 1] = 360 & 0xFF, 360 >> 8
    rgb, depth = bytes(rgb), bytes(depth)
    blob = next(b for b in white_blobs(W, H, 3, rgb) if b["bbox"][0] >= 36)
    tf = top_face(W, H, 3, rgb, depth, 0.001, K, blob["bbox"])
    assert surface_height(W, H, 3, rgb, depth, 0.001, K, blob["bbox"], tf) == pytest.approx(0.04, abs=0.002)


def test_no_visible_surface_means_no_height_not_a_guess():
    rgb, depth = scene([BLOCK])
    (blob,) = white_blobs(W, H, 3, rgb)
    tf = top_face(W, H, 3, rgb, depth, 0.001, K, blob["bbox"])
    holes = bytes(len(depth))  # the depth all holes: the colour blob is still there
    assert surface_height(W, H, 3, rgb, holes, 0.001, K, blob["bbox"], tf) is None


# -- the detector with a spec -----------------------------------------------------------------


def test_without_a_spec_the_foam_block_gate_is_unchanged():
    found, rejects = detect([BLOCK, SMALL, LONG])
    assert sorted(round(b.major_m * 1000) for b in found) == [43, 54]
    assert [r["why"] for r in rejects] == ["not a block"]  # the 108 mm one
    assert all(b.height_m == pytest.approx(0.04, abs=0.002) for b in found)


def test_a_spec_keeps_only_the_part_that_size():
    found, rejects = detect([BLOCK, SMALL, LONG], PartSpec.from_mm(54, 40, 40, tol_pct=15))
    assert [(round(b.major_m * 1000), round(b.minor_m * 1000)) for b in found] == [(54, 40)]
    # SMALL's footprint fits the 40 x 40 face, which stands 54 tall (a box lies on any face)
    assert sorted(r["why"] for r in rejects) == ["2 parts touching?", "too flat"]


def test_a_spec_can_ask_for_a_part_bigger_than_a_foam_block():
    found, _ = detect([BLOCK, LONG], PartSpec.from_mm(108, 54))
    assert len(found) == 1 and round(found[0].major_m * 1000) == 108


def test_the_height_tells_a_flat_part_from_a_block_of_the_same_footprint():
    spec = PartSpec.from_mm(54, 40, 40)
    assert len(detect([BLOCK], spec)[0]) == 1
    found, rejects = detect([BLOCK], spec, height=0.008)  # a sticker, a sheet of paper
    assert found == [] and rejects[0]["why"] == "too flat" and rejects[0]["height_mm"] == 8
    # no height given: not checked
    assert len(detect([BLOCK], PartSpec.from_mm(54, 40), height=0.008)[0]) == 1


# -- cylinders and near misses (0.7.0) ---------------------------------------------------------


def test_a_cylinder_is_its_diameter_twice_and_round_trips():
    from perceptronics.partspec import parse

    c = PartSpec.from_mm(40, 40, 30, 20, shape="cyl")
    assert c.is_round and c.token() == "part=40x40x30 tol=20 shape=cyl"
    assert parse(f"FIND p[0,0,0,0,0,0] {c.token()} proto=2") == c
    assert c.as_dict()["shape"] == "cyl"
    box = parse("part=40x40x30 tol=20")
    assert box.shape == "box" and not box.is_round and box != c
    assert "shape" not in box.token()  # an older pick server reads a box's token as it always did


@pytest.mark.parametrize(
    "text",
    ["part=50x40x30 shape=cyl", "part=40x40 shape=hex", "part=40x40 shape=", "part=40x40 shape=cyl;x"],
)
def test_a_malformed_shape_is_refused(text):
    from perceptronics.partspec import parse

    with pytest.raises(ValueError):
        parse(text)


def test_a_near_miss_is_a_candidate_a_little_off_and_nothing_else():
    spec = PartSpec.from_mm(50, 30, 30, 25)
    assert spec.near_miss(0.050, 0.030, 0.030)  # the part itself
    assert spec.why_not(0.066, 0.030, 0.030) == "too long" and spec.near_miss(0.066, 0.030, 0.030)
    assert spec.why_not(0.050, 0.030, 0.019) == "too flat" and spec.near_miss(0.050, 0.030, 0.019)
    assert spec.near_miss(0.100, 0.030, 0.030)  # two parts end to end: worth telling the operator
    assert not spec.near_miss(0.200, 0.090, 0.030)  # a clamp
    assert not spec.near_miss(0.100, 0.030, 0.090)  # parts' worth long, but nothing like as tall
    assert not spec.near_miss(0.050, 0.030, 0.090)  # three times as tall
    assert spec.near_miss(0.066, 0.030, None)  # no height measured: not held against it
