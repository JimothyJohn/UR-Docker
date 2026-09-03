"""RGB-D data layer: intrinsics + deprojection, depth images, PNG round trips,
and the viewer's frame container. No camera, no SDK required — the one
SDK-backed test cross-checks our deprojection against librealsense's own
implementation and skips when the library isn't installed."""

from __future__ import annotations

import math

import pytest

from perception.frame import Frame
from perception.pngio import encode_png, load_png, load_png16
from perception.rgbd import (
    DepthImage,
    Intrinsics,
    RgbdFrame,
    pack_rgbd,
    synthetic_disks,
    synthetic_rgbd,
    unpack_rgbd,
)

# ----- intrinsics --------------------------------------------------------------


def test_intrinsics_validation():
    with pytest.raises(ValueError, match="distortion model"):
        Intrinsics(640, 480, 600, 600, 320, 240, model="bogus")
    with pytest.raises(ValueError, match="focal"):
        Intrinsics(640, 480, 0, 600, 320, 240)
    with pytest.raises(ValueError, match="coeffs"):
        Intrinsics(640, 480, 600, 600, 320, 240, coeffs=(0.0, 0.0))  # type: ignore[arg-type]


def test_pinhole_deproject_and_project_roundtrip():
    k = Intrinsics(640, 480, 600.0, 610.0, 320.0, 240.0)
    assert k.deproject(320, 240, 1.0) == (0.0, 0.0, 1.0)
    x, y, z = k.deproject(620, 40, 0.5)
    assert x == pytest.approx(300 / 600 * 0.5)
    assert y == pytest.approx(-200 / 610 * 0.5)
    assert z == 0.5
    u, v = k.project(x, y, z)
    assert (u, v) == pytest.approx((620, 40))
    with pytest.raises(ValueError):
        k.project(0, 0, 0)


def test_forward_distortion_models_refuse_deproject():
    k = Intrinsics(640, 480, 600, 600, 320, 240, model="modified_brown_conrady", coeffs=(0.1, 0, 0, 0, 0))
    with pytest.raises(ValueError, match="cannot deproject"):
        k.deproject(1, 1, 1.0)


def test_zero_coeff_brown_conrady_equals_pinhole():
    a = Intrinsics(640, 480, 600, 600, 320, 240)
    b = Intrinsics(640, 480, 600, 600, 320, 240, model="inverse_brown_conrady")
    assert a.deproject(11, 456, 0.7) == b.deproject(11, 456, 0.7)


def test_fov_and_dict_roundtrip():
    k = Intrinsics(640, 480, 320.0, 320.0, 320.0, 240.0, model="brown_conrady", coeffs=(0.01, 0, 0, 0, 0))
    h, v = k.fov_deg()
    assert h == pytest.approx(90.0)
    assert v == pytest.approx(2 * math.degrees(math.atan2(240, 320)))
    assert Intrinsics.from_dict(k.as_dict()) == k


def test_deproject_matches_librealsense_when_available():
    """Our Python undistort must agree with rs2_deproject_pixel_to_point."""
    from perception.realsense import RealSenseLibraryNotFound, load_api

    try:
        api = load_api()
    except RealSenseLibraryNotFound:
        pytest.skip("librealsense2 not installed")
    coeffs = (0.1, -0.05, 0.001, 0.002, 0.01)
    for model in ("none", "inverse_brown_conrady", "brown_conrady"):
        k = Intrinsics(640, 480, 615.0, 612.0, 318.5, 241.2, model=model, coeffs=coeffs)
        for uv in ((320, 240), (0, 0), (639, 479), (100, 400), (500.5, 12.25)):
            ours = k.deproject(*uv, 0.8)
            theirs = api.deproject(k, *uv, 0.8)
            assert ours == pytest.approx(theirs, abs=1e-5), (model, uv)


# ----- depth image -----------------------------------------------------------------


def test_depth_image_validation_and_access():
    metres = [0.5, 0.0, 1.0, None, 70.0, 0.25]  # row 0: x=0..2, row 1: x=0..2
    d = DepthImage.from_metres(3, 2, metres)  # type: ignore[arg-type]
    assert d.raw_at(0, 0) == 500 and d.distance_m(0, 0) == pytest.approx(0.5)
    assert d.raw_at(1, 0) == 0 and d.distance_m(1, 0) is None  # 0.0 m -> invalid
    assert d.raw_at(2, 0) == 1000
    assert d.raw_at(0, 1) == 0 and d.distance_m(0, 1) is None  # None -> invalid
    assert d.raw_at(1, 1) == 65535  # 70 m clips at the uint16 ceiling instead of wrapping
    assert d.raw_at(2, 1) == 250
    assert max(d.values()) == 65535
    with pytest.raises(IndexError):
        d.raw_at(3, 0)
    with pytest.raises(ValueError):
        DepthImage(2, 2, b"\x00" * 7)
    with pytest.raises(ValueError):
        DepthImage(1, 1, b"\x00\x00", scale_m=0)
    st = d.stats()
    assert st["valid_fraction"] == pytest.approx(4 / 6)
    assert st["min_m"] == pytest.approx(0.25)


def test_depth_png16_roundtrip_and_rejections(tmp_path):
    f = synthetic_rgbd(64, 48)
    png = f.depth.to_png16()
    back = DepthImage.from_png16(png, scale_m=f.depth.scale_m)
    assert back.data == f.depth.data
    p = tmp_path / "d.png"
    p.write_bytes(png)
    assert DepthImage.from_png16(p).data == f.depth.data
    # an 8-bit RGB PNG is not a depth PNG
    rgb = encode_png(2, 2, 3, bytes(12))
    with pytest.raises(ValueError, match="16-bit grayscale"):
        load_png16(rgb)
    with pytest.raises(ValueError, match="not a PNG"):
        load_png16(b"nope")


def test_encode_png_rgb_roundtrip_via_decoder(tmp_path):
    fr = Frame(3, 2, bytes(range(18)))
    p = tmp_path / "c.png"
    p.write_bytes(encode_png(3, 2, 3, fr.data))
    w, h, c, buf = load_png(str(p))
    assert (w, h, c) == (3, 2, 3) and buf == fr.data
    with pytest.raises(ValueError):
        encode_png(3, 2, 4, bytes(24))
    with pytest.raises(ValueError):
        encode_png(3, 2, 3, bytes(5))
    with pytest.raises(ValueError):
        encode_png(3, 2, 3, bytes(36), bit_depth=16)


# ----- frames ------------------------------------------------------------------------


def test_rgbd_frame_requires_matching_sizes_when_aligned():
    f = synthetic_rgbd(32, 24)
    small = DepthImage(16, 12, bytes(16 * 12 * 2))
    with pytest.raises(ValueError, match="aligned"):
        RgbdFrame(color=f.color, depth=small, intrinsics=f.intrinsics)
    RgbdFrame(color=f.color, depth=small, intrinsics=f.intrinsics, aligned=False)  # fine when unaligned


def test_synthetic_scene_geometry():
    f = synthetic_rgbd(320, 240)
    for cx, cy, _r, rgb, z in synthetic_disks(320, 240):
        assert f.color.pixel(cx, cy) == rgb
        assert f.depth.distance_m(cx, cy) == pytest.approx(z)
        pt = f.point_at(cx, cy)
        assert pt is not None and pt[2] == pytest.approx(z)
    assert f.depth.distance_m(0, 100) is None  # the invalid strip
    assert f.point_at(0, 100) is None
    assert f.summary()["depth"]["valid_fraction"] < 1.0


def test_pack_unpack_container():
    f = synthetic_rgbd(40, 30, frame_number=7)
    blob = pack_rgbd(f, seq=3, meta={"fps": 9.5})
    header, png, dz = unpack_rgbd(blob)
    assert header["seq"] == 3 and header["fps"] == 9.5 and header["frame_number"] == 7
    assert header["width"] == 40 and header["depth_scale_m"] == f.depth.scale_m
    w, h, c, buf = load_png_bytes(png)
    assert (w, h, c) == (40, 30, 3) and buf == f.color.data
    import zlib

    assert zlib.decompress(dz) == f.depth.data
    with pytest.raises(ValueError):
        unpack_rgbd(b"XXXX" + blob[4:])


def load_png_bytes(png: bytes):
    import tempfile

    with tempfile.NamedTemporaryFile(suffix=".png", delete=False) as t:
        t.write(png)
        name = t.name
    return load_png(name)
