"""Capture store: dataset layout, resumable numbering, name hardening, reload."""

from __future__ import annotations

import json

import pytest

from perception.capture import CaptureStore, validate_name
from perception.rgbd import synthetic_rgbd
from perception.segment import StubSegmenter, extract_features


@pytest.mark.parametrize(
    "bad",
    ["", "../x", "..", ".hidden", "a/b", "a\\b", "x" * 65, "sp ace", "ünï", "a\x00b", "a\nb", "name;rm -rf"],
)
def test_names_are_hardened(bad):
    with pytest.raises(ValueError):
        validate_name(bad)


def test_names_allowed():
    for ok in ("apple", "red-apple_2", "a.b", "X" * 64, "1"):
        assert validate_name(ok) == ok


def test_save_load_and_resume(tmp_path):
    store = CaptureStore(tmp_path / "caps")
    f = synthetic_rgbd(64, 48, frame_number=9)
    m = StubSegmenter().segment(f, (16, 24))
    feats = extract_features(m, f).as_dict()
    r1 = store.save("apple", f, mask=m, features=feats, device={"serial": "S1"}, extra={"note": "x"})
    assert r1["index"] == 1 and set(r1["files"]) == {"color", "depth", "mask", "meta"}
    r2 = store.save("apple", f)
    assert r2["index"] == 2 and "mask" not in r2["files"]
    # numbering resumes across store instances (like the original rst/main.py)
    assert CaptureStore(tmp_path / "caps").next_index("apple") == 3
    assert store.list() == [{"name": "apple", "count": 2}]

    back, mask, meta = store.load("apple", 1)
    assert back.color.data == f.color.data and back.depth.data == f.depth.data
    assert back.intrinsics == f.intrinsics and back.frame_number == 9
    assert mask is not None and mask.area == m.area and mask.data == m.data
    assert (
        meta["features"]["area_px"] == m.area
        and meta["device"]["serial"] == "S1"
        and meta["extra"] == {"note": "x"}
    )
    meta_on_disk = json.loads((tmp_path / "caps" / "apple" / "meta_00001.json").read_text())
    assert meta_on_disk["files"]["depth"] == "depth_00001.png"
    _b, none_mask, _m = store.load("apple", 2)
    assert none_mask is None
    with pytest.raises(ValueError):
        store.save("../apple", f)
    assert CaptureStore(tmp_path / "nowhere").list() == []
