"""urcap/psx_matrix.py — the PolyScope X release list and how it reads the e2e: the
release rule against Docker Hub's tag shapes, drift, and the e2e's check lines."""

from __future__ import annotations

import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "urcap"))
import psx_matrix  # noqa: E402

HUB = [
    "latest",
    "10.14.0",
    "10.14.0-0.10.777",
    "0.19.135",
    "10.14.0-0.10.703-preview-1",
    "10.13.0",
    "10.13.0-preview-0",
    "10.12.1",
    "10.12.0",
    "10.12.0-preview",
    "10.11.0",
    "10.10.0",
    "10.10.0-beta",
    "10.9.0",
    "10.8.0",
    "10.7.0",
    "10.6.0",
    "10.5.0",
    "10.4.0",
    "0.9.68",
]


def test_releases_are_distinct_release_tags_newest_first_from_the_floor():
    keys = [psx_matrix.release_key(t) for t in psx_matrix.RELEASES]
    assert 0 < len(psx_matrix.RELEASES) <= psx_matrix.KEEP and all(keys)
    assert keys == sorted(set(keys), reverse=True)
    assert keys[-1] == psx_matrix.release_key(psx_matrix.FLOOR)


def test_only_plain_versions_are_releases():
    assert psx_matrix.release_key("10.12.1") == (10, 12, 1)
    for t in ("latest", "10.14.0-0.10.777", "10.13.0-preview-0", "10.10.0-beta", "0.19.135"):
        assert psx_matrix.release_key(t) is None, t


def test_newest_releases_from_docker_hubs_tags_is_the_list():
    assert psx_matrix.newest_releases(HUB) == psx_matrix.RELEASES
    assert psx_matrix.newest_releases(HUB, keep=3) == ["10.14.0", "10.13.0", "10.12.1"]
    assert psx_matrix.newest_releases(HUB, floor="10.6.0") == psx_matrix.RELEASES + ["10.7.0", "10.6.0"]
    assert "10.7.0" not in psx_matrix.newest_releases(HUB)  # below the floor, however many are kept


def test_drift_names_a_newer_release_and_a_vanished_one():
    assert psx_matrix.tag_drift(HUB, psx_matrix.RELEASES) == []
    newer = psx_matrix.tag_drift(["10.15.0", *HUB], psx_matrix.RELEASES)
    assert newer == ["PolyScope X 10.15.0 exists on Docker Hub: add it (newest first) and drop the oldest"]
    gone = psx_matrix.tag_drift([t for t in HUB if t != "10.9.0"], psx_matrix.RELEASES)
    assert gone == ["10.9.0 is not on Docker Hub any more"]
    # an older release that fell off the list is not drift
    assert psx_matrix.tag_drift(HUB, psx_matrix.RELEASES[:-1]) == []


def test_the_e2es_check_lines_are_read_back():
    text = (
        "noise\n  ok    feed live — live from http://x\n  FAIL  pick node ready — add a picture point\n"
        + "e2e: FAILED\n"
    )
    got = psx_matrix._checks(text)
    assert got == [
        {"ok": True, "name": "feed live", "detail": "live from http://x"},
        {"ok": False, "name": "pick node ready", "detail": "add a picture point"},
    ]
    md = psx_matrix.summary_markdown(
        [{"tag": "10.13.0", "image": "i", "passed": False, "seconds": 61.2, "checks": got}]
    )
    assert "| 10.13.0 | `i` | FAIL (pick node ready) | 61 s |" in md


def test_list_prints_the_releases_as_json(capsys):
    assert psx_matrix.main(["list"]) == 0
    assert json.loads(capsys.readouterr().out) == psx_matrix.RELEASES


def test_run_refuses_a_release_that_is_not_listed(capsys):
    assert psx_matrix.main(["run", "--version", "9.9.9"]) == 2
    assert "not in RELEASES" in capsys.readouterr().err


@pytest.mark.parametrize("keep", [1, 5, 8])
def test_keep_is_honoured(keep):
    assert len(psx_matrix.newest_releases(HUB, keep=keep)) == keep
