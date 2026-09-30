"""The perceptronics container's contract, checked without Docker: the image's CMD
must parse under the real ``perceptronics-gui`` CLI (a stale flag crashes the
container on start — ``--out`` outlived the capture store that owned it), and
the captures volume must sit where the cockpit's relative ``captures/`` paths
resolve, i.e. under WORKDIR — or snapshots and the hand-eye file die with the
container."""

from __future__ import annotations

import json
import re
from pathlib import Path, PurePosixPath

import pytest

from perceptronics import handeye, webapp

ROOT = Path(__file__).resolve().parents[1]
DOCKERFILE = ROOT / "Dockerfile.perceptronics"
COMPOSE = ROOT / "docker-compose.yml"


def _final_stage() -> list[str]:
    lines = DOCKERFILE.read_text(encoding="utf-8").splitlines()
    starts = [i for i, ln in enumerate(lines) if re.match(r"\s*FROM\s", ln, re.I)]
    return lines[starts[-1] :]


def _instruction(name: str) -> str:
    found = [ln.split(None, 1)[1] for ln in _final_stage() if re.match(rf"\s*{name}\s", ln, re.I)]
    assert found, f"no {name} in the final stage of {DOCKERFILE.name}"
    return found[-1].strip()


def test_cmd_parses_under_the_real_cli(monkeypatch):
    cmd = json.loads(_instruction("CMD"))
    assert cmd[0] == "perceptronics-gui"
    monkeypatch.delenv("UR_CELL", raising=False)
    served = {}
    monkeypatch.setattr(webapp, "camera_from_args", lambda args, config: None)
    monkeypatch.setattr(webapp, "robot_from_args", lambda args: None)
    monkeypatch.setattr(webapp, "views_from_args", lambda args, config: [])
    monkeypatch.setattr(webapp, "serve", lambda *a, **kw: served.update(kw))
    assert webapp.main(cmd[1:]) == 0  # argparse exits 2 on an unknown flag
    assert served["bind"] == "0.0.0.0"
    assert served["open_browser"] is False


@pytest.mark.parametrize("relative", [webapp.DEFAULT_SNAPSHOT_DIR, handeye.DEFAULT_HANDEYE_DIR])
def test_captures_volume_is_where_relative_paths_land(relative):
    workdir = PurePosixPath(_instruction("WORKDIR"))
    volumes = json.loads(_instruction("VOLUME"))
    landed = workdir / relative
    assert any(landed.is_relative_to(v) for v in volumes), f"{landed} is outside VOLUME {volumes}"


def test_compose_mounts_host_captures_on_the_volume():
    volumes = json.loads(_instruction("VOLUME"))
    text = COMPOSE.read_text(encoding="utf-8")
    mounts = re.findall(r"^\s*-\s*\./captures:(\S+)\s*$", text, re.M)
    assert mounts, "no ./captures bind mount in docker-compose.yml"
    assert set(mounts) <= set(volumes), f"compose mounts {mounts}, image volume is {volumes}"
