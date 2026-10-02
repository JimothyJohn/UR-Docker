"""The Windows Docker Desktop deployment's contract, checked without Docker or
Windows: the compose command must parse under the real ``perceptronics-gui`` CLI,
the unauthenticated ports must stay on loopback unless asked, captures must land
on the image's volume, the example cell must be one the cell loader accepts and
the code reads, and the launcher must stay loadable by Windows PowerShell 5.1."""

from __future__ import annotations

import json
import re
import shlex
from pathlib import Path

from perceptronics import webapp
from perceptronics.cell import ALLOWED_PREFIXES, parse_env_text
from perceptronics.picknode import DEFAULT_PICK_PORT

ROOT = Path(__file__).resolve().parents[1]
COMPOSE = ROOT / "docker-compose.windows.yml"
DOCKERFILE = ROOT / "Dockerfile.perceptronics"
SCRIPT = ROOT / "scripts" / "docker-windows.ps1"
CELL_EXAMPLE = ROOT / "deploy" / "windows" / "cell.env.example"
EXTRA_ARGS = "${COCKPIT_ARGS:-}"


def _compose() -> str:
    return COMPOSE.read_text(encoding="utf-8")


def _command(extra: str = "") -> list[str]:
    line = re.search(r"^\s*command:\s*(.+)$", _compose(), re.M).group(1)
    assert EXTRA_ARGS in line, "the launcher's flags reach the cockpit through COCKPIT_ARGS"
    return shlex.split(line.replace(EXTRA_ARGS, extra))


def _serve_with(monkeypatch, argv: list[str]) -> dict:
    monkeypatch.delenv("UR_CELL", raising=False)
    served: dict = {}
    robots: list = []
    monkeypatch.setattr(webapp, "camera_from_args", lambda args, config: None)
    monkeypatch.setattr(webapp, "robot_from_args", lambda args: robots.append(args) or None)
    monkeypatch.setattr(webapp, "views_from_args", lambda args, config: [])
    monkeypatch.setattr(webapp, "serve", lambda *a, **kw: served.update(kw))
    assert webapp.main(argv) == 0  # argparse exits 2 on an unknown flag
    served["robot_args"] = robots[0]
    return served


def test_command_parses_under_the_real_cli(monkeypatch):
    cmd = _command()
    assert cmd[0] == "perceptronics-gui"
    served = _serve_with(monkeypatch, cmd[1:])
    assert served["bind"] == "0.0.0.0"  # inside the container; the host side is the ports: line
    assert served["open_browser"] is False
    assert (served["port"], served["pick_port"]) == (webapp.DEFAULT_PORT, DEFAULT_PICK_PORT)


def test_the_launchers_flags_are_real_cockpit_flags(monkeypatch):
    # every flag the script can put in COCKPIT_ARGS
    flags = set(re.findall(r'\$cockpitArgs \+= "(--[a-z-]+)"', SCRIPT.read_text(encoding="utf-8")))
    assert flags == {"--no-robot", "--robot-dry-run"}
    served = _serve_with(monkeypatch, _command(" ".join(sorted(flags)))[1:])
    assert served["robot_args"].no_robot and served["robot_args"].robot_dry_run


def test_ports_are_loopback_unless_asked():
    ports = re.findall(r'^\s*-\s*"([^"]+:\d+:\d+)"\s*$', _compose(), re.M)
    assert sorted(ports) == [
        f"${{PERCEPTRONICS_PUBLISH:-127.0.0.1}}:{p}:{p}" for p in (webapp.DEFAULT_PORT, DEFAULT_PICK_PORT)
    ], "the cockpit and the pick server have no auth: loopback by default, both published"
    script = SCRIPT.read_text(encoding="utf-8")
    assert 'if ($Lan) { $env:PERCEPTRONICS_PUBLISH = "0.0.0.0" }' in script
    assert "-RemoteAddress LocalSubnet" in script, "the firewall rule admits the local subnet only"


def test_captures_and_cells_mounts():
    volume = json.loads(re.search(r"^VOLUME\s+(.+)$", DOCKERFILE.read_text(encoding="utf-8"), re.M).group(1))
    text = _compose()
    assert re.findall(r"^\s*-\s*\./captures:(\S+)\s*$", text, re.M) == volume
    assert re.search(r"^\s*-\s*/dev/bus/usb:/dev/bus/usb\s*$", text, re.M) and "privileged: true" in text
    # the script points UR_CELL at /cells/<name>.env; the mount must put deploy/windows there
    assert re.search(r"^\s*-\s*\./deploy/windows:/cells:ro\s*$", text, re.M)
    assert '$env:UR_CELL = "/cells/$Cell.env"' in SCRIPT.read_text(encoding="utf-8")


def test_environment_is_passed_through_never_defaulted():
    # `- NAME` (no `=`): unset in the shell means unset in the container, so the cell
    # file's value applies; `NAME=${NAME:-}` would too, but only because cell.py treats
    # empty as unset - keep the compose file independent of that.
    block = re.search(r"^    environment:\n((?:      .*\n)+)", _compose(), re.M).group(1)
    names = re.findall(r"^\s*-\s*(\S+)\s*$", block, re.M)
    passed = [n for n in names if "=" not in n]
    assert {"UR_CELL", "UR_HOST", "PERCEPTRONICS_FAKE"} <= set(passed)
    assert [n for n in names if "=" in n] == ["PYTHONUNBUFFERED=1"]
    script = SCRIPT.read_text(encoding="utf-8")
    for name in ("UR_CELL", "UR_HOST", "PERCEPTRONICS_FAKE", "PERCEPTRONICS_PUBLISH"):
        assert f"$env:{name} = $null" in script, f"the script must clear {name} before setting it"


def test_example_cell_is_loadable_and_only_names_what_the_code_reads():
    values = parse_env_text(CELL_EXAMPLE.read_text(encoding="utf-8"))
    assert all(k.startswith(ALLOWED_PREFIXES) for k in values)
    assert values["UR_HOST"] == "", "the example ships without anybody's robot address"
    assert "PERCEPTRONICS_T_FLANGE_CAMERA" not in values, (
        "a copied hand-eye would win over the user's own solve"
    )
    assert "PERCEPTRONICS_VIEWS" not in values, "the image has no ffmpeg"
    # cell.py only lists the names for /api/info; "read" means some other module uses it
    source = "\n".join(
        p.read_text(encoding="utf-8")
        for d in ("perceptronics", "urctl")
        for p in (ROOT / d).glob("*.py")
        if p.name != "cell.py"
    )
    unread = [k for k in values if f'"{k}"' not in source]
    assert not unread, f"{unread} are read by nothing in perceptronics/ or urctl/"
    ignore = (CELL_EXAMPLE.parent / ".gitignore").read_text(encoding="utf-8").split()
    assert "*.env" in ignore


def test_script_is_ascii_for_windows_powershell():
    # Windows PowerShell 5.1 reads a BOM-less file in the ANSI code page: one em-dash in
    # a string and the parse fails on the coworker's machine, not here.
    raw = SCRIPT.read_bytes()
    assert raw.isascii()
    assert not raw.startswith(b"\xef\xbb\xbf")
    text = raw.decode("ascii")
    for pwsh7_only in ("??", "?.", " && ", " || "):
        assert pwsh7_only not in text, f"{pwsh7_only!r} is PowerShell 7 syntax"
    assert f"-f {COMPOSE.name}" in text.replace("$composeFile", COMPOSE.name)
