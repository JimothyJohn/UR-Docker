"""The USB stick's PolyScope 5 magic file (scripts/urmagic_perceptronic.sh) and what
scripts/urcap5-usb.sh writes into it, run against a temporary robot: a `.urcaps` directory
and a fake Dashboard on a local port. No robot, no stick."""

from __future__ import annotations

import hashlib
import os
import re
import shutil
import socket
import subprocess
import sys
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
MAGIC = ROOT / "scripts" / "urmagic_perceptronic.sh"
USB = ROOT / "scripts" / "urcap5-usb.sh"
PROPS = ROOT / "urcap" / "perceptronic-ps5" / "bundle.properties"
PLACEHOLDERS = ("@URCAP_FILE@", "@URCAP_SHA256@", "@SYMBOLIC_NAME@")

pytestmark = pytest.mark.skipif(
    sys.platform == "win32" or shutil.which("bash") is None, reason="a bash that runs is needed"
)


def _symbolic_name() -> str:
    return re.search(r"^Bundle-SymbolicName=(.+)$", PROPS.read_text(), re.M).group(1).strip()


class FakeDashboard:
    """Answers like a PolyScope 5 Dashboard: banner on connect, one reply line per command."""

    def __init__(self, robotmode="Robotmode: POWER_OFF", program="STOPPED none"):
        self.robotmode, self.program = robotmode, program
        self.commands: list[str] = []
        self.sock = socket.socket()
        self.sock.bind(("127.0.0.1", 0))
        self.sock.listen(8)
        self.port = self.sock.getsockname()[1]
        self.thread = threading.Thread(target=self._serve, daemon=True)
        self.thread.start()

    def _serve(self):
        while True:
            try:
                conn, _ = self.sock.accept()
            except OSError:
                return
            with conn:
                conn.sendall(b"Connected: Universal Robots Dashboard Server\n")
                f = conn.makefile("rb")
                for raw in f:
                    cmd = raw.decode().strip()
                    self.commands.append(cmd)
                    if cmd == "quit":
                        break
                    if cmd == "robotmode":
                        reply = self.robotmode
                    elif cmd == "programState":
                        reply = self.program
                    elif cmd.startswith("popup"):
                        reply = "showing popup"
                    elif cmd.startswith("addToLog"):
                        reply = "Added log message"
                    else:
                        reply = "unknown"
                    conn.sendall((reply + "\n").encode())

    def close(self):
        self.sock.close()


def _stick(tmp_path: Path, payload: bytes = b"PK\x03\x04 not really a jar") -> tuple[Path, str, Path]:
    """A stick directory with the .urcap and the magic file rendered the way urcap5-usb.sh does."""
    stick = tmp_path / "stick"
    stick.mkdir()
    urcap = stick / "perceptronic-ps5-9.9.9.urcap"
    urcap.write_bytes(payload)
    sha = hashlib.sha256(payload).hexdigest()
    script = MAGIC.read_text()
    for key, val in zip(PLACEHOLDERS, (urcap.name, sha, _symbolic_name()), strict=True):
        assert key in script
        script = script.replace(key, val)
    magic = stick / "urmagic_perceptronic.sh"
    magic.write_text(script)
    magic.chmod(0o755)
    return stick, sha, magic


def _run(magic: Path, urcaps: Path, dash: FakeDashboard | None, *, restart="auto", env=None):
    e = {
        **os.environ,
        "URCAPS_DIR": str(urcaps),
        "DASHBOARD_HOST": "127.0.0.1",
        "DASHBOARD_PORT": str(dash.port if dash else 1),  # port 1: nothing listens
        "URMAGIC_RESTART": restart,
        "URMAGIC_REBOOT_CMD": "echo REBOOT-CALLED",
        **(env or {}),
    }
    return subprocess.run(["bash", str(magic)], capture_output=True, text=True, env=e, timeout=60)


def test_placeholders_are_what_the_stick_script_fills_in():
    usb = USB.read_text()
    for key in PLACEHOLDERS:
        assert key in MAGIC.read_text()
        assert key in usb, f"urcap5-usb.sh no longer substitutes {key}"
    assert "urmagic_perceptronic.sh" in usb


def test_fresh_install_copies_the_jar_and_restarts_a_powered_off_robot(tmp_path):
    dash = FakeDashboard()
    try:
        stick, sha, magic = _stick(tmp_path)
        urcaps = tmp_path / "root" / ".urcaps"
        r = _run(magic, urcaps, dash)
        assert r.returncode == 0, r.stdout + r.stderr
        dest = urcaps / f"{_symbolic_name()}.jar"
        assert dest.is_file() and hashlib.sha256(dest.read_bytes()).hexdigest() == sha
        assert not (urcaps / f"{_symbolic_name()}.jar.tmp").exists()
        assert "REBOOT-CALLED" in r.stdout
        assert any(c.startswith("addToLog") for c in dash.commands)
        assert not any(c.startswith("popup") for c in dash.commands), "no popup when it restarts itself"
        log = (stick / "urmagic_perceptronic.log").read_text()
        assert "installed" in log and "restarting" in log
    finally:
        dash.close()


def test_second_run_is_a_no_op(tmp_path):
    dash = FakeDashboard()
    try:
        _, _, magic = _stick(tmp_path)
        urcaps = tmp_path / ".urcaps"
        assert _run(magic, urcaps, dash).returncode == 0
        dash.commands.clear()
        r = _run(magic, urcaps, dash)
        assert r.returncode == 0
        assert "already installed" in r.stdout
        assert "REBOOT-CALLED" not in r.stdout
        assert dash.commands == [], "an installed stick left in must not touch the Dashboard"
    finally:
        dash.close()


@pytest.mark.parametrize(
    "robotmode,program",
    [("Robotmode: RUNNING", "STOPPED none"), ("Robotmode: POWER_OFF", "PLAYING cycle.urp")],
)
def test_a_busy_robot_is_told_not_restarted(tmp_path, robotmode, program):
    dash = FakeDashboard(robotmode, program)
    try:
        _, _, magic = _stick(tmp_path)
        r = _run(magic, tmp_path / ".urcaps", dash)
        assert r.returncode == 0
        assert (tmp_path / ".urcaps" / f"{_symbolic_name()}.jar").is_file()
        assert "REBOOT-CALLED" not in r.stdout
        assert any(c.startswith("popup") and "Restart" in c for c in dash.commands)
    finally:
        dash.close()


def test_restart_modes(tmp_path):
    busy = FakeDashboard("Robotmode: RUNNING")
    try:
        _, _, magic = _stick(tmp_path)
        r = _run(magic, tmp_path / "a", busy, restart="always")
        assert r.returncode == 0 and "REBOOT-CALLED" in r.stdout
    finally:
        busy.close()
    idle = FakeDashboard()
    try:
        r = _run(magic, tmp_path / "b", idle, restart="never")
        assert r.returncode == 0 and "REBOOT-CALLED" not in r.stdout
        assert any(c.startswith("popup") for c in idle.commands)
    finally:
        idle.close()


def test_a_damaged_file_installs_nothing(tmp_path):
    dash = FakeDashboard()
    try:
        stick, _, magic = _stick(tmp_path)
        (stick / "perceptronic-ps5-9.9.9.urcap").write_bytes(b"truncated")
        r = _run(magic, tmp_path / ".urcaps", dash)
        assert r.returncode == 1
        assert not (tmp_path / ".urcaps").exists()
        assert "REBOOT-CALLED" not in r.stdout
        assert any(c.startswith("popup") and "damaged" in c for c in dash.commands)
    finally:
        dash.close()


def test_missing_urcap_next_to_the_script(tmp_path):
    stick, _, magic = _stick(tmp_path)
    (stick / "perceptronic-ps5-9.9.9.urcap").unlink()
    r = _run(magic, tmp_path / ".urcaps", None)
    assert r.returncode == 1 and not (tmp_path / ".urcaps").exists()


def test_without_a_dashboard_it_still_installs_but_never_reboots(tmp_path):
    _, sha, magic = _stick(tmp_path)
    r = _run(magic, tmp_path / ".urcaps", None)
    assert r.returncode == 0, r.stdout + r.stderr
    dest = tmp_path / ".urcaps" / f"{_symbolic_name()}.jar"
    assert hashlib.sha256(dest.read_bytes()).hexdigest() == sha
    assert "REBOOT-CALLED" not in r.stdout
    assert "no Dashboard" in r.stdout


def test_unwritable_urcaps_dir_fails_cleanly(tmp_path):
    if os.geteuid() == 0:
        pytest.skip("root can write anywhere")
    _, _, magic = _stick(tmp_path)
    urcaps = tmp_path / "ro"
    urcaps.mkdir()
    urcaps.chmod(0o500)
    try:
        r = _run(magic, urcaps, None)
        assert r.returncode == 1
        assert not list(urcaps.iterdir())
    finally:
        urcaps.chmod(0o700)


def test_shellcheck_clean():
    if shutil.which("shellcheck") is None:
        pytest.skip("shellcheck not installed")
    for script in (MAGIC, USB, ROOT / "scripts" / "urcapx-autoinstall.sh"):
        subprocess.run(["shellcheck", str(script)], check=True)
