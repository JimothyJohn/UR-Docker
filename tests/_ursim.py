"""URSim test helpers — importable from both conftest.py and test modules.

conftest.py is auto-loaded by pytest but isn't an importable module from
test files; anything tests want to share with conftest lives here.
"""

from __future__ import annotations

import os
import socket
import subprocess
import sys
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SCRIPTS_DIR = REPO_ROOT / "scripts"
PROGRAMS_DIR = REPO_ROOT / "programs"

# Make scripts/urp_convert.py importable as a regular module.
if str(SCRIPTS_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPTS_DIR))

URSIM_HOST = os.environ.get("UR_HOST", "localhost")
URSIM_DASH_PORT = int(os.environ.get("UR_DASH_PORT", "29999"))
URSIM_PRIMARY_PORT = int(os.environ.get("UR_PRIMARY_PORT", "30001"))
URSIM_CONTAINER = os.environ.get("URSIM_CONTAINER", "ur-docker-ursim-1")
DOCKER = os.environ.get("DOCKER", "sudo docker").split()


def tcp_open(host: str, port: int, timeout: float = 1.0) -> bool:
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def dash(*commands: str, timeout: float = 8.0) -> str:
    """Send commands to the Dashboard server and return the joined response.

    Uses a single TCP session, terminating with `quit`. Strips the
    `Connected:`/`Disconnected` framing lines so callers can assert on the
    actual server replies.
    """
    payload = ("\n".join(commands) + "\nquit\n").encode("utf-8")
    with socket.create_connection((URSIM_HOST, URSIM_DASH_PORT), timeout=timeout) as s:
        s.sendall(payload)
        s.settimeout(timeout)
        chunks: list[bytes] = []
        while True:
            try:
                chunk = s.recv(4096)
            except TimeoutError:
                break
            if not chunk:
                break
            chunks.append(chunk)
    text = b"".join(chunks).decode("utf-8", errors="replace")
    lines = [ln for ln in text.splitlines() if ln and not ln.startswith(("Connected:", "Disconnected"))]
    return "\n".join(lines)


def wait_for_dash(predicate, *, timeout: float = 60.0, interval: float = 1.0) -> str:
    """Poll the Dashboard server until `predicate(reply)` returns truthy."""
    deadline = time.monotonic() + timeout
    last = ""
    while time.monotonic() < deadline:
        try:
            last = dash("robotmode", "safetymode")
        except (TimeoutError, OSError):
            last = ""
        if predicate(last):
            return last
        time.sleep(interval)
    raise TimeoutError(f"predicate not satisfied within {timeout}s; last reply:\n{last}")


def docker_available() -> bool:
    """True if `docker` and the URSim container are usable from this host."""
    try:
        r = subprocess.run(
            [*DOCKER, "ps", "--format", "{{.Names}}"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0 and URSIM_CONTAINER in r.stdout


def docker_cp_to_container(local_path: Path, container_path: str) -> None:
    subprocess.run(
        [*DOCKER, "cp", str(local_path), f"{URSIM_CONTAINER}:{container_path}"],
        check=True,
        capture_output=True,
    )


def docker_exec(*cmd: str, check: bool = True) -> subprocess.CompletedProcess:
    return subprocess.run(
        [*DOCKER, "exec", URSIM_CONTAINER, *cmd],
        check=check,
        capture_output=True,
        text=True,
    )
