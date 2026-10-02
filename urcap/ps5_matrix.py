#!/usr/bin/env python3
"""The PolyScope 5 URCap on every supported PolyScope 5 release — stdlib + Docker.

For each version in :data:`MATRIX` (the newest image of **every** PolyScope 5 minor on
Docker Hub, oldest first — nothing is ever dropped), against
``docker-compose.ps5-matrix.yml`` (generated from MATRIX: ``ps5_matrix.py compose``):

1. pull the image and hold the committed URCap to its URCap API (``urcap5.py check``:
   the sources it loads compile against the jars in this image, and every package the jar
   imports is one this PolyScope exports);
2. stage the committed ``urcap/dist/perceptronic-ps5-<ver>.urcap`` as
   ``/urcaps/perceptronic-ps5.jar`` (the image's entrypoint copies ``/urcaps/*.jar``
   into PolyScope's bundle dir at every start) and bring the version's service up;
3. wait for the Dashboard to answer a ``robotmode`` other than ``NO_CONTROLLER``;
4. wait for Felix (PolyScope's OSGi framework; its remote shell on the container's
   port 6666) to report the bundle ``Active`` with both node services registered,
   failing on any polyscope.log exception that names the URCap's package;
5. power on + brake release, then run ``urcap/pick5_e2e.py`` — the Pick node's own
   URScript on this controller, against a pick server on this machine that the container
   reaches at its network's gateway, and the full script (Robotiq + popup) compiled by the
   controller;
6. on failure copy ``polyscope.log`` / ``URControl.log`` and the evidence out, and always
   ``docker compose down -v``.

One JSON summary per version (``--report`` writes them all).

    python3 urcap/ps5_matrix.py run --version 5.26 --artifacts ps5-artifacts
    python3 urcap/ps5_matrix.py run --version all
    python3 urcap/ps5_matrix.py down --version all
    python3 urcap/ps5_matrix.py list                  # the minors, as JSON (CI's matrix)
    python3 urcap/ps5_matrix.py ports                 # the host-port table
    python3 urcap/ps5_matrix.py compose > docker-compose.ps5-matrix.yml
    python3 urcap/ps5_matrix.py check-tags            # Docker Hub vs MATRIX; exit 1 on drift
"""

from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import socket
import subprocess
import sys
import time
import urllib.request
from dataclasses import asdict, dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
COMPOSE_FILE = REPO / "docker-compose.ps5-matrix.yml"
DIST = REPO / "urcap" / "dist"
SRC = REPO / "urcap" / "perceptronic-ps5"
STAGE = REPO / "target" / "ps5-matrix" / "urcaps"
SDK = REPO / "target" / "ps5-matrix" / "sdk"
JAR_NAME = "perceptronic-ps5.jar"
IMAGE = "universalrobots/ursim_e-series"
HUB_TAGS = f"https://hub.docker.com/v2/repositories/{IMAGE}/tags?page_size=100"
PACKAGE = "io.advin.perceptronic"
# `sudo docker` where the user is not in the docker group (the Makefile passes its DOCKER).
DOCKER = os.environ.get("DOCKER", "docker").split()

# -- host ports -------------------------------------------------------------------------------
#
# Every version publishes on its own block of 100 host ports, 20000 + 100 × minor, and inside
# the block each service keeps the last two digits of its usual port: Dashboard 29999 → 99,
# Primary 30001 → 01, RTDE 30004 → 04, noVNC 6080 → 80, and the pick server the e2e runs on
# the host (7622, the cockpit's) → 22. PolyScope 5.26: 22699 / 22601 / 22604 / 22680 / 22622.
# Blocks never overlap; minors 0..98 stay in 20000..29899 — clear of the default sims
# (docker-compose.yml: 29999, 30000-30004, 30020, 502, 6080, 15900, 8000, 31001, 31004), the
# cockpit's 7621/7622/7623/7631, and the Linux (32768+) and macOS (49152+) ephemeral ranges.

PORT_BASE = 20000
SLOTS = {"dashboard": 99, "primary": 1, "rtde": 4, "novnc": 80, "pick": 22}
MAX_MINOR = 98


@dataclass(frozen=True)
class Version:
    """One matrix entry: the image tag and the host ports its service publishes."""

    tag: str  # the image tag: the newest patch of its minor (the bare minor where UR tagged no patch)
    dashboard: int
    primary: int
    rtde: int
    novnc: int
    pick: int  # the pick server's port on the host for this version's run

    @property
    def minor(self) -> str:
        return ".".join(self.tag.split(".")[:2])

    @property
    def service(self) -> str:
        return "ursim-" + self.minor.replace(".", "-")

    @property
    def project(self) -> str:
        return "ur-ps5-matrix-" + self.minor.replace(".", "")


def _version(tag: str) -> Version:
    minor = int(tag.split(".")[1])
    if not 0 <= minor <= MAX_MINOR:
        raise ValueError(f"PolyScope 5.{minor}: the port scheme covers minors 0..{MAX_MINOR}")
    base = PORT_BASE + 100 * minor
    return Version(tag, **{name: base + slot for name, slot in SLOTS.items()})


# The newest image of every PolyScope 5 minor on Docker Hub (2026-09-28), oldest first. 5.4-5.8
# carry only the bare minor tag (their VERSION env: 5.4.3, 5.5.1, 5.6.0, 5.7.0, 5.8.2). The source
# of truth for docker-compose.ps5-matrix.yml and the CI matrix; `check-tags` only ever adds.
TAGS = (
    "5.4",
    "5.5",
    "5.6",
    "5.7",
    "5.8",
    "5.9.4",
    "5.10.2",
    "5.11.11",
    "5.12.8",
    "5.13.1",
    "5.14.6",
    "5.15.2",
    "5.16.1",
    "5.17.3",
    "5.18.1",
    "5.19.0",
    "5.20.0",
    "5.21.3",
    "5.22.2",
    "5.23.0",
    "5.24.0",
    "5.25.2",
    "5.26.1",
)

# What each image carries: its ``VERSION`` environment variable, read from the registry
# (``urcap5.image_version``, 2026-09-29). A bare tag (5.4 - 5.8) says nothing about its patch,
# so ``check-tags`` compares these too: a tag re-pushed with other contents is drift, not news.
IMAGE_VERSIONS: dict[str, str] = {
    "5.4": "5.4.3",
    "5.5": "5.5.1",
    "5.6": "5.6.0",
    "5.7": "5.7.0",
    "5.8": "5.8.2",
    "5.9.4": "5.9.4",
    "5.10.2": "5.10.2",
    "5.11.11": "5.11.11",
    "5.12.8": "5.12.8",
    "5.13.1": "5.13.1",
    "5.14.6": "5.14.6",
    "5.15.2": "5.15.2",
    "5.16.1": "5.16.1",
    "5.17.3": "5.17.3",
    "5.18.1": "5.18.1",
    "5.19.0": "5.19.0",
    "5.20.0": "5.20.0",
    "5.21.3": "5.21.3",
    "5.22.2": "5.22.2",
    "5.23.0": "5.23.0",
    "5.24.0": "5.24.0",
    "5.25.2": "5.25.2",
    "5.26.1": "5.26.1",
}
MATRIX: dict[str, Version] = {v.minor: v for v in map(_version, TAGS)}

# Minors on Docker Hub the URCap does not support, each with the specific reason (README table).
EXCLUDED: dict[str, str] = {}

# Every host port docker-compose.yml (the default dev sims) and the cockpit bind.
DEFAULT_PORTS = frozenset(
    {15900, 6080, 29999, 30000, 30001, 30002, 30003, 30004, 30020, 502, 8000, 31001, 31004}
    | {7621, 7622, 7623, 7631}  # cockpit, its pick server, pick5_e2e's default, the sidecar
)


def host_ports(v: Version) -> list[int]:
    return [v.dashboard, v.primary, v.rtde, v.novnc, v.pick]


# -- docker-compose.ps5-matrix.yml, generated -------------------------------------------------

# The healthcheck (python3 is in the image; nc is not): the Dashboard answers a robotmode.
HEALTH = (
    "import socket,sys,time; s=socket.create_connection(('localhost',29999),3); "
    "s.sendall(('robotmode'+chr(10)+'quit'+chr(10)).encode()); time.sleep(0.3); "
    "d=s.recv(256).decode(); sys.exit(0 if 'Robotmode:' in d and 'NO_CONTROLLER' not in d else 1)"
)

COMPOSE_HEAD = """\
# GENERATED by `python3 urcap/ps5_matrix.py compose` from urcap/ps5_matrix.py's MATRIX — edit
# that, then regenerate (tests/test_ps5_matrix.py holds this file to it).
#
# The PolyScope 5 (e-Series) URCap's version matrix: one URSim per PolyScope 5 minor on
# Docker Hub (the newest image of each, oldest first), for urcap/ps5_matrix.py and
# .github/workflows/urcap5-matrix.yml. Separate from docker-compose.yml so the default
# dev flow (`make sim-up`, the `ur-docker` project) is untouched; every version publishes on
# its own block of host ports, so any of them and the default sim can run side by side.
#
#   python3 urcap/ps5_matrix.py run --version 5.26     # API check, up, URCap, pick e2e, down -v
#   python3 urcap/ps5_matrix.py run --version all
#   docker compose -f docker-compose.ps5-matrix.yml up -d ursim-5-26   # by hand
#
# Host ports: 20000 + 100 x minor + the last two digits of the usual port (Dashboard 29999
# -> 99, Primary 30001 -> 01, RTDE 30004 -> 04, noVNC 6080 -> 80; the e2e's pick server on
# the host, 7622 -> 22). Minors 0..98 stay inside 20000..29899, clear of the default sims
# (docker-compose.yml: 29999, 30000-30004, 30020, 502, 6080, 15900, 8000, 31001, 31004),
# the cockpit (7621/7622/7623/7631) and the Linux/macOS ephemeral ranges.
#
{table}
#
# Can one container host several versions on different ports? No, not sensibly. Read
# from the images' own config and layers (registry API, 2026-09-28; every tag from 5.4 to
# 5.26.1 carries the same /entrypoint.sh): each image is ONE PolyScope + URControl install
# rooted at /ursim (HOME=/ursim, JAVA_HOME=/usr/lib/jvm/jdk1.8.0_371, VERSION=<version>),
# started by /entrypoint.sh on a fixed X display (`Xvfb :1`, /tmp/.X1-lock removed at start),
# x11vnc on 5900, noVNC on 6080, a single runit service dir (/etc/service/runsvdir*),
# /ursim/start-ursim.sh <MODEL>, and fixed log paths /ursim/polyscope.log and
# /ursim/URControl.log. The controller's ports (29999, 30001-30004, 30020, 502, 50001-3)
# are URControl's own, not configurable per install. Several versions in one container
# would mean several /ursim trees, X displays and network namespaces — i.e. containers.
# Docker's port mapping is the supported way to offset them.
#
# URCaps: the entrypoint runs `cp -r /urcaps/*.jar /ursim/GUI/bundle/` at every start, so
# /urcaps must hold *.jar files. The committed urcap/dist/perceptronic-ps5-<ver>.urcap
# is a jar under another suffix; ps5_matrix.py stages it as
# target/ps5-matrix/urcaps/perceptronic-ps5.jar (PS5_URCAPS_DIR overrides) — that
# keeps this file free of the version number.
#
# The images declare no VOLUME, but teardown is `docker compose down -v` anyway (the
# PolyScope X sim leaked ~9 GB of anonymous volumes per run without -v).
name: ur-ps5-matrix

x-ursim: &ursim
  stdin_open: true
  tty: true
  environment:
    # UR3 -> the entrypoint picks UR3e where /ursim/programs.UR3e exists (5.4's image has
    # only programs.UR3, 5.26's programs.UR3e): the arm the node ships to, either way.
    - ROBOT_MODEL=${PS5_ROBOT_MODEL:-UR3}
  # Same reasons as docker-compose.yml's ursim: URControl's socket() gets ENOSYS under
  # Docker's default seccomp profile on modern kernels; Modbus 502 needs NET_BIND_SERVICE.
  security_opt:
    - seccomp:unconfined
  cap_add:
    - NET_BIND_SERVICE
  volumes:
    - ${PS5_URCAPS_DIR:-./target/ps5-matrix/urcaps}:/urcaps:ro
  healthcheck:
    test:
      [
        "CMD",
        "python3",
        "-c",
        "{health}",
      ]
    interval: 15s
    timeout: 10s
    retries: 20
    start_period: 90s

services:
"""


def compose_table(versions=None) -> str:
    rows = [("service", "image tag", "Dashboard", "Primary", "RTDE", "noVNC", "pick server (host)")]
    for v in versions or MATRIX.values():
        rows.append((v.service, v.tag, *map(str, (v.dashboard, v.primary, v.rtde, v.novnc, v.pick))))
    widths = [max(len(r[i]) for r in rows) for i in range(len(rows[0]))]
    return "\n".join(
        "#   " + "  ".join(c.ljust(w) for c, w in zip(r, widths, strict=True)).rstrip() for r in rows
    )


def render_compose(versions=None) -> str:
    """docker-compose.ps5-matrix.yml, exactly — the committed file must equal this."""
    versions = list(versions or MATRIX.values())
    out = [COMPOSE_HEAD.replace("{table}", compose_table(versions)).replace("{health}", HEALTH)]
    for i, v in enumerate(versions):
        out.append(
            f"  {v.service}:\n"
            "    <<: *ursim\n"
            f"    image: {IMAGE}:{v.tag}\n"
            "    ports:\n"
            f'      - "{v.dashboard}:29999" # Dashboard\n'
            f'      - "{v.primary}:30001" # Primary\n'
            f'      - "{v.rtde}:30004" # RTDE\n'
            f'      - "{v.novnc}:6080" # noVNC\n'
        )
        if i < len(versions) - 1:
            out.append("\n")
    return "".join(out)


# -- Docker Hub tags ------------------------------------------------------------------------

_TAG = re.compile(r"(\d+)\.(\d+)\.(\d+)")
_MINOR_TAG = re.compile(r"(\d+)\.(\d+)")


def parse_tag(tag: str) -> tuple[int, int, int] | None:
    """``5.26.1`` → ``(5, 26, 1)``; anything that isn't a full numeric PolyScope 5 tag
    (``latest``, ``5.26``, ``5.26.1-rc``, ``6.0.0``, ``05.1.0``) → None."""
    m = _TAG.fullmatch(tag)
    if not m or m.group(1) != "5" or any(p != str(int(p)) for p in m.groups()):
        return None
    return int(m.group(1)), int(m.group(2)), int(m.group(3))


def tag_key(tag: str) -> tuple[int, int, int] | None:
    """Order PolyScope 5 image tags: a full ``5.26.1`` is ``(5, 26, 1)``, a bare minor
    ``5.8`` (all UR published for 5.4-5.8) is ``(5, 8, -1)`` — older than any patch of it."""
    full = parse_tag(tag)
    if full:
        return full
    m = _MINOR_TAG.fullmatch(tag)
    if not m or m.group(1) != "5" or any(p != str(int(p)) for p in m.groups()):
        return None
    return 5, int(m.group(2)), -1


def minor_of(tag: str) -> str:
    return ".".join(tag.split(".")[:2])


def newest_per_minor(tags: list[str]) -> list[str]:
    """The newest image tag of every PolyScope 5 minor, oldest minor first: its highest
    ``5.x.y``, or the bare ``5.x`` when UR tagged no patch of it."""
    best: dict[int, tuple[tuple[int, int, int], str]] = {}
    for tag in tags:
        k = tag_key(tag)
        if k and (k[1] not in best or k > best[k[1]][0]):
            best[k[1]] = (k, tag)
    return [best[m][1] for m in sorted(best)]


def tag_drift(tags: list[str], matrix: list[str], excluded: dict[str, str] | None = None) -> list[str]:
    """What the matrix is missing given Docker Hub's ``tags``, one sentence per fix. It only
    ever grows: a minor it lacks (not :data:`EXCLUDED`) is added, a newer image of a minor it
    has replaces that one; nothing is dropped. A tag it names that Hub no longer serves is
    reported too — the matrix can't pull it (decide with Nick; don't drop it silently)."""
    excluded = excluded if excluded is not None else EXCLUDED
    have = {minor_of(t): t for t in matrix}
    problems = []
    for t in newest_per_minor(tags):
        m = minor_of(t)
        if m in excluded:
            continue
        if m not in have:
            problems.append(f"PolyScope {m} exists ({IMAGE}:{t}): add it")
        elif (tag_key(t) or (0, 0, 0)) > (tag_key(have[m]) or (0, 0, 0)):
            problems.append(f"PolyScope {t} exists: bump {have[m]} to it")
    for t in matrix:
        if t not in tags:
            problems.append(f"{IMAGE}:{t} is not on Docker Hub (the matrix can't pull it)")
    return problems


def version_drift(recorded: dict[str, str], actual: dict[str, str | None]) -> list[str]:
    """Images whose contents changed under the same tag, one sentence each: ``actual`` maps a
    matrix tag to the ``VERSION`` its image carries now (None: none readable)."""
    problems = []
    for tag, want in recorded.items():
        got = actual.get(tag)
        if got is None:
            problems.append(f"{IMAGE}:{tag} has no readable VERSION (was {want}) — check the image by hand")
        elif got != want:
            problems.append(
                f"{IMAGE}:{tag} now carries PolyScope {got} (was {want}): the tag was re-pushed — "
                "re-run the matrix on it and update IMAGE_VERSIONS"
            )
    return problems


def fetch_versions(tags) -> dict[str, str | None]:
    """The ``VERSION`` each tag's image carries now, from Docker Hub's registry."""
    from urcap5 import image_version

    return {t: image_version(f"{IMAGE}:{t}") for t in tags}


def fetch_tags(url: str = HUB_TAGS) -> list[str]:
    names: list[str] = []
    while url:
        with urllib.request.urlopen(url, timeout=30) as r:  # noqa: S310 (fixed https URL)
            page = json.load(r)
        names += [t["name"] for t in page.get("results", [])]
        url = page.get("next") or ""
    return names


# -- the URCap's evidence ------------------------------------------------------------------
#
# polyscope.log never says a URCap *started*: on 5.24.0 / 5.25.2 / 5.26.1 it logs only
# "Adding 'reference:' to bundle uri : file:/ursim/GUI/bundle/perceptronic-ps5.jar" and
# (5.24, 5.25) URCapHelper's "Removing 'reference:' from bundle location" lines — found by
# the first matrix run, 2026-09-28. The runtime truth is Felix's own: PolyScope's
# /ursim/GUI/conf/config.properties enables the Felix remote shell on 127.0.0.1:6666 inside
# the container (osgi.shell.telnet.ip/port), where `ps` gives each bundle's state and
# `services <id>` the services its activator registered — the node and toolbar services.

NODE_SERVICES = (
    "com.ur.urcap.api.contribution.installation.swing.SwingInstallationNodeService",
    "com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService",
    "com.ur.urcap.api.contribution.toolbar.swing.SwingToolbarService",  # the P button (since 0.6.0)
)
_PS_ROW = re.compile(r"^\[\s*(\d+)\]\s*\[\s*([A-Za-z]+)\s*\]\s*\[\s*(\d+)\]\s*(.*?)\s*$")
# ... and the old identity (RealSense Pilot, Olympus Controls): the real logs in tests/fixtures
_OURS = re.compile(r"io\.advin|perceptronic|com\.olympuscontrols|realsense-?pilot", re.I)
_ERROR = re.compile(
    r"exception|\berror\b|severe|could not|failed|unresolved|omitted|refused|rejected|incompatible", re.I
)


def parse_ps(text: str) -> list[dict]:
    """Felix shell ``ps`` rows: ``[ 164] [Active     ] [    1] Perceptronic (0.4.0)``."""
    rows = []
    for line in text.splitlines():
        m = _PS_ROW.match(line.strip())
        if m:
            rows.append(
                {"id": int(m.group(1)), "state": m.group(2), "level": int(m.group(3)), "name": m.group(4)}
            )
    return rows


def find_bundle(rows: list[dict], name: str, version: str) -> dict | None:
    """The row for ``Bundle-Name (Bundle-Version)`` exactly; None when absent."""
    want = f"{name} ({version})"
    return next((r for r in rows if r["name"] == want), None)


def services_command(bundle_id: int) -> str:
    """The shell command that lists what a bundle registered. PolyScope's Felix shell
    (org.apache.felix.shell 1.4.3) has no ``services`` ("Command not found", 2026-09-28)."""
    return f"inspect service capability {bundle_id}"


def missing_services(services_text: str, wanted: tuple[str, ...] = NODE_SERVICES) -> list[str]:
    """Which of ``wanted`` a ``services <id>`` listing does not name as an objectClass."""
    classes = set(re.findall(r"objectClass\s*=\s*\[?([\w.$, ]+)\]?", services_text))
    named = {c.strip() for group in classes for c in group.split(",")}
    return [w for w in wanted if w not in named]


def log_errors(log: str, package: str = PACKAGE) -> list[str]:
    """polyscope.log lines that say something went wrong with the URCap: any Java stack
    frame inside its package (``at io.advin...``) with the exception line that
    heads it, and any line naming the URCap together with a failure word."""
    lines = log.splitlines()
    errors: list[str] = []
    for i, line in enumerate(lines):
        text = line.strip()
        if text.startswith("at ") and package in text:
            head = next(
                (lines[j].strip() for j in range(i - 1, -1, -1) if not lines[j].strip().startswith("at ")), ""
            )
            for t in (head, text):
                if t and t not in errors:
                    errors.append(t)
        elif _OURS.search(text) and _ERROR.search(text) and text not in errors:
            errors.append(text)
    return errors


# -- docker ---------------------------------------------------------------------------------


def _compose(
    v: Version, *args: str, check: bool = True, timeout: float = 1800
) -> subprocess.CompletedProcess:
    cmd = [*DOCKER, "compose", "-f", str(COMPOSE_FILE), "-p", v.project, *args]
    r = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, check=False)
    if check and r.returncode != 0:
        raise RuntimeError(f"{' '.join(cmd[3:])} failed: {r.stderr.strip()[-2000:]}")
    return r


def _container(v: Version) -> str:
    return f"{v.project}-{v.service}-1"


def _exec(v: Version, script: str, timeout: float = 60) -> str:
    r = subprocess.run(
        [*DOCKER, "exec", _container(v), "sh", "-c", script],
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    return r.stdout + (r.stderr if r.returncode else "")


def stage_urcap(dist: Path = DIST, stage: Path = STAGE) -> Path:
    jars = sorted(dist.glob("perceptronic-ps5-*.urcap"))
    if len(jars) != 1:
        raise RuntimeError(f"expected one perceptronic-ps5-*.urcap in {dist}, found {len(jars)}")
    stage.mkdir(parents=True, exist_ok=True)
    for old in stage.glob("*.jar"):
        old.unlink()
    shutil.copy2(jars[0], stage / JAR_NAME)
    return jars[0]


def dashboard(port: int, command: str, host: str = "127.0.0.1", timeout: float = 5.0) -> str:
    """One Dashboard command; the reply line, or '' when nothing answers."""
    try:
        with socket.create_connection((host, port), timeout=timeout) as s:
            s.settimeout(timeout)
            f = s.makefile("rwb")
            f.readline()  # banner
            f.write(command.encode() + b"\n")
            f.flush()
            return f.readline().decode(errors="replace").strip()
    except OSError:
        return ""


def controller_ready(reply: str) -> bool:
    """Has PolyScope connected to URControl? ``NO_CONTROLLER`` and ``DISCONNECTED`` say not:
    a ``power on`` then is dropped ("Not connected to controller, trying to send string:
    power on" — PolyScope 5.5's polyscope.log, 2026-09-29, where it answered DISCONNECTED
    for a second after its Dashboard came up)."""
    m = re.fullmatch(r"Robotmode: ([A-Z_]+)", reply.strip())
    return bool(m) and m.group(1) not in ("NO_CONTROLLER", "DISCONNECTED", "BOOTING")


def wait_for_dashboard(v: Version, timeout: float) -> str:
    t0 = time.monotonic()
    reply = ""
    while time.monotonic() - t0 < timeout:
        reply = dashboard(v.dashboard, "robotmode")
        if controller_ready(reply):
            return reply
        time.sleep(3)
    raise TimeoutError(f"Dashboard :{v.dashboard} not ready after {timeout:.0f} s (last reply {reply!r})")


# Runs inside the container: connect to the Felix remote shell, run each argv command,
# print "### <command>" and its output up to the next "-> " prompt.
FELIX_SHELL = r"""
import socket, sys, time
s = socket.create_connection(("127.0.0.1", 6666), 5)
s.settimeout(10)
def until_prompt():
    buf, end = b"", time.monotonic() + 20
    while not buf.rstrip().endswith(b"->") and time.monotonic() < end:
        chunk = s.recv(65536)
        if not chunk:
            break
        buf += chunk
    return buf.decode(errors="replace")
until_prompt()
for c in sys.argv[1:]:
    s.sendall(c.encode() + b"\n")
    print("### " + c)
    print(until_prompt())
"""


def felix_shell(v: Version, *commands: str, timeout: float = 60) -> dict[str, str]:
    """``{command: output}`` from the container's Felix remote shell ({} when it is not up)."""
    r = subprocess.run(
        [*DOCKER, "exec", "-i", _container(v), "python3", "-", *commands],
        input=FELIX_SHELL,
        capture_output=True,
        text=True,
        timeout=timeout,
        check=False,
    )
    out: dict[str, str] = {}
    for chunk in r.stdout.split("### ")[1:]:
        cmd, _, body = chunk.partition("\n")
        out[cmd.strip()] = body
    return out


def urcap_identity(jar: Path) -> tuple[str, str]:
    """``(Bundle-Name, Bundle-Version)`` from the jar's own manifest."""
    sys.path.insert(0, str(REPO / "urcap"))
    from urcap5 import read_bundle

    h = read_bundle(jar)["headers"]
    return h["Bundle-Name"], h["Bundle-Version"]


def wait_for_urcap(v: Version, jar: Path, timeout: float) -> dict:
    """Poll the Felix shell until the URCap's bundle is Active with both node services
    registered (``ok``), or ``timeout``; always reports the last state seen."""
    name, version = urcap_identity(jar)
    t0 = time.monotonic()
    seen: dict = {
        "ok": False,
        "bundle": f"{name} ({version})",
        "state": None,
        "missing_services": list(NODE_SERVICES),
    }
    while time.monotonic() - t0 < timeout:
        row = find_bundle(parse_ps(felix_shell(v, "ps").get("ps", "")), name, version)
        if row:
            seen.update(id=row["id"], state=row["state"])
            if row["state"] == "Active":
                listing = felix_shell(v, services_command(row["id"])).get(services_command(row["id"]), "")
                seen["missing_services"] = missing_services(listing)
                if not seen["missing_services"]:
                    seen["ok"] = True
                    break
        time.sleep(3)
    seen["waited_s"] = round(time.monotonic() - t0, 1)
    return seen


def gateway(v: Version) -> str:
    """The address the container reaches this machine at: its network's gateway."""
    r = subprocess.run(
        [
            *DOCKER,
            "inspect",
            "-f",
            "{{range .NetworkSettings.Networks}}{{.Gateway}} {{end}}",
            _container(v),
        ],
        capture_output=True,
        text=True,
        check=True,
    )
    gw = r.stdout.split()
    if not gw:
        raise RuntimeError(f"{_container(v)} has no network gateway")
    return gw[0]


# Run inside the container (sh) when a run fails, or always with PS5_MATRIX_COLLECT.
EVIDENCE = r"""
echo '## polyscope.log lines about URCaps'
grep -n -i -E 'urcap|olympus|realsense' /ursim/polyscope.log | grep -v "Adding 'reference" | tail -100
echo '## /ursim/GUI/bundle (ours)'; ls -la /ursim/GUI/bundle | grep -i realsense
echo '## /ursim/.urcaps'; ls -la /ursim/.urcaps 2>&1
echo '## felix-cache bundle.info naming us'
for d in $(grep -l -i realsense /ursim/GUI/felix-cache/bundle*/bundle.info 2>/dev/null); do
  echo "# $d"; cat -A "$d"
done
echo '## felix / shell config'; ls /ursim/GUI/conf 2>&1
grep -r -n -i -E 'shell|telnet|felix' /ursim/GUI/conf 2>/dev/null | head -40
"""


def collect(v: Version, out: Path) -> list[str]:
    out.mkdir(parents=True, exist_ok=True)
    saved = []
    for src in ("/ursim/polyscope.log", "/ursim/URControl.log"):
        dst = out / Path(src).name
        r = subprocess.run(
            [*DOCKER, "cp", f"{_container(v)}:{src}", str(dst)], capture_output=True, check=False
        )
        if r.returncode == 0:
            saved.append(str(dst))
    (out / "evidence.txt").write_text(_exec(v, EVIDENCE, timeout=120), encoding="utf-8")
    saved.append(str(out / "evidence.txt"))
    shell = felix_shell(v, "ps")
    row = next((r for r in parse_ps(shell.get("ps", "")) if r["name"].startswith("Perceptronic")), None)
    if row:
        shell.update(felix_shell(v, "help", services_command(row["id"]), f"headers {row['id']}"))
    (out / "felix.txt").write_text("".join(f"### {k}\n{t}" for k, t in shell.items()), encoding="utf-8")
    saved.append(str(out / "felix.txt"))
    logs = _compose(v, "logs", "--no-color", v.service, check=False)
    (out / "container.log").write_text(logs.stdout + logs.stderr, encoding="utf-8")
    saved.append(str(out / "container.log"))
    return saved


def api_check(v: Version, jar: Path) -> dict:
    """``urcap5.py check`` against this image's own URCap API jars (read out of the pulled
    image, nothing runs): the sources this PolyScope loads compile, and every package the
    jar needs is exported. Urcap5Error (the javac output) fails the version."""
    sys.path.insert(0, str(REPO / "urcap"))
    import urcap5

    sdk = SDK / v.tag
    urcap5.fetch_sdk(f"{IMAGE}:{v.tag}", sdk, source="docker")
    return urcap5.check(SRC, sdk, jar, v.tag)


def down(v: Version) -> None:
    _compose(v, "down", "-v", "--remove-orphans", check=False, timeout=300)


def run_version(v: Version, artifacts: Path, *, boot_timeout: float, urcap_timeout: float) -> dict:
    report: dict = {"version": v.tag, "service": v.service, "ports": asdict(v), "ok": False, "stages": {}}
    stages = report["stages"]
    t0 = time.monotonic()
    try:
        jar = stage_urcap()
        report["urcap"] = jar.name
        _compose(v, "pull", "--quiet", v.service)
        stages["api"] = api_check(v, jar)
        _compose(v, "up", "-d", v.service)
        stages["dashboard"] = wait_for_dashboard(v, boot_timeout)
        stages["boot_s"] = round(time.monotonic() - t0, 1)
        stages["urcap"] = wait_for_urcap(v, jar, urcap_timeout)
        stages["urcap"]["log_errors"] = log_errors(_exec(v, "cat /ursim/polyscope.log 2>/dev/null"))
        if stages["urcap"]["log_errors"]:
            raise RuntimeError(
                f"PolyScope logged {len(stages['urcap']['log_errors'])} error line(s) about the URCap"
            )
        if not stages["urcap"]["ok"]:
            u = stages["urcap"]
            raise RuntimeError(
                f"{u['bundle']} not started with its node services within {urcap_timeout:.0f} s "
                f"(state {u['state']}, missing {', '.join(u['missing_services']) or 'none'})"
            )
        sys.path.insert(0, str(REPO))
        from urctl import Robot, RobotConfig

        robot = Robot(
            RobotConfig.from_env(host="127.0.0.1", dashboard_port=v.dashboard, primary_port=v.primary)
        )
        up = robot.bring_up()
        stages["bring_up"] = {"ok": up.get("ok"), "robot_mode": up.get("robot_mode")}
        if not up.get("ok"):
            raise RuntimeError(f"bring-up failed: {up}")
        gw = gateway(v)
        stages["reach_back"] = gw
        cmd = [
            sys.executable,
            str(REPO / "urcap" / "pick5_e2e.py"),
            "--host",
            "127.0.0.1",
            "--dash-port",
            str(v.dashboard),
            "--primary-port",
            str(v.primary),
            "--reach-back",
            gw,
            "--bind",
            gw,
            "--port",
            str(v.pick),
        ]
        env = {**os.environ, "UR_RTDE_PORT": str(v.rtde)}
        e2e = subprocess.run(cmd, capture_output=True, text=True, timeout=600, env=env, check=False)
        stages["pick_e2e"] = {"exit": e2e.returncode, "output": (e2e.stdout + e2e.stderr).splitlines()[-60:]}
        # A late exception (e.g. when the node's classes are touched at program run) still fails.
        late = log_errors(_exec(v, "cat /ursim/polyscope.log 2>/dev/null"))
        if late:
            stages["urcap"]["log_errors"] = late
            raise RuntimeError(f"PolyScope logged {len(late)} error line(s) about the URCap during the run")
        if e2e.returncode != 0:
            raise RuntimeError("pick5_e2e failed")
        report["ok"] = True
    except Exception as exc:  # noqa: BLE001 — every failure becomes the report's reason
        report["error"] = f"{type(exc).__name__}: {exc}"
    finally:
        out = artifacts / v.minor
        try:
            if not report["ok"] or os.environ.get("PS5_MATRIX_COLLECT"):
                report["artifacts"] = collect(v, out)
        except Exception as exc:  # noqa: BLE001
            report["collect_error"] = str(exc)
        down(v)
        report["took_s"] = round(time.monotonic() - t0, 1)
    return report


def _selected(name: str) -> list[Version]:
    if name == "all":
        return list(MATRIX.values())
    key = ".".join(name.split(".")[:2])
    if key not in MATRIX:
        raise SystemExit(f"unknown version {name!r}; the matrix is {', '.join(MATRIX)} (or all)")
    return [MATRIX[key]]


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="ps5_matrix", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    rn = sub.add_parser("run", help="boot, check the URCap, run the pick e2e, tear down")
    rn.add_argument("--version", default="all", help="5.4 / 5.25.2 / ... or all")
    rn.add_argument("--artifacts", default="ps5-artifacts", help="where logs go on failure")
    rn.add_argument("--report", help="write the JSON summaries here")
    rn.add_argument("--boot-timeout", type=float, default=600.0)
    rn.add_argument("--urcap-timeout", type=float, default=240.0)
    dn = sub.add_parser("down", help="docker compose down -v")
    dn.add_argument("--version", default="all")
    sub.add_parser("ports", help="print the host-port table as JSON")
    sub.add_parser("compose", help="print docker-compose.ps5-matrix.yml as MATRIX makes it")
    sub.add_parser("check-tags", help="compare MATRIX with Docker Hub's tags (it only ever grows)")
    sub.add_parser("list", help="print the matrix's minors as a JSON list (for CI)")
    args = ap.parse_args(argv)

    if args.cmd == "ports":
        print(json.dumps({m: asdict(v) | {"service": v.service} for m, v in MATRIX.items()}, indent=2))
        return 0
    if args.cmd == "compose":
        sys.stdout.write(render_compose())
        return 0
    if args.cmd == "list":
        print(json.dumps(list(MATRIX)))
        return 0
    if args.cmd == "check-tags":
        tags = fetch_tags()
        problems = tag_drift(tags, [v.tag for v in MATRIX.values()])
        problems += version_drift(IMAGE_VERSIONS, fetch_versions(IMAGE_VERSIONS))
        print(
            json.dumps(
                {
                    "matrix": [v.tag for v in MATRIX.values()],
                    "newest": newest_per_minor(tags),
                    "excluded": EXCLUDED,
                    "problems": problems,
                }
            )
        )
        for p in problems:
            print(f"::error::{p} (urcap/ps5_matrix.py TAGS, then `ps5_matrix.py compose`)", file=sys.stderr)
        return 1 if problems else 0
    if args.cmd == "down":
        for v in _selected(args.version):
            down(v)
        return 0

    reports = []
    for v in _selected(args.version):
        rep = run_version(
            v, Path(args.artifacts), boot_timeout=args.boot_timeout, urcap_timeout=args.urcap_timeout
        )
        print(json.dumps(rep, indent=2), flush=True)
        reports.append(rep)
    if args.report:
        Path(args.report).parent.mkdir(parents=True, exist_ok=True)
        Path(args.report).write_text(json.dumps(reports, indent=2), encoding="utf-8")
    return 0 if all(r["ok"] for r in reports) else 1


if __name__ == "__main__":
    sys.exit(main())
