#!/usr/bin/env python3
"""The PolyScope X URCap on the newest PolyScope X releases — stdlib + Docker + the e2e.

For each release in :data:`RELEASES` (the newest PolyScope X release tags UR publishes a
``universalrobots/ursim_polyscopex`` image for — a release is ``10.<minor>.<patch>`` with
no ``-preview`` / ``-beta`` suffix; every tag is published for amd64 and arm64 — up to
:data:`KEEP` of them and none older than :data:`FLOOR`, the oldest release the URCap
supports), run
``urcap/e2e.py`` against that image: boot it, install the committed package over the
urservice endpoint, and in headless Chromium load the application node (feed, hover,
click → segment, PolyScope's kinematics services) and the Pick program node (toolbox →
tree row → its dialog → a picture point from PolyScope's joints → the row's verdict).
One JSON summary per release; ``--rmi`` removes each image after its run (they are 4–5
GB each and a sim's inner Docker is ~12 GB more, which ``e2e.py`` frees with ``rm -v``).

``check-tags`` compares RELEASES with Docker Hub: a newer release UR has published that the
list lacks is drift (``list`` prints the list as JSON for CI).

    python3 urcap/psx_matrix.py run --version 10.14.0 --artifacts psx-artifacts
    python3 urcap/psx_matrix.py run --version all --rmi
    python3 urcap/psx_matrix.py list
    python3 urcap/psx_matrix.py check-tags
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
E2E = REPO / "urcap" / "e2e.py"
IMAGE = "universalrobots/ursim_polyscopex"
HUB_TAGS = f"https://hub.docker.com/v2/repositories/{IMAGE}/tags?page_size=100"
KEEP = 10
# The oldest PolyScope X the URCap is built for. 10.6 and 10.7 were dropped 2026-09-30 (Nick):
# their simulators' own web app does not start on GitHub's amd64 runners, and nobody will
# run them — the floor moves up, never down.
FLOOR = "10.8.0"

# The newest PolyScope X releases with a simulator image (Docker Hub, 2026-09-30), newest
# first, none older than FLOOR. `check-tags` says when UR has published a newer one.
RELEASES: list[str] = [
    "10.14.0",
    "10.13.0",
    "10.12.1",
    "10.12.0",
    "10.11.0",
    "10.10.0",
    "10.9.0",
    "10.8.0",
]

_RELEASE = re.compile(r"^(\d+)\.(\d+)\.(\d+)$")


def release_key(tag: str) -> tuple[int, int, int] | None:
    """``10.12.1`` → (10, 12, 1); None for anything else — previews, betas, and the
    SDK-numbered ``0.x.y`` tags UR pushes beside every release (PolyScope X is 10.x)."""
    m = _RELEASE.fullmatch(tag)
    if not m or int(m.group(1)) < 10:
        return None
    return (int(m.group(1)), int(m.group(2)), int(m.group(3)))


def newest_releases(tags: list[str], keep: int = KEEP, floor: str = FLOOR) -> list[str]:
    """The ``keep`` newest release tags not older than ``floor``, newest first."""
    rel = [t for t in tags if release_key(t) and release_key(t) >= release_key(floor)]
    return sorted(set(rel), key=release_key, reverse=True)[:keep]


def tag_drift(hub_tags: list[str], releases: list[str], keep: int = KEEP) -> list[str]:
    """What RELEASES should say given Docker Hub's tags: a release newer than the newest
    listed that is missing, or a listed tag Docker Hub does not have."""
    out = []
    want = newest_releases(hub_tags, keep)
    top = max((release_key(t) for t in releases if release_key(t)), default=(0, 0, 0))
    for t in want:
        if t not in releases and release_key(t) > top:
            out.append(f"PolyScope X {t} exists on Docker Hub: add it (newest first) and drop the oldest")
    for t in releases:
        if t not in hub_tags:
            out.append(f"{t} is not on Docker Hub any more")
    return out


def fetch_tags(url: str = HUB_TAGS) -> list[str]:
    tags: list[str] = []
    while url:
        with urllib.request.urlopen(url, timeout=60) as r:
            d = json.load(r)
        tags += [t["name"] for t in d.get("results", [])]
        url = d.get("next")
    return tags


def run_release(
    tag: str, artifacts: Path, *, docker: str = "docker", rmi: bool = False, python: str = sys.executable
) -> dict:
    """One e2e against ``IMAGE:tag``; ``{tag, image, passed, seconds, report, log}``."""
    image = f"{IMAGE}:{tag}"
    out = artifacts / tag
    out.mkdir(parents=True, exist_ok=True)
    report = out / "e2e.json"
    log = out / "e2e.log"
    t0 = time.monotonic()
    cmd = [python, str(E2E), "--image", image, "--artifacts", str(out), "--report", str(report)]
    env = {**os.environ, "DOCKER": docker}
    with log.open("w", encoding="utf-8") as fh:
        proc = subprocess.run(cmd, cwd=REPO, env=env, stdout=fh, stderr=subprocess.STDOUT, text=True)
    text = log.read_text(encoding="utf-8", errors="replace")
    passed = proc.returncode == 0 and "e2e: passed" in text
    result = {
        "tag": tag,
        "image": image,
        "passed": passed,
        "seconds": round(time.monotonic() - t0, 1),
        "report": str(report) if report.is_file() else None,
        "log": str(log),
        "checks": _checks(text),
    }
    if rmi:
        subprocess.run([docker, "rmi", image], capture_output=True)
    return result


def _checks(text: str) -> list[dict]:
    """The e2e's own check lines (``  ok    name — detail`` / ``  FAIL  name — detail``)."""
    out = []
    for line in text.splitlines():
        m = re.match(r"^\s+(ok|FAIL)\s+(.*?)(?: — (.*))?$", line)
        if m:
            out.append({"ok": m.group(1) == "ok", "name": m.group(2), "detail": (m.group(3) or "")[:300]})
    return out


def summary_markdown(results: list[dict]) -> str:
    rows = ["| PolyScope X | Image | Result | Time |", "| --- | --- | --- | --- |"]
    for r in results:
        failed = [c["name"] for c in r["checks"] if not c["ok"]]
        verdict = "PASS" if r["passed"] else f"FAIL ({'; '.join(failed) or 'see log'})"
        rows.append(f"| {r['tag']} | `{r['image']}` | {verdict} | {r['seconds']:.0f} s |")
    return "\n".join(rows) + "\n"


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="psx_matrix", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    rn = sub.add_parser("run", help="e2e per release: boot the sim, install, click through both nodes")
    rn.add_argument("--version", default="all", help="a release tag, or all")
    rn.add_argument("--artifacts", default="target/psx-matrix", help="per-release logs, reports, screenshots")
    rn.add_argument("--rmi", action="store_true", help="remove each image after its run")
    rn.add_argument("--docker", default=os.environ.get("DOCKER", "docker"), help=argparse.SUPPRESS)
    sub.add_parser("list", help="the releases as JSON (CI's matrix)")
    sub.add_parser("check-tags", help="RELEASES vs Docker Hub; exit 1 on drift")
    args = ap.parse_args(argv)
    if args.cmd == "list":
        print(json.dumps(RELEASES))
        return 0
    if args.cmd == "check-tags":
        drift = tag_drift(fetch_tags(), RELEASES)
        for d in drift:
            print(d)
        print("RELEASES is current" if not drift else f"{len(drift)} drift(s)")
        return 1 if drift else 0
    versions = RELEASES if args.version == "all" else [args.version]
    unknown = [v for v in versions if v not in RELEASES]
    if unknown:
        print(f"psx_matrix: not in RELEASES: {', '.join(unknown)}", file=sys.stderr)
        return 2
    artifacts = Path(args.artifacts)
    results = []
    for tag in versions:
        print(f"== PolyScope X {tag}", flush=True)
        r = run_release(tag, artifacts, docker=args.docker, rmi=args.rmi)
        results.append(r)
        print(f"   {'PASS' if r['passed'] else 'FAIL'} in {r['seconds']:.0f} s", flush=True)
        for c in r["checks"]:
            if not c["ok"]:
                print(f"   FAIL {c['name']} — {c['detail']}", flush=True)
    artifacts.mkdir(parents=True, exist_ok=True)
    (artifacts / "summary.json").write_text(json.dumps(results, indent=1), encoding="utf-8")
    (artifacts / "summary.md").write_text(summary_markdown(results), encoding="utf-8")
    print(summary_markdown(results))
    return 0 if all(r["passed"] for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
