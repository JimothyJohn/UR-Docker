#!/usr/bin/env python3
"""Keep the PolyScope X URCap on UR's newest release — stdlib only.

The URCap (``urcap/perceptronic``) is built and tested for exactly one
PolyScope X release: the newest patch of the newest minor UR has published
release notes for. ``urcap/target.json`` pins it, together with what that
release pairs with — the simulator image (by digest) and the URCap SDK
(``UniversalRobots/PolyScopeX_URCap_SDK``) with the component versions its
templates use. Nothing here needs a login: the release notes, the SDK, the
Docker Hub tags and UR's npm feed are all public.

    python3 urcap/track.py check      # exit 1 (reasons on stderr) when UR has moved on
    python3 urcap/track.py update     # re-resolve, rewrite target.json + the docs' target lines
    python3 urcap/track.py compat     # the URCap against the pinned SDK (API surface, manifest, worker)
    python3 urcap/track.py show       # the resolved newest release as JSON; writes nothing

``update`` then ``compat`` then ``urcap/e2e.py`` (install + load the node in the
pinned simulator) is what ``.github/workflows/urcap-track.yml`` runs weekly; a
green run lands as a PR, anything else as a ``urcap-attention`` issue.
"""

from __future__ import annotations

import argparse
import datetime as dt
import io
import json
import os
import re
import sys
import tarfile
import urllib.error
import urllib.request
import zipfile
from dataclasses import dataclass
from html import unescape
from pathlib import Path

HERE = Path(__file__).resolve().parent
TARGET = HERE / "target.json"
URCAP = HERE / "perceptronic"
FRONTEND = URCAP / "perceptronic-frontend"
DOCS = (HERE / "README.md", HERE / "DEVELOPING.md")
DOC_MARK = re.compile(r"(<!-- urcap-target -->)(.*?)(<!-- /urcap-target -->)", re.S)

IMAGE_REPO = "universalrobots/ursim_polyscopex"
SDK_REPO = "UniversalRobots/PolyScopeX_URCap_SDK"
# The worker speaks threads.js's protocol by hand (see its header); a template that
# moves off this version needs the protocol re-verified before anything ships.
WORKER_THREADS = "1.7.0"
# How long a release may wait for its SDK / simulator image before that is a problem.
PENDING_GRACE_DAYS = 21
# With no pin yet, walk forward from the first PolyScope X release-notes article.
FIRST_MINOR = "10.7"

# Every PolyScope X member the URCap calls (main.js) or implements (the worker),
# checked against the pinned contribution-api typings. tests/test_urcap_track.py
# holds this list to the source so a new call can't skip the check.
API_SURFACE: dict[str, tuple[str, ...]] = {
    "ApplicationPresenterAPI": (
        "applicationNodeService",
        "robotPositionService",
        "robotMoveService",
        "robotInfoService",
    ),
    "ApplicationNodeService": ("updateNode",),
    "RobotPositionService": (
        "getKinematicInfo",
        "getInverseKinematics",
        "convertJointPositionsToTcpPose",
        "getJointPositions",
    ),
    "RobotMoveService": ("autoMove",),
    "RobotInfoService": ("getRobotType",),
    "ApplicationPresenter": ("applicationNode", "applicationAPI", "robotSettings"),
    "ApplicationBehaviors": ("factory", "upgradeNode", "downgradeNode"),
    # the Perceptronic Pick program node (pick.js + pick-node.worker.js / after-node.worker.js)
    "ProgramPresenterAPI": (
        "programNodeService",
        "applicationService",
        "variableService",
        "robotPositionService",
        "robotMoveService",
        "dialogService",
        "symbolService",
    ),
    "SymbolService": ("generateVariable",),
    "ProgramNodeService": ("updateNode",),
    "DialogService": ("openCustomDialog",),
    "ApplicationService": ("getApplicationNode",),
    "VariableService": ("createVariable",),
    "ProgramPresenter": (
        "contributedNode",
        "presenterAPI",
        "robotSettings",
        "programTree",
        "applicationContext",
    ),
    "ProgramBehaviors": (
        "factory",
        "programNodeLabel",
        "validator",
        "generateCodeBeforeChildren",
        "generateCodeAfterChildren",
        "allowsChild",
        "upgradeNode",
        "onLifeCycleHook",
    ),
}


class TrackError(RuntimeError):
    pass


@dataclass(frozen=True)
class Sources:
    """Where each fact comes from; tests point these at a local server."""

    notes: str = (
        "https://www.universal-robots.com/articles/ur/release-notes/release-note-software-version-{slug}/"
    )
    github_api: str = "https://api.github.com"
    github_raw: str = "https://raw.githubusercontent.com"
    docker_hub: str = "https://hub.docker.com/v2/repositories"
    npm_feed: str = "https://pkgs.dev.azure.com/polyscopex/api/_packaging/polyscopex/npm/registry"


# -- HTTP -----------------------------------------------------------------------------------------


def _get(url: str, *, accept: str | None = None) -> tuple[int, bytes]:
    headers = {"User-Agent": "perceptronics-urcap-track (+https://github.com/Olympus-Controls/UR-utils)"}
    if accept:
        headers["Accept"] = accept
    token = os.environ.get("GH_TOKEN") or os.environ.get("GITHUB_TOKEN")
    if token and url.startswith("https://api.github.com/"):
        headers["Authorization"] = f"Bearer {token}"
    try:
        with urllib.request.urlopen(urllib.request.Request(url, headers=headers), timeout=60) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, b""
    except (urllib.error.URLError, OSError) as exc:
        raise TrackError(f"GET {url}: {exc}") from None


def _get_ok(url: str, **kw) -> bytes:
    status, body = _get(url, **kw)
    if status != 200:
        raise TrackError(f"GET {url}: HTTP {status}")
    return body


def _json(url: str, **kw):
    try:
        return json.loads(_get_ok(url, **kw))
    except json.JSONDecodeError as exc:
        raise TrackError(f"GET {url}: not JSON ({exc})") from None


# -- release notes --------------------------------------------------------------------------------


def notes_slug(minor: str) -> str:
    """``10.14`` → ``1014x``, the article's URL slug."""
    major, m = minor.split(".")
    return f"{major}{m}x"


def html_blocks(html: str) -> list[tuple[str, str]]:
    """``[(tag, text)]`` for each heading / paragraph / list item, in order."""
    html = re.sub(r"(?is)<(script|style|noscript)\b.*?</\1>", "", html)
    out = []
    for m in re.finditer(r"(?is)<(h[1-6]|p|li|td)\b[^>]*>(.*?)</\1>", html):
        text = unescape(re.sub(r"<[^>]+>", " ", m.group(2)))
        text = re.sub(r"\s+", " ", text).strip()
        if text:
            out.append((m.group(1).lower(), text))
    return out


def parse_notes(html: str) -> dict:
    """The newest release on a PolyScope X release-notes page.

    UR lists patches newest first (``SW 10.13.1 Release Notes`` above
    ``SW 10.13.0 …``), each followed by a *Release Versions* block."""
    text = "\n".join(t for _, t in html_blocks(html))
    release = re.search(r"\bSW (\d+\.\d+\.\d+) Release Notes", text)
    if not release:
        raise TrackError("release notes: no 'SW x.y.z Release Notes' heading")
    after = text[release.start() :]
    image = re.search(r"PolyScope X robot image:\s*(\d+\.\d+\.\d+)", after)
    date = re.search(r"Date of release:\s*([A-Z][a-z]+ \d{1,2})(?:st|nd|rd|th)?,? (\d{4})", text)
    released = None
    if date:
        try:
            released = dt.datetime.strptime(f"{date.group(1)} {date.group(2)}", "%B %d %Y").date().isoformat()
        except ValueError:
            released = None
    sdk = re.search(r"URCap X SDK:\s*([\d.]+\d)", after)
    return {
        "release": release.group(1),
        "robot_image": image.group(1) if image else release.group(1),
        "released": released,
        "notes_sdk": sdk.group(1) if sdk else None,
        "breaking": [t for _, t in html_blocks(html) if re.search(r"(?i)breaking change", t)],
        "urcap_sections": urcap_sections(html),
    }


def urcap_sections(html: str, limit: int = 12) -> list[dict]:
    """The notes' passages a URCap author must read: every section whose heading
    mentions URCap or API (its paragraphs, trimmed) — for the PR/issue body."""
    blocks = html_blocks(html)
    out: list[dict] = []
    covered = -1  # a matching subsection of a section already taken is inside its text
    for i, (tag, text) in enumerate(blocks):
        if i <= covered or not tag.startswith("h") or not re.search(r"\bURCaps?\b|\bAPI\b", text):
            continue
        level = int(tag[1])
        body = []
        covered = i
        for tag2, text2 in blocks[i + 1 :]:
            if tag2.startswith("h") and int(tag2[1]) <= level:
                break
            body.append(text2)
            covered += 1
        out.append({"heading": text, "text": " ".join(body)[:1200]})
        if len(out) >= limit:
            break
    return out


def newest_minor(src: Sources, start: str) -> tuple[str, str]:
    """``(minor, html)`` of the newest release-notes article at or after ``start``.

    UR has published one article per minor (10.7 … 10.14), so this probes the
    next minor, and the next major's ``.0``, until one is missing."""
    major, m = (int(x) for x in start.split("."))
    status, body = _get(src.notes.format(slug=notes_slug(start)))
    if status != 200:
        raise TrackError(f"the pinned minor's release notes ({start}) answered HTTP {status}")
    minor, html = start, body.decode("utf-8", "replace")
    while True:
        for cand in (f"{major}.{m + 1}", f"{major + 1}.0"):
            status, body = _get(src.notes.format(slug=notes_slug(cand)))
            if status == 200:
                minor, html = cand, body.decode("utf-8", "replace")
                major, m = (int(x) for x in cand.split("."))
                break
        else:
            return minor, html


# -- simulator image ------------------------------------------------------------------------------


def image_tags(src: Sources, minor: str) -> list[dict]:
    """Released (``x.y.z``, not preview/build) simulator tags of ``minor``, newest first."""
    url = f"{src.docker_hub}/{IMAGE_REPO}/tags?page_size=100&name={minor}."
    tags = []
    while url:
        page = _json(url)
        for t in page.get("results", []):
            if re.fullmatch(rf"{re.escape(minor)}\.\d+", t.get("name", "")):
                tags.append(
                    {
                        "tag": t["name"],
                        "digest": t.get("digest"),
                        "architectures": sorted(
                            {i.get("architecture") for i in t.get("images", []) if i.get("architecture")}
                        ),
                    }
                )
        url = page.get("next")
    return sorted(tags, key=lambda t: int(t["tag"].rsplit(".", 1)[1]), reverse=True)


def image_digest(src: Sources, tag: str) -> str | None:
    status, body = _get(f"{src.docker_hub}/{IMAGE_REPO}/tags/{tag}")
    return json.loads(body).get("digest") if status == 200 else None


# -- SDK ------------------------------------------------------------------------------------------


def sdk_release(src: Sources, minor: str) -> dict | None:
    """The newest SDK release UR titled for this minor (``PolyScope X 10.14 - SDK-v0.21``)."""
    rels = _json(f"{src.github_api}/repos/{SDK_REPO}/releases?per_page=50")
    hits = [
        r
        for r in rels
        if not r.get("draft")
        and not r.get("prerelease")
        and re.search(rf"PolyScope X {re.escape(minor)}(?![\d.])", r.get("name") or "")
    ]
    return max(hits, key=lambda r: r.get("published_at") or "", default=None)


def _raw(src: Sources, tag: str, path: str) -> bytes:
    return _get_ok(f"{src.github_raw}/{SDK_REPO}/{tag}/{path}")


def parse_devcontainer(dockerfile: str, devcontainer: str) -> dict:
    env = dict(re.findall(r"\b(PS_VERSION|URSIM_VERSION|SDK_VERSION)=([\d.]+)", dockerfile))
    missing = {"PS_VERSION", "URSIM_VERSION", "SDK_VERSION"} - env.keys()
    if missing:
        raise TrackError(f"SDK devcontainer Dockerfile has no {', '.join(sorted(missing))}")
    spec = re.search(r"manifest-spec-(\d+\.\d+\.\d+)\.json", devcontainer)
    return {
        "version": env["SDK_VERSION"],
        "polyscope": env["PS_VERSION"],
        "ursim": env["URSIM_VERSION"],
        "manifest_spec": spec.group(1) if spec else None,
    }


def template_versions(sdk_zip: bytes) -> dict:
    """The component versions the SDK's generator writes into a new JavaScript URCap.
    The templates are Jinja (``{% if %}`` inside the JSON), so the pins are read by
    pattern rather than parsed."""

    def pin(text: str, package: str) -> str | None:
        m = re.search(rf'"{re.escape(package)}"\s*:\s*"([^"]+)"', text)
        return m.group(1) if m else None

    out: dict[str, str | None] = {"contribution_api": None, "urcap_utils": None, "threads": None}
    with zipfile.ZipFile(io.BytesIO(sdk_zip)) as z:
        for name in z.namelist():
            if not name.endswith("/package.json") or "/templates/" not in name:
                continue
            text = z.read(name).decode("utf-8", "replace")
            if "frontendType == 'javascript'" in name:
                out["contribution_api"] = pin(text, "@universal-robots/contribution-api")
                out["threads"] = pin(text, "threads")
            elif name.endswith("{{ urcapId }}/package.json"):
                out["urcap_utils"] = pin(text, "@universal-robots/urcap-utils")
    return out


def manifest_spec(sdk_zip: bytes, version: str) -> dict:
    with zipfile.ZipFile(io.BytesIO(sdk_zip)) as z:
        for name in z.namelist():
            if name.endswith(f"manifest-spec-{version}.json"):
                return json.loads(z.read(name))
    raise TrackError(f"the SDK zip has no manifest-spec-{version}.json")


def sdk_zip(src: Sources, tag: str, version: str) -> bytes:
    return _raw(src, tag, f".devcontainer/web-sdk-external-{version}.zip")


# -- resolve / diff -------------------------------------------------------------------------------


def held_minor(src: Sources, minor: str) -> tuple[str, str]:
    """``(minor, html)`` of exactly ``minor``'s release-notes article (a ``hold``)."""
    status, body = _get(src.notes.format(slug=notes_slug(minor)))
    if status != 200:
        raise TrackError(f"the held minor's release notes ({minor}) answered HTTP {status}")
    return minor, body.decode("utf-8", "replace")


def resolve(src: Sources, start_minor: str, hold: str | None = None) -> tuple[dict | None, dict, list[str]]:
    """``(target, notes, pending)``: the newest release as a target.json body
    (``None`` while its SDK or simulator image isn't out), the parsed notes, and
    what it is still waiting for. With ``hold``, the newest patch of that minor
    only — newer minors are not looked at."""
    minor, html = held_minor(src, hold) if hold else newest_minor(src, start_minor)
    notes = parse_notes(html)
    pending: list[str] = []
    tags = image_tags(src, minor)
    image = next((t for t in tags if t["tag"] == notes["robot_image"]), None)
    if image is None and tags:
        # A patch UR shipped without a simulator build (10.13.1 has none): test the
        # newest simulator of the same minor rather than wait forever.
        image = tags[0]
        pending.append(f"no simulator image for {notes['robot_image']}; testing against {image['tag']}")
    elif image is None:
        pending.append(f"no {IMAGE_REPO} image for {minor}.x yet")
    rel = sdk_release(src, minor)
    if rel is None:
        pending.append(f"no {SDK_REPO} release titled for PolyScope X {minor} yet")
    if image is None or rel is None:
        return None, {**notes, "minor": minor}, pending

    tag = rel["tag_name"]
    sdk = {"tag": tag, "release": rel.get("name")}
    sdk.update(
        parse_devcontainer(
            _raw(src, tag, ".devcontainer/Dockerfile").decode(),
            _raw(src, tag, ".devcontainer/devcontainer.json").decode(),
        )
    )
    sdk.update(template_versions(sdk_zip(src, tag, sdk["version"])))
    sdk_sim = image_digest(src, sdk["ursim"])
    target = {
        "polyscope_x": {
            "minor": minor,
            "release": notes["release"],
            "released": notes["released"],
            "notes": src.notes.format(slug=notes_slug(minor)),
            "notes_sdk": notes["notes_sdk"],
        },
        "simulator": {
            "image": f"{IMAGE_REPO}:{image['tag']}",
            "digest": image["digest"],
            "architectures": image["architectures"],
            # The SDK names its simulator by build number (0.19.135); it should be
            # the same image as the notes' robot image.
            "matches_sdk": bool(sdk_sim) and sdk_sim == image["digest"],
        },
        "sdk": sdk,
    }
    return target, {**notes, "minor": minor}, pending


def overdue(notes: dict, today: dt.date | None = None) -> bool:
    if not notes.get("released"):
        return False
    age = (today or dt.date.today()) - dt.date.fromisoformat(notes["released"])
    return age.days > PENDING_GRACE_DAYS


def drift(pinned: dict, latest: dict) -> list[str]:
    """Why ``pinned`` is not ``latest`` (empty = up to date)."""
    out = []
    p, n = pinned.get("polyscope_x", {}), latest["polyscope_x"]
    if p.get("minor") != n["minor"]:
        out.append(f"new PolyScope X minor: {p.get('minor')} → {n['minor']} ({n['notes']})")
    elif p.get("release") != n["release"]:
        out.append(f"new PolyScope X release: {p.get('release')} → {n['release']}")
    ps, ns = pinned.get("simulator", {}), latest["simulator"]
    if ps.get("image") != ns["image"]:
        out.append(f"simulator image: {ps.get('image')} → {ns['image']}")
    elif ps.get("digest") != ns["digest"]:
        out.append(f"simulator tag {ns['image']} moved: {ps.get('digest')} → {ns['digest']} (re-test it)")
    for key in ("tag", "version", "contribution_api", "urcap_utils", "threads", "manifest_spec"):
        if pinned.get("sdk", {}).get(key) != latest["sdk"].get(key):
            out.append(f"SDK {key}: {pinned.get('sdk', {}).get(key)} → {latest['sdk'].get(key)}")
    return out


def load_target(path: Path = TARGET) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise TrackError(f"{path} is missing — run `track.py update`") from None


def write_target(target: dict, path: Path = TARGET) -> None:
    path.write_text(json.dumps(target, indent=2) + "\n", encoding="utf-8")


def doc_line(target: dict) -> str:
    px, sim, sdk = target["polyscope_x"], target["simulator"], target["sdk"]
    which = f"held at {target['hold']}" if target.get("hold") else "the newest minor"
    return (
        f"Built and tested for **PolyScope X {px['release']}** ({which}, {px['minor']}; "
        f"simulator `{sim['image']}`, URCap SDK {sdk['version']} / contribution-api "
        f"{sdk['contribution_api']}) — pinned in [`target.json`](target.json) and kept current by "
        "`.github/workflows/urcap-track.yml`."
    )


def patch_docs(target: dict, docs: tuple[Path, ...] = DOCS) -> list[Path]:
    """Rewrite each doc's ``<!-- urcap-target -->…<!-- /urcap-target -->`` line."""
    changed = []
    line = doc_line(target)
    for path in docs:
        text = path.read_text(encoding="utf-8")
        new = DOC_MARK.sub(lambda m: m.group(1) + line + m.group(3), text)
        if new != text:
            path.write_text(new, encoding="utf-8")
            changed.append(path)
    return changed


# -- compat: the URCap against the pinned SDK -----------------------------------------------------


def _strip_comments(src: str) -> str:
    return re.sub(r"/\*.*?\*/|//[^\n]*", "", src, flags=re.S)


def _declaration(dts: str, name: str) -> tuple[str, list[str]] | None:
    """``(body, bases)`` of interface/class/type ``name`` in a .d.ts: the text of
    its own ``{…}`` members and the names it extends or intersects."""
    m = re.search(rf"\b(interface|class|type)\s+{re.escape(name)}\b", dts)
    if not m:
        return None
    i, depth = m.end(), 0
    # skip a generic parameter list, e.g. BaseBehavior<T = unknown>
    while i < len(dts) and dts[i].isspace():
        i += 1
    if i < len(dts) and dts[i] == "<":
        while i < len(dts):
            depth += {"<": 1, ">": -1}.get(dts[i], 0)
            i += 1
            if depth == 0:
                break
    if m.group(1) == "type":
        eq = dts.index("=", i)
        j, depth = eq + 1, 0
        while j < len(dts) and not (dts[j] == ";" and depth == 0):
            if dts[j] == ">" and dts[j - 1] == "=":  # an arrow, not a closing generic
                j += 1
                continue
            depth += {"{": 1, "}": -1, "(": 1, ")": -1, "<": 1, ">": -1}.get(dts[j], 0)
            j += 1
        expr = dts[eq + 1 : j]
        bodies, bases, k, depth, start = [], [], 0, 0, None
        for k, c in enumerate(expr):
            if c == "{":
                if depth == 0:
                    start = k + 1
                depth += 1
            elif c == "}":
                depth -= 1
                if depth == 0 and start is not None:
                    bodies.append(expr[start:k])
        top = re.sub(r"\{.*\}", "", expr, flags=re.S)
        bases = [b for b in re.findall(r"\b([A-Z]\w*)\s*(?:<|&|$)", top)]
        return "\n".join(bodies), bases
    brace = dts.index("{", i)
    header = dts[i:brace]
    bases = (
        re.findall(r"\b([A-Z]\w*)\b", re.sub(r"<[^<>]*>", "", header.split("extends", 1)[-1]))
        if ("extends" in header)
        else []
    )
    j, depth = brace, 0
    while True:
        depth += {"{": 1, "}": -1}.get(dts[j], 0)
        j += 1
        if depth == 0:
            break
    return dts[brace + 1 : j - 1], bases


def members(dts: str, name: str, _seen: set | None = None) -> set[str] | None:
    """Member names of ``name`` including what it inherits; ``None`` if undeclared."""
    seen = _seen if _seen is not None else set()
    if name in seen:
        return set()
    seen.add(name)
    decl = _declaration(dts, name)
    if decl is None:
        return None
    body, bases = decl
    flat = body
    while True:  # drop nested object types so only this level's members remain
        nxt = re.sub(r"\{[^{}]*\}", "", flat)
        if nxt == flat:
            break
        flat = nxt
    found = set(
        re.findall(
            r"(?:^|[;\n])\s*(?:(?:readonly|static|private|protected|public|abstract)\s+)*([A-Za-z_$][\w$]*)\??\s*[:(<]",
            flat,
        )
    )
    for base in bases:
        found |= members(dts, base, seen) or set()
    return found


def check_api_surface(dts: str, surface: dict[str, tuple[str, ...]] = API_SURFACE) -> list[str]:
    dts = _strip_comments(dts)
    out = []
    for iface, names in surface.items():
        have = members(dts, iface)
        if have is None:
            out.append(f"contribution-api no longer declares {iface}")
            continue
        out += [f"contribution-api: {iface}.{n} is gone" for n in names if n not in have]
    return out


def read_yaml(text: str) -> dict:
    """The block-YAML subset manifest.yaml uses: nested maps, lists, plain or quoted scalars."""
    toks: list[tuple[int, str]] = []
    for raw in text.splitlines():
        if not raw.strip() or raw.lstrip().startswith("#"):
            continue
        indent, line = len(raw) - len(raw.lstrip()), raw.strip()
        if line == "-" or line.startswith("- "):
            toks.append((indent, "-"))  # a list item: its content sits two columns in
            if line[1:].strip():
                toks.append((indent + 2, line[1:].strip()))
        else:
            toks.append((indent, line))
    if not toks:
        return {}
    value, i = _yaml_block(toks, 0, toks[0][0])
    if i != len(toks) or not isinstance(value, dict):
        raise TrackError(f"manifest.yaml: cannot read {toks[min(i, len(toks) - 1)][1]!r}")
    return value


def _yaml_scalar(v: str) -> str:
    return v[1:-1] if len(v) >= 2 and v[0] in "\"'" and v[-1] == v[0] else v


def _yaml_block(toks: list[tuple[int, str]], i: int, indent: int):
    if toks[i][1] == "-":
        items: list = []
        while i < len(toks) and toks[i] == (indent, "-"):
            i += 1
            if i < len(toks) and toks[i][0] > indent:
                if ":" in toks[i][1]:
                    item, i = _yaml_block(toks, i, toks[i][0])
                else:
                    item, i = _yaml_scalar(toks[i][1]), i + 1
                items.append(item)
            else:
                items.append(None)
        return items, i
    out: dict = {}
    while i < len(toks) and toks[i][0] == indent and toks[i][1] != "-":
        key, sep, value = toks[i][1].partition(":")
        if not sep:
            raise TrackError(f"manifest.yaml: cannot read {toks[i][1]!r}")
        i, key, value = i + 1, key.strip(), value.strip()
        if value:
            out[key] = _yaml_scalar(value)
        elif i < len(toks) and (toks[i][0] > indent or toks[i] == (indent, "-")):
            out[key], i = _yaml_block(toks, i, toks[i][0])
        else:
            out[key] = None
    return out, i


def validate(value, schema: dict, root: dict | None = None, path: str = "$") -> list[str]:
    """A JSON-Schema (2020-12) subset: the keywords UR's manifest spec uses."""
    root = root or schema
    if "$ref" in schema:
        ref = schema["$ref"]
        if not ref.startswith("#/"):
            return [f"{path}: unsupported $ref {ref}"]
        target = root
        for part in ref[2:].split("/"):
            target = target[part]
        return validate(value, target, root, path)
    errs: list[str] = []
    types = {
        "object": dict,
        "array": list,
        "string": str,
        "boolean": bool,
        "integer": int,
        "number": (int, float),
    }
    t = schema.get("type")
    if t and not any(isinstance(value, types[x]) for x in ([t] if isinstance(t, str) else t) if x in types):
        return [f"{path}: expected {t}, got {type(value).__name__}"]
    if "const" in schema and value != schema["const"]:
        errs.append(f"{path}: must be {schema['const']!r}")
    if "enum" in schema and value not in schema["enum"]:
        errs.append(f"{path}: {value!r} not in {schema['enum']}")
    if "not" in schema and not validate(value, schema["not"], root, path):
        errs.append(f"{path}: matches a forbidden schema")
    if isinstance(value, str):
        if "pattern" in schema and not re.search(schema["pattern"], value):
            errs.append(f"{path}: {value!r} does not match {schema['pattern']}")
        if len(value) < schema.get("minLength", 0) or len(value) > schema.get("maxLength", 1 << 30):
            bounds = f"[{schema.get('minLength', 0)}, {schema.get('maxLength')}]"
            errs.append(f"{path}: length {len(value)} outside {bounds}")
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        if value < schema.get("minimum", float("-inf")) or value > schema.get("maximum", float("inf")):
            errs.append(f"{path}: {value} outside [{schema.get('minimum')}, {schema.get('maximum')}]")
    if isinstance(value, list) and "items" in schema:
        for i, v in enumerate(value):
            errs += validate(v, schema["items"], root, f"{path}[{i}]")
    if isinstance(value, dict):
        props = schema.get("properties", {})
        errs += [f"{path}: missing required {k!r}" for k in schema.get("required", []) if k not in value]
        for k, v in value.items():
            if k in props:
                errs += validate(v, props[k], root, f"{path}.{k}")
                continue
            pat = next((s for p, s in schema.get("patternProperties", {}).items() if re.search(p, k)), None)
            if pat is not None:
                errs += validate(v, pat, root, f"{path}.{k}")
            elif schema.get("additionalProperties") is False:
                errs.append(f"{path}: unknown key {k!r}")
            elif isinstance(schema.get("additionalProperties"), dict):
                errs += validate(v, schema["additionalProperties"], root, f"{path}.{k}")
    return errs


def contribution_api_types(src: Sources, version: str) -> str:
    url = f"{src.npm_feed}/@universal-robots/contribution-api/-/contribution-api-{version}.tgz"
    with tarfile.open(fileobj=io.BytesIO(_get_ok(url)), mode="r:gz") as tar:
        for m in tar.getmembers():
            if m.name.endswith("types/universal-robots-contribution-api.d.ts"):
                f = tar.extractfile(m)
                if f:
                    return f.read().decode("utf-8")
    raise TrackError(f"contribution-api {version}: no universal-robots-contribution-api.d.ts in the package")


def compat(target: dict, src: Sources | None = None, urcap: Path = URCAP) -> list[str]:
    """What stops the URCap from working on the pinned release, as far as the SDK
    can tell without a simulator: the API it calls, its manifest, its worker."""
    src = src or Sources()
    sdk = target["sdk"]
    problems = []
    if sdk.get("threads") != WORKER_THREADS:
        problems.append(
            f"the SDK template uses threads {sdk.get('threads')}, the worker speaks {WORKER_THREADS}'s "
            "protocol — re-verify perceptronic-node.worker.js against it"
        )
    problems += check_api_surface(contribution_api_types(src, sdk["contribution_api"]))
    spec = manifest_spec(sdk_zip(src, sdk["tag"], sdk["version"]), sdk["manifest_spec"])
    manifest = read_yaml((urcap / "manifest.yaml").read_text(encoding="utf-8"))
    problems += [
        f"manifest.yaml vs manifest-spec-{sdk['manifest_spec']}: {e}" for e in validate(manifest, spec)
    ]
    if not target["simulator"].get("matches_sdk"):
        problems.append(
            f"the SDK's simulator ({sdk['ursim']}) is not {target['simulator']['image']} by digest — "
            "the notes and the SDK disagree on what this release is"
        )
    return problems


# -- report ---------------------------------------------------------------------------------------


def _neutral(text: str) -> str:
    """Scraped text as inert markdown for a PR/issue body: no @-mentions that ping
    anyone, no raw HTML, no line breaks that could start a new block."""
    text = re.sub(r"\s+", " ", text)
    text = text.replace("<", "&lt;").replace(">", "&gt;")
    return re.sub(r"@(?=\w)", "@\u200b", text)


def report(
    pinned: dict | None, latest: dict | None, notes: dict, reasons: list[str], pending: list[str]
) -> str:
    lines = ["Weekly PolyScope X release check (`urcap/track.py`).", ""]
    if latest:
        px = latest["polyscope_x"]
        lines += [
            f"**Target:** PolyScope X {px['release']} — [release notes]({px['notes']})"
            + (f", released {px['released']}" if px.get("released") else ""),
            f"**Simulator:** `{latest['simulator']['image']}@{latest['simulator']['digest']}`",
            f"**SDK:** {_neutral(latest['sdk']['release'] or '')} "
            f"(`{latest['sdk']['tag']}`, {latest['sdk']['version']})",
            "",
        ]
    if reasons:
        lines += ["What changed:", *[f"- {r}" for r in reasons], ""]
    if pending:
        lines += ["Still waiting on:", *[f"- {p}" for p in pending], ""]
    if pinned and latest:
        rows = [
            (k, pinned.get("sdk", {}).get(k), latest["sdk"].get(k))
            for k in ("version", "contribution_api", "urcap_utils", "threads", "manifest_spec")
        ]
        lines += ["| SDK component | was | now |", "| --- | --- | --- |"]
        lines += [f"| {k} | {a} | {b} |" for k, a, b in rows]
        lines.append("")
    if notes.get("breaking"):
        lines += [
            "**Breaking changes in the notes:**",
            *[f"- {_neutral(b[:400])}" for b in notes["breaking"]],
            "",
        ]
    if notes.get("urcap_sections"):
        lines += ["<details><summary>URCap / API sections of the notes</summary>", ""]
        for s in notes["urcap_sections"]:
            lines += [f"**{_neutral(s['heading'])}** — {_neutral(s['text'])}", ""]
        lines += ["</details>", ""]
    return "\n".join(lines)


# -- CLI ------------------------------------------------------------------------------------------


def _github_output(path: str | None, **values: str) -> None:
    if not path:
        return
    with open(path, "a", encoding="utf-8") as f:
        for k, v in values.items():
            f.write(f"{k}={v}\n")


def main(argv: list[str] | None = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(
        prog="track", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("check", help="exit 1 when the pin is behind UR")
    up = sub.add_parser("update", help="re-resolve and rewrite target.json + the docs' target lines")
    up.add_argument("--report", help="write a markdown summary here")
    up.add_argument("--github-output", default=os.environ.get("GITHUB_OUTPUT"), help=argparse.SUPPRESS)
    sub.add_parser("compat", help="check the URCap against the pinned SDK")
    sub.add_parser("show", help="print the resolved newest release (writes nothing)")
    sub.add_parser("image", help="print the pinned simulator as image@digest")
    args = ap.parse_args(argv)
    src = Sources()
    try:
        if args.cmd == "image":
            sim = load_target(TARGET)["simulator"]
            print(f"{sim['image']}@{sim['digest']}")
            return 0
        if args.cmd == "compat":
            problems = compat(load_target(TARGET), src)
            for p in problems:
                print(p, file=sys.stderr)
            print("compat: ok" if not problems else f"compat: {len(problems)} problem(s)")
            return 1 if problems else 0
        pinned = load_target(TARGET) if TARGET.exists() else {}
        start = pinned.get("polyscope_x", {}).get("minor", FIRST_MINOR)
        # "hold": "10.13" in target.json keeps the pin on that minor's newest patch.
        hold = pinned.get("hold")
        if hold:
            print(f"note: held at {hold} (target.json hold); newer minors not tracked", file=sys.stderr)
        latest, notes, pending = resolve(src, start, hold)
        if latest is not None and hold:
            latest = {"hold": hold, **latest}
        if args.cmd == "show":
            print(json.dumps({"target": latest, "pending": pending, "notes": notes}, indent=2))
            return 0
        reasons = drift(pinned, latest) if latest else []
        if latest is None:
            stale = overdue(notes)
            for p in pending:
                print(("overdue: " if stale else "pending: ") + p, file=sys.stderr)
            if args.cmd == "update":
                _github_output(args.github_output, changed="false", status="overdue" if stale else "pending")
            return 1 if stale else 0
        for p in pending:
            print(f"note: {p}", file=sys.stderr)
        if args.cmd == "check":
            for r in reasons:
                print(r, file=sys.stderr)
            return 1 if reasons else 0
        # update
        if reasons:
            write_target(latest, TARGET)
            patch_docs(latest, DOCS)
        if args.report:
            Path(args.report).write_text(report(pinned, latest, notes, reasons, pending), encoding="utf-8")
        summary = "; ".join(reasons) or f"up to date with {latest['polyscope_x']['release']}"
        _github_output(
            args.github_output,
            changed="true" if reasons else "false",
            status="changed" if reasons else "current",
            release=latest["polyscope_x"]["release"],
            summary=re.sub(r"[\r\n]+", " ", summary)[:300],
        )
        print(summary)
        return 0
    except TrackError as exc:
        print(f"track: {exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
