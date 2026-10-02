#!/usr/bin/env python3
"""Build and install the PolyScope 5 (e-Series) URCap (``.urcap``) — stdlib + a JDK.

A PolyScope 5 URCap is an OSGi bundle jar. UR's SDK builds it with Maven and the
maven-bundle-plugin; this does the same steps with ``javac`` / ``jdeps`` and
:mod:`zipfile`, the way ``urcapx.py`` replaces the PolyScope X npm toolchain:

* ``sdk [--image TAG ...] [--source registry|docker]`` — the URCap API jars are not on
  Maven Central, and UR's SDK download sits behind a login, but every PolyScope 5 ships
  them as OSGi bundles in ``/ursim/GUI/bundle``. This copies the ones the URCap compiles
  against out of the e-Series URSim image of each version into ``target/urcap5-sdk/<tag>/``
  (git-ignored; never committed — they are UR's), with ``sdk.json``: what each jar exports
  and the URCap API version that PolyScope reports. By default it reads the image's layer
  straight from Docker Hub's registry — no Docker, any host; ``--source docker`` uses a
  local Docker instead. No ``--image``: every version ``bundle.properties`` builds against.
* ``package SRC [--out DIR]`` — compiles ``SRC/src`` for Java 8 (PolyScope 5 runs
  1.8.0_371) against the **oldest supported PolyScope's** API jars (``compat.floor``), and
  each ``compat.since.<version>`` group — code that uses newer API, reached only behind a
  runtime check — against that version's jars; derives ``Import-Package`` from the classes
  with ``jdeps`` (every non-``java.*`` package; ``com.ur.urcap.api.*`` as ``[1.0.0,2.0.0)``;
  ``compat.optional-packages`` with ``resolution:=optional``), writes the manifest from
  ``SRC/bundle.properties`` plus the two compatibility flags PolyScope's loader requires,
  embeds ``META-INF/maven/<group>/<artifact>/pom.xml`` naming the ``com.ur.urcap:api``
  version (the file PolyScope 5.10+ reads the API version from; it must be the floor's), and
  zips it reproducibly: sorted entries, fixed timestamps, the manifest first. The jar also
  carries ``META-INF/urcap5-sources.sha256`` — a digest of the sources — so a test can tell
  a stale committed ``dist/`` without a JDK.
* ``check --sdk DIR [--dist JAR]`` — does the URCap work with that PolyScope's API: the
  sources it loads compile against its jars, and every package the committed jar imports
  (optional ones aside) is one it exports. CI runs it for every PolyScope 5 minor.
* ``release-check TAG [--src SRC] [--dist DIR]`` — before a ``urcap5-v<version>`` tag
  is published as a GitHub Release (``.github/workflows/release-urcap5.yml``): the tag's
  version is ``Bundle-Version``, the committed ``dist/`` jar of that version exists, was
  built from the current sources, and passes PolyScope 5's install checks. Prints the
  jar's path and sha256 as JSON; exits 1 with the reason otherwise.
* ``install FILE --container NAME`` — the e-Series URSim image copies ``/urcaps/*.jar``
  into its bundle directory at start (``/entrypoint.sh``), so this copies the jar
  there and restarts the container. On a real robot, install from a USB stick:
  Settings → System → URCaps → ``+``.

    python3 urcap/urcap5.py sdk
    python3 urcap/urcap5.py package urcap/perceptronic-ps5 --out urcap/dist
    python3 urcap/urcap5.py sdk --image 5.12.8 && \
        python3 urcap/urcap5.py check --sdk target/urcap5-sdk/5.12.8
    python3 urcap/urcap5.py install urcap/dist/perceptronic-ps5-0.5.0.urcap \
        --container ur-utils-ursim-e-ur3e
"""

from __future__ import annotations

import argparse
import difflib
import hashlib
import io
import json
import os
import re
import shutil
import subprocess
import sys
import tarfile
import tempfile
import urllib.request
import zipfile
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
SDK_ROOT = REPO_ROOT / "target" / "urcap5-sdk"  # one directory per PolyScope image tag
API_RANGE = "[1.0.0,2.0.0)"
FIXED_TIME = (2026, 1, 1, 0, 0, 0)  # zip entries' timestamp: the build is content-only
DIGEST_ENTRY = "META-INF/urcap5-sources.sha256"
REQUIRED_KEYS = (
    "Bundle-SymbolicName",
    "Bundle-Name",
    "Bundle-Vendor",
    "Bundle-Version",
    "Bundle-Activator",
    "URCapCompatibility-CB3",
    "URCapCompatibility-eSeries",
    "urcap.api.version",
)


class Urcap5Error(RuntimeError):
    pass


# -- properties + manifest ------------------------------------------------------------------


def read_properties(text: str) -> dict[str, str]:
    """``key=value`` lines; ``#`` comments and blank lines ignored."""
    out: dict[str, str] = {}
    for n, line in enumerate(text.splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        key, sep, value = line.partition("=")
        if not sep or not key.strip():
            raise Urcap5Error(f"bundle.properties line {n}: expected key=value")
        out[key.strip()] = value.strip()
    missing = [k for k in REQUIRED_KEYS if not out.get(k)]
    if missing:
        raise Urcap5Error(f"bundle.properties is missing {', '.join(missing)}")
    for flag in ("URCapCompatibility-CB3", "URCapCompatibility-eSeries"):
        if out[flag] not in ("true", "false"):
            raise Urcap5Error(f"{flag} must be true or false")
    if out["URCapCompatibility-CB3"] == out["URCapCompatibility-eSeries"] == "false":
        raise Urcap5Error("a URCap compatible with neither CB3 nor e-Series will not load")
    return out


def manifest_line(key: str, value: str) -> bytes:
    """One JAR-manifest header, wrapped at 72 bytes with a leading-space continuation."""
    raw = f"{key}: {value}".encode()
    lines = [raw[:72]]
    rest = raw[72:]
    while rest:
        lines.append(b" " + rest[:71])
        rest = rest[71:]
    return b"\r\n".join(lines) + b"\r\n"


def import_package(packages: list[str], optional: list[str] | tuple[str, ...] = ()) -> str:
    """``Import-Package`` for the packages the classes reference (``java.*`` is never
    imported in OSGi; the URCap API gets a major-version range; ``optional`` packages —
    newer than the oldest supported PolyScope — must not stop the bundle resolving)."""
    parts = []
    for pkg in sorted(set(packages)):
        if pkg.startswith("java.") or pkg == "java":
            continue
        clause = f'{pkg};version="{API_RANGE}"' if pkg.startswith("com.ur.urcap.api") else pkg
        parts.append(clause + (";resolution:=optional" if pkg in optional else ""))
    return ",".join(parts)


def build_manifest(
    props: dict[str, str], packages: list[str], optional: list[str] | tuple[str, ...] = ()
) -> bytes:
    headers = [
        ("Manifest-Version", "1.0"),
        ("Bundle-ManifestVersion", "2"),
        ("Bundle-SymbolicName", props["Bundle-SymbolicName"]),
        ("Bundle-Name", props["Bundle-Name"]),
        ("Bundle-Vendor", props["Bundle-Vendor"]),
        ("Bundle-Version", props["Bundle-Version"]),
        ("Bundle-Activator", props["Bundle-Activator"]),
        # PolyScope 5's installer refuses a jar without it (URCapsServiceImpl.isValidFile).
        ("Bundle-Category", "URCap"),
        ("Bundle-RequiredExecutionEnvironment", "JavaSE-1.8"),
        ("Import-Package", import_package(packages, optional)),
        ("URCapCompatibility-CB3", props["URCapCompatibility-CB3"]),
        ("URCapCompatibility-eSeries", props["URCapCompatibility-eSeries"]),
    ]
    if props.get("Bundle-Description"):
        headers.insert(6, ("Bundle-Description", props["Bundle-Description"]))
    if props.get("Bundle-Copyright"):
        headers.insert(5, ("Bundle-Copyright", props["Bundle-Copyright"]))
    return b"".join(manifest_line(k, v) for k, v in headers) + b"\r\n"


def pom_xml(props: dict[str, str]) -> tuple[str, bytes]:
    """The embedded pom PolyScope 5 reads the URCap API version from."""
    sym = props["Bundle-SymbolicName"]
    group, _, artifact = sym.rpartition(".")
    group = group or sym
    body = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<project xmlns="http://maven.apache.org/POM/4.0.0">\n'
        "  <modelVersion>4.0.0</modelVersion>\n"
        f"  <groupId>{group}</groupId>\n"
        f"  <artifactId>{artifact}</artifactId>\n"
        f"  <version>{props['Bundle-Version']}</version>\n"
        "  <packaging>bundle</packaging>\n"
        f"  <name>{props['Bundle-Name']}</name>\n"
        "  <dependencies>\n"
        "    <dependency>\n"
        "      <groupId>com.ur.urcap</groupId>\n"
        "      <artifactId>api</artifactId>\n"
        f"      <version>{props['urcap.api.version']}</version>\n"
        "      <scope>provided</scope>\n"
        "    </dependency>\n"
        "  </dependencies>\n"
        "</project>\n"
    )
    return f"META-INF/maven/{group}/{artifact}/pom.xml", body.encode()


# -- sources + compatibility plan -----------------------------------------------------------


def sources_digest(src: Path) -> str:
    """SHA-256 over ``bundle.properties`` and every ``src/**/*.java`` (path + bytes, sorted)."""
    h = hashlib.sha256()
    files = [src / "bundle.properties", *sorted((src / "src").rglob("*.java"))]
    for f in files:
        rel = f.relative_to(src).as_posix()
        h.update(rel.encode() + b"\0" + f.read_bytes() + b"\0")
    return h.hexdigest()


def version_key(version: str) -> tuple[int, ...]:
    """``"5.8"`` / ``"5.10.2"`` / ``"1.19.0"`` → ``(5, 8)`` / ``(5, 10, 2)`` — numeric, never lexical."""
    if not re.fullmatch(r"[0-9]{1,4}(\.[0-9]{1,4}){1,3}", version or ""):  # not \d: it takes any script
        raise Urcap5Error(f"{version!r} is not a version like 5.8 or 5.10.2")
    return tuple(int(p) for p in version.split("."))


def compat_plan(props: dict[str, str]) -> dict:
    """What ``bundle.properties`` says about the PolyScope versions the URCap runs on:

    * ``compat.floor`` — the oldest PolyScope 5 it supports. Everything is compiled against
      that release's URCap API jars, so a call the floor lacks fails the build, not a
      pendant.
    * ``compat.since.<version>`` — source files (under ``src/``) that use API newer than the
      floor. They are compiled against that version's jars, and the rest of the code only
      reaches them behind a runtime check (by class name, never a static reference).
    * ``compat.optional-packages`` — the packages only those files import: OSGi must not
      refuse the bundle on a PolyScope that lacks them (``resolution:=optional``).
    """
    floor = props.get("compat.floor", "")
    version_key(floor)
    since: dict[str, list[str]] = {}
    for key, value in props.items():
        if key.startswith("compat.since."):
            v = key[len("compat.since.") :]
            if version_key(v) <= version_key(floor):
                raise Urcap5Error(f"{key}: {v} is not newer than compat.floor={floor}")
            files = [f.strip() for f in value.split(",") if f.strip()]
            if not files:
                raise Urcap5Error(f"{key} names no source file")
            since[v] = files
    optional = sorted(p.strip() for p in props.get("compat.optional-packages", "").split(",") if p.strip())
    return {
        "floor": floor,
        "since": dict(sorted(since.items(), key=lambda kv: version_key(kv[0]))),
        "optional": optional,
    }


def sources_for(src: Path, plan: dict, version: str | None = None) -> list[Path]:
    """The sources a PolyScope ``version`` loads: every ``src/**/*.java`` except the
    ``compat.since`` groups newer than it (``None``: the floor — only the base code)."""
    base = src / "src"
    every = sorted(base.rglob("*.java"))
    known = {p.relative_to(base).as_posix() for p in every}
    target = version_key(version) if version else version_key(plan["floor"])
    skip: set[str] = set()
    for v, files in plan["since"].items():
        for f in files:
            if f not in known:
                raise Urcap5Error(f"compat.since.{v} names {f}, which is not under {base}")
        if version_key(v) > target:
            skip.update(files)
    return [p for p in every if p.relative_to(base).as_posix() not in skip]


# -- the URCap API jars out of a PolyScope 5 URSim image --------------------------------------
#
# The URCap API is not on Maven Central and UR's SDK download sits behind a login, but every
# PolyScope 5 ships it as OSGi bundles in /ursim/GUI/bundle — under names that change with the
# release (5.4: polyscope-urcap/api-1.7.0.jar; 5.5+: polyscope-urcap-urcap-api-*.jar plus a
# growing set of *-urcap-api* / urcap-api-export-* bundles, some in subdirectories up to 5.7).
# So the jars are picked by what they export, not by name: every bundle exporting a
# com.ur.urcap.api package, and the bundle exporting the newest org.osgi.framework (5.13+
# ship both org.osgi.core-4.1.0 and osgi.core-6.0.0; the old one has no generics).

IMAGE_REPO = "universalrobots/ursim_e-series"
SDK_INFO = "sdk.json"
_REGISTRY = "https://registry-1.docker.io/v2/"
_TOKEN = "https://auth.docker.io/token?service=registry.docker.io&scope=repository:{repo}:pull"
_ACCEPT = ",".join(
    (
        "application/vnd.docker.distribution.manifest.v2+json",
        "application/vnd.oci.image.manifest.v1+json",
        "application/vnd.docker.distribution.manifest.list.v2+json",
        "application/vnd.oci.image.index.v1+json",
    )
)
_BUNDLE = "ursim/GUI/bundle/"
_PROVIDER = re.compile(r"PackageDependencyProvider(\d+)_(\d+)_(\d+)Impl\.class$")


def split_image(image: str) -> tuple[str, str]:
    """``5.4`` / ``ursim_e-series:5.4`` / ``universalrobots/ursim_e-series:5.4`` → (repo, tag)."""
    repo, _, tag = image.rpartition(":")
    if not repo:
        repo = IMAGE_REPO
    elif "/" not in repo:
        repo = "universalrobots/" + repo
    if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]{0,127}", tag):  # a Docker tag, ASCII only
        raise Urcap5Error(f"{image!r} has no image tag")
    return repo, tag


def split_clauses(header: str) -> list[str]:
    """An OSGi header's clauses: split on commas outside double quotes."""
    parts, cur, quoted = [], "", False
    for ch in header:
        if ch == '"':
            quoted = not quoted
        if ch == "," and not quoted:
            parts.append(cur)
            cur = ""
        else:
            cur += ch
    parts.append(cur)
    return [p for p in parts if p.strip()]


def parse_exports(header: str) -> dict[str, str | None]:
    """``Export-Package`` → ``{package: version or None}``."""
    out: dict[str, str | None] = {}
    for part in split_clauses(header):
        name = part.split(";", 1)[0].strip()
        m = re.search(r';\s*version\s*=\s*"?([0-9][0-9A-Za-z.\-_]*)', part)
        out[name] = m.group(1) if m else None
    return out


def _manifest(z: zipfile.ZipFile) -> dict[str, str]:
    raw = z.read("META-INF/MANIFEST.MF").decode("utf-8", errors="replace")
    headers: dict[str, str] = {}
    key = None
    for line in raw.replace("\r\n", "\n").split("\n"):
        if line.startswith(" ") and key:
            headers[key] += line[1:]
        elif ": " in line:
            key, _, value = line.partition(": ")
            headers[key] = value
    return headers


def scan_bundles(files) -> dict:
    """From ``(path, bytes)`` of every jar in a PolyScope's bundle directory: the jars the
    URCap compiles against, what each exports, and the URCap API version this PolyScope
    reports.

    The API version: PolyScope 5.10+ refuses a URCap whose embedded pom names a newer
    ``com.ur.urcap:api`` than ``URCapAPIPackageRepository.getlatestAPIVersionAvailable()``
    (``URCapBundleCompatibilityValidatorImpl.checkVersionCompatibility``, polyscope-urcap-impl)
    — the newest ``PackageDependencyProvider<x_y_z>Impl`` that impl jar carries. 5.4 ships the
    API as one Maven artifact, ``polyscope-urcap/api-1.7.0.jar`` (pom.properties
    ``com.ur.urcap:api:1.7.0``). 5.5–5.9 record it in neither place (and read no pom): None.
    """
    api: dict[str, bytes] = {}
    exports: dict[str, dict[str, str | None]] = {}
    osgi: list[tuple[tuple[int, ...], str, bytes]] = []
    providers: set[tuple[int, int, int]] = set()
    maven_api: str | None = None
    for path, data in files:
        try:
            z = zipfile.ZipFile(io.BytesIO(data))
            names = z.namelist()
            headers = _manifest(z)
        except (zipfile.BadZipFile, KeyError, OSError):
            continue
        for n in names:
            m = _PROVIDER.search(n)
            if m:
                providers.add((int(m.group(1)), int(m.group(2)), int(m.group(3))))
            if n.endswith("pom.properties"):
                pom = dict(
                    line.split("=", 1)
                    for line in z.read(n).decode(errors="replace").splitlines()
                    if "=" in line
                )
                if pom.get("groupId") == "com.ur.urcap" and pom.get("artifactId") == "api":
                    maven_api = pom.get("version")
        exp = parse_exports(headers.get("Export-Package", ""))
        name = path.rsplit("/", 1)[-1]
        if any(p.startswith("com.ur.urcap.api") for p in exp):
            api[name] = data
            exports[name] = {p: v for p, v in exp.items() if p.startswith("com.ur.urcap.api")}
        if "org.osgi.framework" in exp:
            v = exp["org.osgi.framework"] or "0"
            osgi.append((tuple(int(x) for x in re.findall(r"\d+", v)[:3]), name, data))
    if not api:
        raise Urcap5Error("no bundle exporting com.ur.urcap.api — not a PolyScope 5 bundle directory?")
    if not osgi:
        raise Urcap5Error("no bundle exporting org.osgi.framework")
    osgi_version, osgi_name, osgi_data = max(osgi)
    jars = {**api, osgi_name: osgi_data}
    exports[osgi_name] = {"org.osgi.framework": ".".join(map(str, osgi_version))}
    api_version = ".".join(map(str, max(providers))) if providers else maven_api
    return {"jars": jars, "exports": exports, "api_version": api_version}


def _http(url: str, headers: dict[str, str] | None = None):
    req = urllib.request.Request(url, headers=headers or {})
    return urllib.request.urlopen(req, timeout=120)  # noqa: S310 — fixed https registry URLs


def registry_image(image: str) -> tuple[str, str, dict, dict, dict]:
    """``(repo, tag, auth headers, manifest, config)`` of ``image``'s amd64 image, from Docker
    Hub's registry — the config holds its history and its environment (``VERSION``)."""
    repo, tag = split_image(image)
    with _http(_TOKEN.format(repo=repo)) as r:
        token = json.load(r)["token"]
    auth = {"Authorization": f"Bearer {token}"}
    with _http(f"{_REGISTRY}{repo}/manifests/{tag}", {**auth, "Accept": _ACCEPT}) as r:
        manifest = json.load(r)
    if "manifests" in manifest:  # an index: the amd64 image
        amd64 = [m for m in manifest["manifests"] if m.get("platform", {}).get("architecture") == "amd64"]
        if not amd64:
            raise Urcap5Error(f"{repo}:{tag} has no amd64 image")
        with _http(f"{_REGISTRY}{repo}/manifests/{amd64[0]['digest']}", {**auth, "Accept": _ACCEPT}) as r:
            manifest = json.load(r)
    with _http(f"{_REGISTRY}{repo}/blobs/{manifest['config']['digest']}", auth) as r:
        config = json.load(r)
    return repo, tag, auth, manifest, config


def image_version(image: str) -> str | None:
    """The PolyScope version ``image`` carries: its ``VERSION`` environment variable (URSim
    images set it to the full ``5.x.y`` even when the tag is a bare ``5.x``), or None."""
    env = registry_image(image)[4].get("config", {}).get("Env") or []
    return next((e.split("=", 1)[1] for e in env if e.startswith("VERSION=")), None)


def registry_bundle_files(image: str):
    """Every jar under ``/ursim/GUI/bundle`` of ``image``, straight from Docker Hub's registry
    (no Docker needed; the image is amd64-only and needs not run): the layer the image's
    history says ``COPY ursim_<version> /ursim`` made, streamed, nothing else downloaded."""
    repo, tag, auth, manifest, config = registry_image(image)
    history = [h for h in config.get("history", []) if not h.get("empty_layer")]
    layers = manifest["layers"]
    picks = [i for i, h in enumerate(history) if "COPY ursim_" in h.get("created_by", "")]
    if len(history) != len(layers) or len(picks) != 1:
        raise Urcap5Error(f"{repo}:{tag}: no single `COPY ursim_*` layer in its history")
    with _http(f"{_REGISTRY}{repo}/blobs/{layers[picks[0]]['digest']}", auth) as r:
        with tarfile.open(fileobj=r, mode="r|gz") as tar:
            for member in tar:
                if member.isfile() and member.name.startswith(_BUNDLE) and member.name.endswith(".jar"):
                    f = tar.extractfile(member)
                    if f is not None:
                        yield member.name[len(_BUNDLE) :], f.read()


def docker_bundle_files(image: str):
    """The same jars via a local Docker that already has (or can pull) the image: ``docker
    create`` + ``docker cp`` — nothing runs, so an amd64 image works on any host."""
    docker = os.environ.get("DOCKER", "docker").split()
    if not shutil.which(docker[-1]) and not shutil.which(docker[0]):
        raise Urcap5Error("docker is not on PATH (or use --source registry)")
    repo, tag = split_image(image)
    cid = subprocess.run(
        [*docker, "create", "--platform", "linux/amd64", f"{repo}:{tag}"],
        capture_output=True,
        text=True,
        check=False,
    )
    if cid.returncode != 0:
        raise Urcap5Error(f"docker create {repo}:{tag} failed: {cid.stderr.strip()}")
    container = cid.stdout.strip()
    try:
        with tempfile.TemporaryDirectory() as tmp:
            cp = subprocess.run(
                [*docker, "cp", f"{container}:/{_BUNDLE}", tmp], capture_output=True, text=True, check=False
            )
            if cp.returncode != 0:
                raise Urcap5Error(f"docker cp failed: {cp.stderr.strip()}")
            root = Path(tmp) / "bundle"
            for jar in sorted(root.rglob("*.jar")):
                yield jar.relative_to(root).as_posix(), jar.read_bytes()
    finally:
        subprocess.run([*docker, "rm", container], capture_output=True, check=False)


def fetch_sdk(image: str, dest: Path | None = None, *, source: str = "registry") -> dict:
    """Write ``image``'s URCap API jars and ``sdk.json`` (what each exports, the API version)
    into ``dest`` (default ``target/urcap5-sdk/<tag>/``)."""
    repo, tag = split_image(image)
    dest = dest or SDK_ROOT / tag
    files = registry_bundle_files(image) if source == "registry" else docker_bundle_files(image)
    found = scan_bundles(files)
    dest.mkdir(parents=True, exist_ok=True)
    for old in dest.glob("*.jar"):
        old.unlink()
    for name, data in sorted(found["jars"].items()):
        (dest / name).write_bytes(data)
    info = {
        "image": f"{repo}:{tag}",
        "api_version": found["api_version"],
        "jars": sorted(found["jars"]),
        "exports": found["exports"],
    }
    (dest / SDK_INFO).write_text(json.dumps(info, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    return {**info, "dir": str(dest)}


def sdk_info(sdk_dir: Path) -> dict:
    """``sdk.json`` of a fetched SDK directory, or Urcap5Error saying how to fetch it."""
    info = sdk_dir / SDK_INFO
    if not info.is_file() or not any(sdk_dir.glob("*.jar")):
        raise Urcap5Error(
            f"no URCap API jars in {sdk_dir} — run `python3 urcap/urcap5.py sdk --image {sdk_dir.name}` first"
        )
    return json.loads(info.read_text(encoding="utf-8"))


def sdk_jars(sdk_dir: Path) -> list[Path]:
    sdk_info(sdk_dir)
    return sorted(sdk_dir.glob("*.jar"))


# -- package --------------------------------------------------------------------------------


def _java_tool(name: str) -> str:
    tool = shutil.which(name)
    if not tool:
        raise Urcap5Error(f"{name} is not on PATH (a JDK 11+ builds for Java 8 with --release 8)")
    return tool


def referenced_packages(classes: Path, classpath: list[Path]) -> list[str]:
    """Packages the compiled classes use, from ``jdeps -verbose:package``."""
    out = subprocess.run(
        [
            _java_tool("jdeps"),
            "-verbose:package",
            "-cp",
            os.pathsep.join(str(p) for p in classpath),
            str(classes),
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if out.returncode != 0:
        raise Urcap5Error(f"jdeps failed: {out.stderr.strip() or out.stdout.strip()}")
    pkgs = set()
    for line in out.stdout.splitlines():
        if "->" not in line:
            continue
        target = line.split("->", 1)[1].split()
        if target and "." in target[0] and "/" not in target[0] and not target[0].endswith(".jar"):
            pkgs.add(target[0])
    return sorted(pkgs)


def javac(sources: list[Path], classpath: list[Path], out: Path) -> None:
    """``javac --release 8`` with every lint as an error — what the URCap is built with."""
    cc = subprocess.run(
        [
            _java_tool("javac"),
            "--release",
            "8",
            "-Xlint:all",
            "-Xlint:-options",
            "-Werror",
            "-encoding",
            "UTF-8",
            "-cp",
            os.pathsep.join(str(j) for j in classpath),
            "-d",
            str(out),
            *[str(f) for f in sources],
        ],
        capture_output=True,
        text=True,
        check=False,
    )
    if cc.returncode != 0:
        raise Urcap5Error("javac failed:\n" + (cc.stderr or cc.stdout))


def check_api_version(props: dict[str, str], floor_info: dict) -> None:
    """``urcap.api.version`` must be what the floor PolyScope reports (when it reports one):
    newer and PolyScope 5.10+ refuse the URCap below that version; older says less than true."""
    want = floor_info.get("api_version")
    if want and props["urcap.api.version"] != want:
        raise Urcap5Error(
            f"urcap.api.version={props['urcap.api.version']} but compat.floor's PolyScope "
            f"({floor_info.get('image')}) reports URCap API {want}"
        )


def package(src: str | Path, out_dir: str | Path, *, sdk_root: Path | None = None) -> Path:
    """Build ``src`` into a ``.urcap``: the base code against the floor PolyScope's API jars,
    each ``compat.since`` group against its version's jars on top of the base classes."""
    src, out_dir = Path(src), Path(out_dir)
    sdk_root = sdk_root or SDK_ROOT
    props = read_properties((src / "bundle.properties").read_text(encoding="utf-8"))
    plan = compat_plan(props)
    base = sources_for(src, plan)
    if not base:
        raise Urcap5Error(f"no Java sources under {src / 'src'}")
    floor_dir = sdk_root / plan["floor"]
    check_api_version(props, sdk_info(floor_dir))
    used = sdk_jars(floor_dir)
    own = props["Bundle-SymbolicName"].rpartition(".")[0]
    with tempfile.TemporaryDirectory() as tmp:
        classes = Path(tmp) / "classes"
        classes.mkdir()
        javac(base, used, classes)
        for version, files in plan["since"].items():
            jars = sdk_jars(sdk_root / version)
            javac([src / "src" / f for f in files], [classes, *jars], classes)
            used += [j for j in jars if j not in used]
        packages = [p for p in referenced_packages(classes, used) if not p.startswith(own)]
        stale = [p for p in plan["optional"] if p not in packages]
        if stale:
            raise Urcap5Error(f"compat.optional-packages names {', '.join(stale)}, which no class imports")
        entries: list[tuple[str, bytes]] = [
            (p.relative_to(classes).as_posix(), p.read_bytes()) for p in sorted(classes.rglob("*.class"))
        ]
    pom_path, pom = pom_xml(props)
    licence = src / "LICENSE" if (src / "LICENSE").is_file() else REPO_ROOT / "LICENSE"
    entries += [(pom_path, pom), (DIGEST_ENTRY, (sources_digest(src) + "\n").encode())]
    if licence.is_file():
        entries.append(("META-INF/LICENSE", licence.read_bytes()))
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / dist_name(props)
    tmp_out = out.with_suffix(".urcap.tmp")
    with zipfile.ZipFile(tmp_out, "w", zipfile.ZIP_DEFLATED) as z:
        z.writestr(zipfile.ZipInfo("META-INF/", FIXED_TIME), b"")
        _write(z, "META-INF/MANIFEST.MF", build_manifest(props, packages, plan["optional"]))
        for path, data in sorted(entries):
            _write(z, path, data)
    tmp_out.replace(out)
    return out


def imports_of(bundle: dict) -> dict[str, dict[str, str]]:
    """A built bundle's ``Import-Package`` → ``{package: {attribute or directive: value}}``."""
    out: dict[str, dict[str, str]] = {}
    for clause in split_clauses(bundle["headers"].get("Import-Package", "")):
        name, *attrs = [a.strip() for a in clause.split(";")]
        pairs = (a.partition("=") for a in attrs)
        out[name] = {k.strip().rstrip(":"): v.strip().strip('"') for k, _, v in pairs}
    return out


def missing_imports(imports: dict[str, dict[str, str]], exported: set[str]) -> list[str]:
    """The URCap API / OSGi packages a bundle requires (not ``resolution:=optional``) that a
    PolyScope doesn't export — any one of them and OSGi never resolves the bundle."""
    return sorted(
        p
        for p, attrs in imports.items()
        if p.startswith(("com.ur.urcap.api", "org.osgi."))
        and attrs.get("resolution") != "optional"
        and p not in exported
    )


def check(src: str | Path, sdk_dir: Path, dist: Path, version: str | None = None) -> dict:
    """Does the URCap work with the PolyScope whose API jars are in ``sdk_dir``?

    1. the sources that PolyScope loads (:func:`sources_for` its version) compile against its
       jars — an API it lacks is a compile error here instead of a NoSuchMethodError on a
       pendant;
    2. every package the built ``dist`` jar imports without ``resolution:=optional`` is one
       this PolyScope exports — else OSGi never starts the bundle.
    """
    src = Path(src)
    info = sdk_info(sdk_dir)
    version = version or split_image(info["image"])[1]
    props = read_properties((src / "bundle.properties").read_text(encoding="utf-8"))
    plan = compat_plan(props)
    if version_key(version) < version_key(plan["floor"]):
        raise Urcap5Error(f"PolyScope {version} is older than compat.floor={plan['floor']}")
    sources = sources_for(src, plan, version)
    with tempfile.TemporaryDirectory() as tmp:
        javac(sources, sdk_jars(sdk_dir), Path(tmp))
    exported = {p for pkgs in info["exports"].values() for p in pkgs}
    missing = missing_imports(imports_of(read_bundle(dist)), exported)
    if missing:
        raise Urcap5Error(
            f"{dist.name} imports {', '.join(missing)}, which PolyScope {version} does not export"
        )
    return {
        "version": version,
        "image": info["image"],
        "api_version": info.get("api_version"),
        "compiled": len(sources),
        "skipped": sorted(
            f for v, files in plan["since"].items() if version_key(v) > version_key(version) for f in files
        ),
    }


def dist_name(props: dict[str, str]) -> str:
    """The jar's file name for these bundle properties: ``perceptronic-ps5-0.5.0.urcap``."""
    artifact = props["Bundle-SymbolicName"].rpartition(".")[2]
    name = f"{artifact}-ps5"
    return f"{name}-{props['Bundle-Version']}.urcap"


def _write(z: zipfile.ZipFile, path: str, data: bytes) -> None:
    info = zipfile.ZipInfo(path, FIXED_TIME)
    info.compress_type = zipfile.ZIP_DEFLATED
    info.external_attr = 0o644 << 16
    z.writestr(info, data)


def read_bundle(path: str | Path) -> dict:
    """The manifest headers, the embedded API version and the sources digest of a ``.urcap``."""
    with zipfile.ZipFile(path) as z:
        names = z.namelist()
        raw = z.read("META-INF/MANIFEST.MF").decode()
        headers: dict[str, str] = {}
        key = None
        for line in raw.replace("\r\n", "\n").split("\n"):
            if line.startswith(" ") and key:
                headers[key] += line[1:]
            elif ": " in line:
                key, _, value = line.partition(": ")
                headers[key] = value
        digest = z.read(DIGEST_ENTRY).decode().strip() if DIGEST_ENTRY in names else None
        poms = [n for n in names if n.startswith("META-INF/maven/") and n.endswith("/pom.xml")]
        pom = z.read(poms[0]).decode() if poms else ""
    return {"headers": headers, "names": names, "sources_sha256": digest, "pom": pom}


# -- release --------------------------------------------------------------------------------

TAG_PREFIX = "urcap5-v"
_SEMVER = re.compile(r"\d{1,4}\.\d{1,4}\.\d{1,4}")


def release_check(tag: str, src: str | Path, dist_dir: str | Path) -> dict:
    """What a ``urcap5-v<version>`` tag may publish: ``{version, path, sha256}``, or
    Urcap5Error naming what is wrong. Everything is checked against the committed jar —
    CI cannot rebuild it (the URCap API jars are UR's and live only in the URSim image)
    and needs not: the build is reproducible and the jar carries its sources' digest."""
    src, dist_dir = Path(src), Path(dist_dir)
    if not tag.startswith(TAG_PREFIX) or not _SEMVER.fullmatch(tag[len(TAG_PREFIX) :]):
        raise Urcap5Error(f"tag {tag!r} is not {TAG_PREFIX}<major>.<minor>.<patch>")
    version = tag[len(TAG_PREFIX) :]
    props = read_properties((src / "bundle.properties").read_text(encoding="utf-8"))
    if props["Bundle-Version"] != version:
        raise Urcap5Error(
            f"tag {tag} but {src / 'bundle.properties'} says Bundle-Version={props['Bundle-Version']}"
        )
    path = dist_dir / dist_name(props)
    if not path.is_file():
        raise Urcap5Error(f"{path} is not committed — run `make urcap5-package` and commit it")
    bundle = read_bundle(path)
    h = bundle["headers"]
    if h.get("Bundle-Version") != version:
        raise Urcap5Error(f"{path} is Bundle-Version {h.get('Bundle-Version')}, not {version}")
    if bundle["sources_sha256"] != sources_digest(src):
        raise Urcap5Error(f"{path} was not built from the current sources — run `make urcap5-package`")
    if h.get("Bundle-Category", "").lower() != "urcap" or "META-INF/MANIFEST.MF" not in bundle["names"][:2]:
        raise Urcap5Error(f"{path} would be refused by PolyScope 5's installer (category / manifest order)")
    # the USB stick's auto-install file is released beside the jar: it must be the one for this jar
    stick = dist_dir / MAGIC_NAME
    if not stick.is_file():
        raise Urcap5Error(f"{stick} is not committed — run `make urcap5-package` and commit it")
    template = MAGIC_TEMPLATE.read_text(encoding="utf-8")
    if stick.read_text(encoding="utf-8") != render_magic(template, path, props["Bundle-SymbolicName"]):
        raise Urcap5Error(f"{stick} is not the one for {path.name} — run `make urcap5-package`")
    return {
        "version": version,
        "path": str(path),
        "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "stick": str(stick),
    }


# -- install --------------------------------------------------------------------------------


def install(path: str | Path, container: str) -> dict:
    """Copy the jar into the URSim container's ``/urcaps`` and restart it (its entrypoint
    copies ``/urcaps/*.jar`` into PolyScope's bundle directory at start)."""
    path = Path(path)
    if not path.is_file():
        raise Urcap5Error(f"{path} does not exist")
    for cmd in (
        ["docker", "cp", str(path), f"{container}:/urcaps/{path.stem}.jar"],
        ["docker", "restart", container],
    ):
        r = subprocess.run(cmd, capture_output=True, text=True, check=False)
        if r.returncode != 0:
            raise Urcap5Error(f"{' '.join(cmd[:2])} failed: {r.stderr.strip()}")
    return {"ok": True, "container": container, "jar": f"/urcaps/{path.stem}.jar", "restarted": True}


# -- is a rebuilt jar the committed one? ------------------------------------------------------


_ANONYMOUS = re.compile(r"\$[0-9]+$")


def _member_name(line: str) -> str:
    """The member a ``javap -p`` line declares: the word before ``(`` (a method or constructor)
    or before `` =`` / ``;`` (a field)."""
    head = line.split("(", 1)[0] if "(" in line else line.split(" =", 1)[0].rstrip(";")
    return head.split()[-1] if head.split() else ""


def _source_members(cls: str, lines: list[str]) -> list[str]:
    """What the sources declare, without what javac adds on its own and names differently
    from one JDK major to the next: members named with a ``$`` (``lambda$new$0``, ``this$0``,
    ``val$x``, ``access$000``, ``$values``) and the constructors of anonymous classes (their
    parameters are the captured variables, in javac's order). Each member line keeps its
    ``descriptor:`` line."""
    out: list[str] = []
    skip = False
    for line in lines:
        if line.strip().startswith("descriptor:"):
            if not skip:
                out.append(line)
            continue
        skip = False
        if line.startswith("  ") and line.strip():
            name = _member_name(line.strip())
            simple = name.rsplit(".", 1)[-1]
            ctor = name == cls
            if ("$" in simple and not ctor) or (ctor and _ANONYMOUS.search(cls)):
                skip = True
                continue
        out.append(line)
    return out


def class_signatures(jar: Path, javap: str | None = None) -> dict[str, str]:
    """Each class in ``jar`` as ``javap -p -s -constants`` prints it — every field, method and
    constant with its JVM signature — minus the ``Compiled from`` line and the members javac
    synthesises (:func:`_source_members`). Independent of which javac built it: two JDK majors
    order a constant pool, number lambdas and shape anonymous classes differently, but they
    declare the same members."""
    javap = javap or shutil.which("javap")
    if not javap:
        raise Urcap5Error("javap is not on PATH (it ships with the JDK)")
    with zipfile.ZipFile(jar) as z:
        names = sorted(n[: -len(".class")].replace("/", ".") for n in z.namelist() if n.endswith(".class"))
    if not names:
        return {}
    out = subprocess.run(
        [javap, "-p", "-s", "-constants", "-cp", str(jar), *names], capture_output=True, text=True, check=True
    ).stdout
    sigs: dict[str, str] = {}
    current: str | None = None
    lines: list[str] = []

    def flush() -> None:
        if current is not None:
            sigs[current] = "\n".join(_source_members(current, lines))

    for line in out.splitlines():
        if line.startswith("Compiled from"):
            flush()
            current, lines = None, []
            continue
        if current is None:
            m = re.search(r"(?:class|interface|enum)\s+([\w.$]+)", line)
            if m:
                current = m.group(1)
        lines.append(line.rstrip())
    flush()
    return sigs


def compare_jars(built: Path, committed: Path, javap: str | None = None) -> list[str]:
    """Why ``built`` is not ``committed``, one line each; ``[]`` when it is. The same entries;
    every entry that is not a class (the manifest, the embedded pom, the sources digest)
    byte-identical; every class declaring the same members (:func:`class_signatures`). Method
    bodies are held by the sources digest — any edited ``.java`` changes it."""
    problems: list[str] = []
    with zipfile.ZipFile(built) as a, zipfile.ZipFile(committed) as b:
        na, nb = set(a.namelist()), set(b.namelist())
        for n in sorted(na - nb):
            problems.append(f"{n}: in the rebuild, not in the committed jar")
        for n in sorted(nb - na):
            problems.append(f"{n}: in the committed jar, not in the rebuild")
        for n in sorted(na & nb):
            if not n.endswith(".class") and a.read(n) != b.read(n):
                problems.append(f"{n}: differs")
    if problems:
        return problems
    sa, sb = class_signatures(built, javap), class_signatures(committed, javap)
    for cls in sorted(set(sa) | set(sb)):
        if sa.get(cls) != sb.get(cls):
            diff = difflib.unified_diff(
                (sb.get(cls) or "").splitlines(),
                (sa.get(cls) or "").splitlines(),
                "committed",
                "rebuilt",
                n=0,
                lineterm="",
            )
            shown = [d for d in diff if d[:1] in "+-" and not d.startswith(("+++", "---"))][:6]
            problems.append(f"{cls}: its members differ " + " | ".join(shown))
    return problems


MAGIC_TEMPLATE = REPO_ROOT / "scripts" / "urmagic_perceptronic.sh"
MAGIC_NAME = "urmagic_perceptronic.sh"


def render_magic(template: str, urcap: Path, symbolic_name: str) -> str:
    """The USB stick's auto-install file for one built jar: scripts/urmagic_perceptronic.sh
    with the jar's file name, sha256 and bundle id filled in (what scripts/urcap5-usb.sh
    writes on the stick) — committed beside the jar so a stick can be made by copying two files."""
    values = {
        "@URCAP_FILE@": urcap.name,
        "@URCAP_SHA256@": hashlib.sha256(urcap.read_bytes()).hexdigest(),
        "@SYMBOLIC_NAME@": symbolic_name,
    }
    for key, value in values.items():
        if key not in template:
            raise Urcap5Error(f"{MAGIC_TEMPLATE.name} has no {key}")
        template = template.replace(key, value)
    return template


def write_magic(src: str | Path, out: str | Path) -> Path:
    props = read_properties((Path(src) / "bundle.properties").read_text(encoding="utf-8"))
    urcap = Path(out) / dist_name(props)
    if not urcap.is_file():
        raise Urcap5Error(f"{urcap} is not built — run `make urcap5-package`")
    path = Path(out) / MAGIC_NAME
    text = render_magic(MAGIC_TEMPLATE.read_text(encoding="utf-8"), urcap, props["Bundle-SymbolicName"])
    path.write_bytes(text.encode("utf-8"))
    return path


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(prog="urcap5", description=__doc__.split("\n\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sd = sub.add_parser("sdk", help="copy the URCap API jars out of PolyScope 5 URSim images")
    sd.add_argument(
        "--image", action="append", help="image or tag (repeatable); default: what the build needs"
    )
    sd.add_argument("--src", default="urcap/perceptronic-ps5")
    sd.add_argument("--source", choices=("registry", "docker"), default="registry")
    sd.add_argument("--dir", help="where to put them (one --image only); default target/urcap5-sdk/<tag>")
    pk = sub.add_parser("package", help="build SRC into a .urcap")
    pk.add_argument("src")
    pk.add_argument("--out", default="urcap/dist")
    mg = sub.add_parser("magic", help="write the USB stick's auto-install file for the built .urcap")
    mg.add_argument("src")
    mg.add_argument("--out", default="urcap/dist")
    ck = sub.add_parser("check", help="does the URCap work with the PolyScope whose API jars are in --sdk")
    ck.add_argument("--sdk", required=True, help="a directory `urcap5.py sdk` wrote")
    ck.add_argument("--src", default="urcap/perceptronic-ps5")
    ck.add_argument("--dist", help="the built jar (default: the committed one for this version)")
    ck.add_argument("--version", help="the PolyScope version (default: the image tag)")
    cj = sub.add_parser("compare", help="is a rebuilt jar the committed one (entries, bytes, class members)")
    cj.add_argument("built", type=Path)
    cj.add_argument("committed", type=Path)
    rc = sub.add_parser("release-check", help="check a urcap5-v<version> tag against the committed jar")
    rc.add_argument("tag")
    rc.add_argument("--src", default="urcap/perceptronic-ps5")
    rc.add_argument("--dist", default="urcap/dist")
    ins = sub.add_parser("install", help="install a .urcap into a running e-Series URSim container")
    ins.add_argument("file")
    ins.add_argument("--container", required=True)
    args = ap.parse_args(argv)
    try:
        if args.cmd == "sdk":
            images = args.image
            if not images:
                plan = compat_plan(
                    read_properties((Path(args.src) / "bundle.properties").read_text(encoding="utf-8"))
                )
                images = [plan["floor"], *plan["since"]]
            if args.dir and len(images) != 1:
                raise Urcap5Error("--dir takes exactly one --image")
            for image in images:
                info = fetch_sdk(image, Path(args.dir) if args.dir else None, source=args.source)
                print(json.dumps({k: info[k] for k in ("image", "api_version", "jars", "dir")}))
        elif args.cmd == "package":
            print(package(args.src, args.out))
        elif args.cmd == "magic":
            print(write_magic(args.src, args.out))
        elif args.cmd == "check":
            src = Path(args.src)
            props = read_properties((src / "bundle.properties").read_text(encoding="utf-8"))
            dist = Path(args.dist) if args.dist else REPO_ROOT / "urcap" / "dist" / dist_name(props)
            print(json.dumps(check(src, Path(args.sdk), dist, args.version)))
        elif args.cmd == "compare":
            problems = compare_jars(args.built, args.committed)
            for p in problems:
                print(p, file=sys.stderr)
            return 1 if problems else 0
        elif args.cmd == "release-check":
            print(json.dumps(release_check(args.tag, args.src, args.dist)))
        else:
            print(install(args.file, args.container))
    except Urcap5Error as exc:
        print(f"urcap5: {exc}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
