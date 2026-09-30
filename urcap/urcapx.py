#!/usr/bin/env python3
"""Package, install, list and delete a PolyScope X URCap (``.urcapx``) — stdlib only.

What UR's ``@universal-robots/urcap-utils`` 2.1.2 does with node, done here without
an npm toolchain (the URCap under ``urcap/`` is plain JavaScript, so nothing needs
building):

* ``package SRC [--out DIR]`` — ``SRC`` holds ``manifest.yaml`` and the web-archive
  folder(s) it names. Writes ``DIR/<urcapID>-<version>.urcapx``: a **gzipped tar**
  with ``manifest.yaml`` as the first member (``package-urcap.js`` + ``tar-helper.js``
  in urcap-utils), a ``LICENSE`` (``SRC/LICENSE`` or the repo's) and the folders.
* ``install FILE [--host H] [--port P] [--replace]`` — the endpoint PolyScope X's
  own System Manager uses, ``/universal-robots/urservice/api/v1/urcaps``: a
  multipart ``POST`` of field ``urcapxFile`` (verified on the 10.13.0 sim,
  2026-09-26: 201 on install, 409 ``already_installed`` on a duplicate, 200 on
  ``DELETE …/<vendor>/<urcap>``). The SDK's CLI posts to the Robot-API instead
  (``/universal-robots/robot-api/urcaps/v1/urcaps/``, field ``urcapx_file``),
  which answers **403 unless the robot is in Remote mode** — from inside the
  container too. ``--replace`` deletes an installed copy first (the only way to
  update: the endpoint has no PUT).
* ``list`` / ``delete VENDOR URCAP`` — the same endpoint. Refresh the PolyScope
  page afterwards; the simulator in this repo listens on ``localhost:8000``.

    python3 urcap/urcapx.py package urcap/perceptronic --out urcap/dist
    python3 urcap/urcapx.py install urcap/dist/perceptronic-0.2.0.urcapx --port 8000 --replace
"""

from __future__ import annotations

import argparse
import gzip
import hashlib
import io
import json
import re
import shutil
import sys
import tarfile
import tempfile
import urllib.error
import urllib.request
import uuid
from pathlib import Path

MANIFEST = "manifest.yaml"
API_PATH = "/universal-robots/urservice/api/v1/urcaps"
FILE_FIELD = "urcapxFile"
REPO_ROOT = Path(__file__).resolve().parent.parent


class UrcapError(RuntimeError):
    pass


# -- manifest (the two-level YAML we write; not a YAML parser) -------------------------------


def read_manifest(text: str) -> dict:
    """``{vendorID, urcapID, urcapName, vendorName, version, folders: [...]}`` from
    a ``manifest.yaml`` of the shape the SDK generator writes."""

    def scalar(key: str) -> str | None:
        m = re.search(rf"^\s*{key}:\s*\"?([^\"\n]+?)\"?\s*$", text, re.M)
        return m.group(1).strip() if m else None

    out = {k: scalar(k) for k in ("vendorID", "urcapID", "urcapName", "vendorName", "version")}
    missing = [k for k in ("vendorID", "urcapID", "version") if not out[k]]
    if missing:
        raise UrcapError(f"{MANIFEST} is missing {', '.join(missing)}")
    if not re.fullmatch(r"[a-z][a-z0-9_-]*[a-z0-9]", out["urcapID"]) or not re.fullmatch(
        r"[a-z][a-z0-9_-]*[a-z0-9]", out["vendorID"]
    ):
        raise UrcapError("vendorID/urcapID must match ^[a-z][a-z0-9_-]*[a-z0-9]$ (manifest-spec-19.10.31)")
    out["version"] = re.sub(r"^v", "", out["version"])
    out["folders"] = [m.strip() for m in re.findall(r"^\s*folder:\s*\"?([^\"\n]+?)\"?\s*$", text, re.M)]
    return out


# -- package ------------------------------------------------------------------------------------


def package(src: str | Path, out_dir: str | Path) -> Path:
    src = Path(src)
    manifest_path = src / MANIFEST
    if not manifest_path.is_file():
        raise UrcapError(f"no {MANIFEST} in {src}")
    meta = read_manifest(manifest_path.read_text(encoding="utf-8"))
    for folder in meta["folders"]:
        if not (src / folder).is_dir():
            raise UrcapError(f"web archive folder {folder!r} named in {MANIFEST} is not in {src}")
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    out = out_dir / f"{meta['urcapID']}-{meta['version']}.urcapx"
    with tempfile.TemporaryDirectory() as tmp:
        stage = Path(tmp) / "dist"
        stage.mkdir()
        shutil.copyfile(manifest_path, stage / MANIFEST)
        licence = src / "LICENSE" if (src / "LICENSE").is_file() else REPO_ROOT / "LICENSE"
        if licence.is_file():
            shutil.copyfile(licence, stage / "LICENSE")
        for folder in meta["folders"]:
            shutil.copytree(src / folder, stage / folder)
        members = sorted(p.name for p in stage.iterdir() if p.name != MANIFEST)
        # Reproducible: the same source gives the same bytes (owners, modes and one
        # mtime fixed; tarfile.add recurses in sorted order), so the committed
        # urcap/dist/ package can be checked against a fresh build. The mtime is
        # derived from the contents rather than 0 so an updated package never
        # shares Last-Modified/ETag with the old one should an installer keep tar
        # mtimes (PolyScope's nginx serves the extracted files). The 10.13 sim's
        # installer doesn't — it stamps the install time (2026-09-27) — so this is
        # belt and braces.
        mtime = _content_mtime(stage)

        def normalise(info: tarfile.TarInfo) -> tarfile.TarInfo:
            info.mtime = mtime
            info.uid = info.gid = 0
            info.uname = info.gname = ""
            info.mode = 0o755 if info.isdir() else 0o644
            return info

        with open(out, "wb") as raw, gzip.GzipFile(filename="", mode="wb", fileobj=raw, mtime=0) as gz:
            with tarfile.open(fileobj=gz, mode="w", format=tarfile.USTAR_FORMAT) as tar:
                for name in [MANIFEST, *members]:
                    tar.add(stage / name, arcname=name, filter=normalise)
    return out


def _content_mtime(stage: Path) -> int:
    """A timestamp in 2023–2026 that changes whenever any staged file does."""
    h = hashlib.sha256()
    for p in sorted(stage.rglob("*")):
        if p.is_file():
            h.update(p.relative_to(stage).as_posix().encode() + b"\0" + p.read_bytes() + b"\0")
    return 1_672_531_200 + int.from_bytes(h.digest()[:8], "big") % 100_000_000


# -- the urservice URCap endpoint -----------------------------------------------------------------


def _base(host: str, port: int) -> str:
    return f"http://{host}:{port}{API_PATH}"


def _request(
    url: str, *, method: str = "GET", data: bytes | None = None, headers: dict | None = None
) -> dict:
    req = urllib.request.Request(url, data=data, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=180) as r:
            body = r.read()
            status = r.status
    except urllib.error.HTTPError as exc:
        body = exc.read()
        status = exc.code
    except (urllib.error.URLError, OSError) as exc:
        raise UrcapError(f"{method} {url}: {exc}") from None
    try:
        payload = json.loads(body or b"{}")
    except json.JSONDecodeError:
        payload = {"raw": body.decode("utf-8", "replace")[:500]}
    return {"status": status, "payload": payload}


def list_urcaps(host: str, port: int) -> list[dict]:
    """The installed URCaps: ``[{id: {vendorID, urcapID}, version, urcapName, …}]``."""
    res = _request(_base(host, port))
    if res["status"] != 200:
        raise UrcapError(f"list failed: HTTP {res['status']} {res['payload']}")
    items = res["payload"]
    if isinstance(items, dict) and "message" in items:  # the Robot-API wraps the list in a string
        try:
            items = json.loads(items["message"])
        except (TypeError, json.JSONDecodeError):
            items = []
    return items if isinstance(items, list) else []


def is_installed(host: str, port: int, vendor: str, urcap: str) -> bool:
    return any(
        (it.get("id") or {}).get("vendorID") == vendor and (it.get("id") or {}).get("urcapID") == urcap
        for it in list_urcaps(host, port)
    )


def _multipart(field: str, filename: str, content: bytes) -> tuple[bytes, str]:
    boundary = f"----urcapx-{uuid.uuid4().hex}"
    buf = io.BytesIO()
    buf.write(f"--{boundary}\r\n".encode())
    buf.write(f'Content-Disposition: form-data; name="{field}"; filename="{filename}"\r\n'.encode())
    buf.write(b"Content-Type: application/octet-stream\r\n\r\n")
    buf.write(content)
    buf.write(f"\r\n--{boundary}--\r\n".encode())
    return buf.getvalue(), f"multipart/form-data; boundary={boundary}"


def manifest_from_urcapx(path: str | Path) -> dict:
    with tarfile.open(path, "r:gz") as tar:
        member = tar.extractfile(MANIFEST)
        if member is None:
            raise UrcapError(f"{path} has no {MANIFEST}")
        return read_manifest(member.read().decode("utf-8"))


def _errors(payload) -> str:
    if isinstance(payload, dict):
        errs = payload.get("errors")
        if isinstance(errs, list) and errs:
            return "; ".join(f"{e.get('code')}: {e.get('message')}" for e in errs if isinstance(e, dict))
        return str(payload.get("message") or payload.get("details") or "")
    return str(payload)


def install(path: str | Path, host: str, port: int, *, replace: bool = False) -> dict:
    """POST ``path``; with ``replace`` an installed copy is deleted first."""
    path = Path(path)
    meta = manifest_from_urcapx(path)
    vendor, urcap = meta["vendorID"], meta["urcapID"]
    replaced = False
    if replace and is_installed(host, port, vendor, urcap):
        d = delete(host, port, vendor, urcap)
        if d["status"] not in (200, 204):
            raise UrcapError(f"delete before reinstall failed: HTTP {d['status']} {_errors(d['payload'])}")
        replaced = True
    body, ctype = _multipart(FILE_FIELD, path.name, path.read_bytes())
    res = _request(
        _base(host, port),
        method="POST",
        data=body,
        headers={"Content-Type": ctype, "Content-Length": str(len(body))},
    )
    res.update({"vendorID": vendor, "urcapID": urcap, "version": meta["version"], "replaced": replaced})
    res["ok"] = res["status"] in (200, 201)
    if res["status"] == 409:
        res["hint"] = f"already installed — pass --replace to delete {vendor}/{urcap} and install this one"
    elif res["status"] == 403:
        res["hint"] = (
            "403: this controller gates URCap installs on Remote / External Control mode; "
            "switch it in the UI, or install through System Manager → URCaps → + URCap"
        )
    elif not res["ok"]:
        res["hint"] = _errors(res["payload"])
    return res


def delete(host: str, port: int, vendor: str, urcap: str) -> dict:
    return _request(f"{_base(host, port)}/{vendor}/{urcap}", method="DELETE")


# -- release --------------------------------------------------------------------------------------

TAG_PREFIX = "urcapx-v"
_SEMVER = re.compile(r"\d{1,4}\.\d{1,4}\.\d{1,4}")


def release_check(tag: str, src: str | Path, dist_dir: str | Path) -> dict:
    """What a ``urcapx-v<version>`` tag may publish: ``{version, path, sha256}``, or
    UrcapError naming what is wrong. The committed ``dist/<urcapID>-<version>.urcapx`` must
    be the tag's version and exactly what the tagged sources package to (the build is
    reproducible: same tar bytes inside the gzip)."""
    src, dist_dir = Path(src), Path(dist_dir)
    if not tag.startswith(TAG_PREFIX) or not _SEMVER.fullmatch(tag[len(TAG_PREFIX) :]):
        raise UrcapError(f"tag {tag!r} is not {TAG_PREFIX}<major>.<minor>.<patch>")
    version = tag[len(TAG_PREFIX) :]
    meta = read_manifest((src / MANIFEST).read_text(encoding="utf-8"))
    if meta["version"] != version:
        raise UrcapError(f"tag {tag} but {src / MANIFEST} says version {meta['version']}")
    path = dist_dir / f"{meta['urcapID']}-{version}.urcapx"
    if not path.is_file():
        raise UrcapError(f"{path} is not committed — run `make urcap-package` and commit it")
    with tempfile.TemporaryDirectory() as tmp:
        fresh = package(src, tmp)
        if gzip.decompress(fresh.read_bytes()) != gzip.decompress(path.read_bytes()):
            raise UrcapError(f"{path} is not what the sources package to — run `make urcap-package`")
    packed = manifest_from_urcapx(path)
    if packed["version"] != version or packed["urcapID"] != meta["urcapID"]:
        raise UrcapError(
            f"{path} carries {packed['urcapID']} {packed['version']}, not {meta['urcapID']} {version}"
        )
    return {"version": version, "path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}


# -- CLI ------------------------------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        prog="urcapx", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    pk = sub.add_parser("package", help="build SRC into a .urcapx")
    pk.add_argument("src")
    pk.add_argument("--out", default="target", help="output directory (default target/)")
    rc = sub.add_parser("release-check", help="check a urcapx-v<version> tag against the committed package")
    rc.add_argument("tag")
    rc.add_argument("--src", default=str(REPO_ROOT / "urcap" / "perceptronic"))
    rc.add_argument("--dist", default=str(REPO_ROOT / "urcap" / "dist"))
    for name in ("install", "list", "delete"):
        p = sub.add_parser(name)
        p.add_argument("--host", default="localhost")
        p.add_argument("--port", type=int, default=8000, help="PolyScope X web port (the sim publishes 8000)")
        if name == "install":
            p.add_argument("file")
            p.add_argument("--replace", action="store_true", help="delete an installed copy first")
        if name == "delete":
            p.add_argument("vendor")
            p.add_argument("urcap")
    args = ap.parse_args(argv)
    try:
        if args.cmd == "package":
            out = package(args.src, args.out)
            print(out)
            return 0
        if args.cmd == "release-check":
            print(json.dumps(release_check(args.tag, args.src, args.dist)))
            return 0
        if args.cmd == "list":
            for it in list_urcaps(args.host, args.port):
                ident = it.get("id") or {}
                ident_s = f"{ident.get('vendorID')}/{ident.get('urcapID')}"
                print(f"{ident_s}  {it.get('version')}  {it.get('urcapName')}")
            return 0
        if args.cmd == "install":
            res = install(args.file, args.host, args.port, replace=args.replace)
            print(json.dumps({k: v for k, v in res.items() if k != "payload"}, indent=1))
            if res["ok"]:
                print("installed — refresh the PolyScope X page", file=sys.stderr)
            return 0 if res["ok"] else 1
        if args.cmd == "delete":
            res = delete(args.host, args.port, args.vendor, args.urcap)
            print(json.dumps(res, indent=1))
            return 0 if res["status"] in (200, 204) else 1
    except UrcapError as exc:
        print(f"urcapx: {exc}", file=sys.stderr)
        return 1
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
