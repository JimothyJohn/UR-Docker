"""The PolyScope X URCap (urcap/perceptronic): its source tree agrees with itself,
urcap/urcapx.py packages it reproducibly (and the downloadable urcap/dist/ copy is
that build, byte for byte) the way UR's urcap-utils does and installs it the way
the Robot-API expects (against a real HTTP server), the behavior worker speaks the
threads.js protocol (run under node when present), and the cockpit's CORS + colour
endpoint that the URCap page depends on."""

from __future__ import annotations

import email.parser
import gzip
import json
import re
import shutil
import subprocess
import sys
import tarfile
import threading
import urllib.error
import urllib.request
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

from perceptronics.config import PerceptionConfig
from perceptronics.realsense import SyntheticRgbdCamera
from perceptronics.webapp import ViewerApp, ViewerHandler

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "urcap"))
import urcapx  # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
URCAP = ROOT / "urcap" / "perceptronic"
DIST = ROOT / "urcap" / "dist"
FRONTEND = URCAP / "perceptronic-frontend"
PACKAGE_VERSION = urcapx.read_manifest((URCAP / "manifest.yaml").read_text(encoding="utf-8"))["version"]
NODE = shutil.which("node")


# -- the source tree ----------------------------------------------------------------------------


def test_manifest_contribution_and_sources_agree():
    meta = urcapx.read_manifest((URCAP / "manifest.yaml").read_text(encoding="utf-8"))
    assert meta["vendorID"] == "advin" and meta["urcapID"] == "perceptronic"
    assert meta["folders"] == ["perceptronic-frontend"]
    contribution = json.loads((FRONTEND / "contribution.json").read_text(encoding="utf-8"))
    (node,) = contribution["applicationNodes"]
    tag = node["componentTagName"]
    assert (FRONTEND / node["presenterURI"]).is_file() and (FRONTEND / node["behaviorURI"]).is_file()
    main_js = (FRONTEND / node["presenterURI"]).read_text(encoding="utf-8")
    worker_js = (FRONTEND / node["behaviorURI"]).read_text(encoding="utf-8")
    assert f'const TAG = "{tag}"' in main_js and f'const NODE_TYPE = "{tag}"' in worker_js
    i18n = json.loads((FRONTEND / node["translationPath"] / "en.json").read_text(encoding="utf-8"))
    assert i18n["application"]["nodes"][tag]["title"] == "Perceptronic"
    # one program node: 3D Pick (0.5.0 — "After picture N" went with the node's children)
    (pick,) = contribution["programNodes"]
    assert pick["componentTagName"] == f"{tag}-pick"
    lib = (FRONTEND / "pickscript.js").read_text(encoding="utf-8")
    for key in ("presenterURI", "behaviorURI", "iconURI"):
        assert (FRONTEND / pick[key]).is_file(), pick[key]
    assert pick["categoryName"] == "perceptronic" and pick["translationPath"] == "assets/i18n/"
    worker = (FRONTEND / pick["behaviorURI"]).read_text(encoding="utf-8")
    assert 'importScripts("pickscript.js")' in worker
    assert i18n["program"]["tree"]["nodes"] == {pick["componentTagName"]: "3D Pick"}
    pick_js = (FRONTEND / pick["presenterURI"]).read_text(encoding="utf-8")
    assert f'"{pick["componentTagName"]}"' in pick_js and f'"{pick["componentTagName"]}"' in lib
    assert f"{tag}-after" not in pick_js + lib + worker
    assert f'const APP_TYPE = "{tag}"' in lib and f'const APP_TAG = "{tag}"' in pick_js
    for key in ("smartSkills", "sidebarItems", "operatorScreens"):
        assert contribution[key] == []


def test_manifest_reader_rejects_bad_ids_and_missing_fields():
    with pytest.raises(urcapx.UrcapError, match="missing"):
        urcapx.read_manifest("metadata:\n  urcapID: x\n")
    with pytest.raises(urcapx.UrcapError, match="must match"):
        urcapx.read_manifest('metadata:\n  vendorID: "Bad ID"\n  urcapID: "ok"\n  version: 1.0.0\n')
    meta = urcapx.read_manifest('metadata:\n  vendorID: "vend"\n  urcapID: "urc"\n  version: v2.3.4\n')
    assert meta["version"] == "2.3.4" and meta["folders"] == []


# -- packaging ------------------------------------------------------------------------------------


def test_package_is_a_gzipped_tar_with_the_manifest_first(tmp_path):
    out = urcapx.package(URCAP, tmp_path)
    assert out.name == "perceptronic-0.6.0.urcapx"
    with tarfile.open(out, "r:gz") as tar:
        names = tar.getnames()
    assert names[0] == "manifest.yaml"
    assert "LICENSE" in names
    for rel in (
        "main.js",
        "perceptronic-node.worker.js",
        "pick.js",
        "pickscript.js",
        "pick-node.worker.js",
        "contribution.json",
        "assets/i18n/en.json",
        "assets/icons/perceptronic-pick.svg",
    ):
        assert f"perceptronic-frontend/{rel}" in names
    assert not any(n.startswith("./") or n.startswith("/") for n in names)
    assert urcapx.manifest_from_urcapx(out)["urcapID"] == "perceptronic"


def test_package_refuses_a_missing_folder(tmp_path):
    src = tmp_path / "bad"
    src.mkdir()
    (src / "manifest.yaml").write_text(
        'metadata:\n  vendorID: "vend"\n  urcapID: "urc"\n  version: 1.0.0\nartifacts:\n  webArchives:\n'
        '  - id: "f"\n    folder: "f"\n',
        encoding="utf-8",
    )
    with pytest.raises(urcapx.UrcapError, match="not in"):
        urcapx.package(src, tmp_path / "out")


def test_package_is_reproducible_and_normalised(tmp_path):
    a = urcapx.package(URCAP, tmp_path / "a").read_bytes()
    b = urcapx.package(URCAP, tmp_path / "b").read_bytes()
    assert a == b
    with tarfile.open(tmp_path / "a" / "perceptronic-0.6.0.urcapx", "r:gz") as tar:
        infos = tar.getmembers()
    assert {(i.uid, i.gid, i.uname, i.gname) for i in infos} == {(0, 0, "", "")}
    assert len({i.mtime for i in infos}) == 1 and infos[0].mtime > 1_600_000_000
    assert {i.mode for i in infos if i.isfile()} == {0o644}


def test_changed_source_changes_the_timestamp(tmp_path):
    """One fixed mtime per build would give an updated main.js the old
    Last-Modified/ETag on an installer that keeps tar mtimes (the 10.13 sim's
    stamps the install time instead), and the browser would keep the stale copy."""
    src = tmp_path / "src"
    shutil.copytree(URCAP, src)
    before = urcapx.package(src, tmp_path / "a")
    with tarfile.open(before, "r:gz") as tar:
        old = tar.getmember("manifest.yaml").mtime
    main = src / "perceptronic-frontend" / "main.js"
    main.write_text(main.read_text(encoding="utf-8") + "\n// edit\n", encoding="utf-8")
    after = urcapx.package(src, tmp_path / "b")
    with tarfile.open(after, "r:gz") as tar:
        assert tar.getmember("manifest.yaml").mtime != old


def test_the_downloadable_package_is_the_current_source(tmp_path):
    """urcap/dist/ is what people download: it must be exactly what the source
    builds to now. Edited the URCap? `make urcap-package` and commit urcap/dist/."""
    fresh = urcapx.package(URCAP, tmp_path)
    shipped = DIST / fresh.name
    assert shipped.is_file(), f"{shipped.relative_to(ROOT)} missing — run `make urcap-package` and commit it"
    # Compare the tar inside, not the gzip bytes: the same tar deflates to different
    # bytes under a different zlib build (windows-latest CI differed from byte 12 —
    # inside the deflate stream — while macOS/Linux matched, 2026-09-27).
    assert gzip.decompress(shipped.read_bytes()) == gzip.decompress(fresh.read_bytes()), (
        f"{shipped.relative_to(ROOT)} is stale — run `make urcap-package` and commit it"
    )
    stale = sorted(p.name for p in DIST.glob("*.urcapx") if p.name != fresh.name)
    assert not stale, f"old packages left in urcap/dist/: {stale} (the README links one version)"
    readme = (ROOT / "urcap" / "README.md").read_text(encoding="utf-8")
    assert f"dist/{fresh.name}" in readme, f"urcap/README.md doesn't link dist/{fresh.name}"
    linked = set(re.findall(r"perceptronic-\d+\.\d+\.\d+\.urcapx", readme))
    assert linked == {fresh.name}, f"urcap/README.md names other versions: {sorted(linked - {fresh.name})}"


# -- installing against a urservice look-alike ----------------------------------------------------


class UrserviceStub(BaseHTTPRequestHandler):
    """What the 10.13.0 sim's /universal-robots/urservice/api/v1/urcaps did on
    2026-09-26: a plain JSON array on GET, 201 on a multipart POST of urcapxFile,
    409 already_installed on a duplicate, 200 on DELETE /<vendor>/<urcap>."""

    installed: list[dict] = []
    requests: list[dict] = []
    status_override: int | None = None

    def log_message(self, *a):
        pass

    def _reply(self, obj, status=200):
        body = json.dumps(obj).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):  # noqa: N802
        UrserviceStub.requests.append({"method": "GET", "path": self.path})
        self._reply(UrserviceStub.installed)

    def do_POST(self):  # noqa: N802
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        msg = email.parser.BytesParser().parsebytes(
            f"Content-Type: {self.headers['Content-Type']}\r\n\r\n".encode() + raw
        )
        parts = [p for p in msg.walk() if p.get_filename()]
        fields = [(p.get_param("name", header="content-disposition"), p.get_filename()) for p in parts]
        UrserviceStub.requests.append(
            {
                "method": "POST",
                "path": self.path,
                "fields": fields,
                "size": sum(len(p.get_payload(decode=True)) for p in parts),
            }
        )
        if UrserviceStub.status_override:
            self._reply(
                {"errors": [{"code": "forced", "message": "forced"}]}, status=UrserviceStub.status_override
            )
            return
        if not fields or fields[0][0] != "urcapxFile":
            self._reply(
                {
                    "errors": [
                        {
                            "code": "internal_error",
                            "message": "did not find 'urcapxFile' in multipart form request",
                        }
                    ]
                },
                status=500,
            )
            return
        ident = {"vendorID": "advin", "urcapID": "perceptronic"}
        if any(it["id"] == ident for it in UrserviceStub.installed):
            self._reply(
                {"errors": [{"code": "already_installed", "message": "urcap already installed"}]}, status=409
            )
            return
        UrserviceStub.installed.append({"id": ident, "version": "0.1.0", "urcapName": "Perceptronic"})
        self._reply({"metadata": {"id": ident}}, status=201)

    def do_DELETE(self):  # noqa: N802
        UrserviceStub.requests.append({"method": "DELETE", "path": self.path})
        vendor, urcap = self.path.rstrip("/").split("/")[-2:]
        before = len(UrserviceStub.installed)
        UrserviceStub.installed = [
            it
            for it in UrserviceStub.installed
            if (it["id"]["vendorID"], it["id"]["urcapID"]) != (vendor, urcap)
        ]
        self._reply(
            {"statusCode": 200} if len(UrserviceStub.installed) < before else {"errors": []}, status=200
        )


@pytest.fixture
def urservice():
    UrserviceStub.installed = []
    UrserviceStub.requests = []
    UrserviceStub.status_override = None
    srv = ThreadingHTTPServer(("127.0.0.1", 0), UrserviceStub)
    srv.daemon_threads = True
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    yield "127.0.0.1", srv.server_address[1]
    srv.shutdown()
    srv.server_close()


def test_install_posts_urcapxfile_then_409_then_replace_deletes_first(tmp_path, urservice):
    host, port = urservice
    out = urcapx.package(URCAP, tmp_path)
    res = urcapx.install(out, host, port)
    assert res["ok"] and res["status"] == 201 and res["replaced"] is False
    upload = [r for r in UrserviceStub.requests if r["method"] == "POST"][-1]
    assert upload["path"] == urcapx.API_PATH
    assert upload["fields"] == [("urcapxFile", out.name)] and upload["size"] == out.stat().st_size
    assert urcapx.is_installed(host, port, "advin", "perceptronic")
    # a second install is a 409 with the --replace hint, and nothing changed
    res = urcapx.install(out, host, port)
    assert not res["ok"] and res["status"] == 409 and "--replace" in res["hint"]
    # --replace: GET, DELETE, POST
    res = urcapx.install(out, host, port, replace=True)
    assert res["ok"] and res["replaced"] is True
    assert [r["method"] for r in UrserviceStub.requests[-3:]] == ["GET", "DELETE", "POST"]
    assert UrserviceStub.requests[-2]["path"].endswith("/advin/perceptronic")
    assert len(UrserviceStub.installed) == 1


def test_install_names_the_remote_mode_gate_on_403(tmp_path, urservice):
    host, port = urservice
    UrserviceStub.status_override = 403
    out = urcapx.package(URCAP, tmp_path)
    res = urcapx.install(out, host, port)
    assert not res["ok"] and res["status"] == 403 and "Remote" in res["hint"]


def test_install_reports_an_unreachable_robot(tmp_path):
    out = urcapx.package(URCAP, tmp_path)
    with pytest.raises(urcapx.UrcapError, match="POST http|GET http"):
        urcapx.install(out, "127.0.0.1", 9)  # discard port: nothing listens


def test_list_accepts_the_robot_api_wrapping_too(monkeypatch):
    monkeypatch.setattr(
        urcapx,
        "_request",
        lambda url, **kw: {
            "status": 200,
            "payload": {"message": json.dumps([{"id": {"vendorID": "v", "urcapID": "u"}}])},
        },
    )
    assert urcapx.list_urcaps("h", 1) == [{"id": {"vendorID": "v", "urcapID": "u"}}]


def test_cli_list_and_package(tmp_path, urservice, capsys):
    host, port = urservice
    UrserviceStub.installed = [
        {
            "id": {"vendorID": "universal-robots", "urcapID": "web-frontend-app"},
            "version": "1.2.0",
            "urcapName": "Web",
        }
    ]
    assert urcapx.main(["list", "--host", host, "--port", str(port)]) == 0
    assert "universal-robots/web-frontend-app  1.2.0" in capsys.readouterr().out
    assert urcapx.main(["package", str(URCAP), "--out", str(tmp_path)]) == 0
    assert (tmp_path / "perceptronic-0.6.0.urcapx").is_file()
    assert (
        urcapx.main(
            ["install", str(tmp_path / "perceptronic-0.6.0.urcapx"), "--host", host, "--port", str(port)]
        )
        == 0
    )
    assert urcapx.main(["delete", "advin", "perceptronic", "--host", host, "--port", str(port)]) == 0
    assert UrserviceStub.installed == [UrserviceStub.installed[0]] and len(UrserviceStub.installed) == 1


# -- release: the urcapx-v<version> tag against the committed package ----------------------------


def test_release_check_accepts_the_committed_package():
    out = urcapx.release_check(f"urcapx-v{PACKAGE_VERSION}", URCAP, DIST)
    assert out["version"] == PACKAGE_VERSION and out["path"].endswith(
        f"perceptronic-{PACKAGE_VERSION}.urcapx"
    )
    assert re.fullmatch(r"[0-9a-f]{64}", out["sha256"])


@pytest.mark.parametrize(
    ("tag", "problem"),
    [
        ("v0.3.0", "is not urcapx-v"),
        ("urcapx-v9.9.9", "says version"),
        ("urcapx-v0.3", "is not urcapx-v"),
    ],
)
def test_release_check_refuses_a_wrong_tag(tag, problem):
    with pytest.raises(urcapx.UrcapError, match=problem):
        urcapx.release_check(tag, URCAP, DIST)


def test_release_check_refuses_a_stale_or_missing_package(tmp_path):
    src = tmp_path / "src"
    shutil.copytree(URCAP, src)
    dist = tmp_path / "dist"
    dist.mkdir()
    with pytest.raises(urcapx.UrcapError, match="not committed"):
        urcapx.release_check(f"urcapx-v{PACKAGE_VERSION}", src, dist)
    urcapx.package(src, dist)
    assert urcapx.release_check(f"urcapx-v{PACKAGE_VERSION}", src, dist)["version"] == PACKAGE_VERSION
    main = src / "perceptronic-frontend" / "main.js"
    main.write_text(main.read_text(encoding="utf-8") + "\n// edit\n", encoding="utf-8")
    with pytest.raises(urcapx.UrcapError, match="not what the sources package to"):
        urcapx.release_check(f"urcapx-v{PACKAGE_VERSION}", src, dist)
    assert (
        urcapx.main(["release-check", f"urcapx-v{PACKAGE_VERSION}", "--src", str(URCAP), "--dist", str(DIST)])
        == 0
    )


# -- the worker under node ----------------------------------------------------------------------

WORKER_HARNESS = r"""
const out = [];
const listeners = [];
globalThis.self = {
  postMessage: (m) => out.push(m),
  addEventListener: (type, fn) => { if (type === "message") listeners.push(fn); },
};
require(process.argv[2]);
const send = (m) => listeners.forEach((fn) => fn({ data: m }));
(async () => {
  send({ type: "run", uid: "u1", method: "factory", args: [] });
  const loaded = { type: "x", version: "0.0.1", cockpitUrl: "http://j:7621" };
  const fresh = { type: "x", version: "1.0.0", cockpitUrl: "" };
  send({ type: "run", uid: "u2", method: "upgradeNode", args: [loaded, fresh, {}] });
  send({ type: "run", uid: "u3", method: "nope", args: [] });
  send({ type: "not-a-run" });
  await new Promise((r) => setTimeout(r, 20));
  process.stdout.write(JSON.stringify(out));
})();
"""


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_worker_speaks_the_threads_protocol(tmp_path):
    harness = tmp_path / "harness.js"
    harness.write_text(WORKER_HARNESS, encoding="utf-8")
    proc = subprocess.run(
        [NODE, str(harness), str(FRONTEND / "perceptronic-node.worker.js")],
        capture_output=True,
        text=True,
        timeout=30,
        check=True,
    )
    msgs = json.loads(proc.stdout)
    assert msgs[0] == {
        "type": "init",
        "exposed": {"type": "module", "methods": ["factory", "upgradeNode", "downgradeNode"]},
    }
    by_uid = {}
    for m in msgs[1:]:
        by_uid.setdefault(m["uid"], []).append(m)
    assert [m["type"] for m in by_uid["u1"]] == ["running", "result"]
    assert by_uid["u1"][1]["complete"] is True
    fresh = {
        "type": "advin-perceptronic",
        "version": "1.1.0",
        "cockpitUrl": "",
        "areas": [],
        "tipMm": 163,
        "robotModel": "",
    }
    assert by_uid["u1"][1]["payload"] == fresh
    # a 1.0.0 node (cockpit URL only) comes up with the 1.1.0 defaults and its URL kept
    assert by_uid["u2"][1]["payload"] == {**fresh, "type": "x", "cockpitUrl": "http://j:7621"}
    assert by_uid["u3"][0]["type"] == "error" and by_uid["u3"][0]["error"]["__error_marker"] == "$$error"
    assert "nope" in by_uid["u3"][0]["error"]["message"]


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_presenter_parses_and_defines_the_element():
    subprocess.run([NODE, "--check", str(FRONTEND / "main.js")], check=True, timeout=30)
    src = (FRONTEND / "main.js").read_text(encoding="utf-8")
    assert re.search(r"customElements\.define\(TAG, Perceptronic\)", src)
    for prop in ("applicationNode", "applicationAPI", "robotSettings", "robotContext"):
        assert f"set {prop}(" in src, prop
    # the pick areas: touches through PolyScope's DH, the plane from pickscript.js
    for member in (
        "robotInfoService.getRobotType",
        "applicationNodeService.updateNode",
        "P.plane(",
        "P.fingertip(",
    ):
        assert member in src, member
    for route in (
        '"depth" : "color"}.png',
        "/api/point",
        "/api/segment",
        "/api/robot/locate",
        "/api/robot/move",
        "/api/robot/stop",
    ):
        assert route in src, route


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_program_node_presenter_parses_and_defines_its_elements():
    subprocess.run([NODE, "--check", str(FRONTEND / "pick.js")], check=True, timeout=30)
    src = (FRONTEND / "pick.js").read_text(encoding="utf-8")
    assert "customElements.define(PICK_TAG, PerceptronicPickNode)" in src
    assert "customElements.define(DIALOG_TAG, PerceptronicPickDialog)" in src
    assert "AFTER_TAG" not in src and "PerceptronicAfterNode" not in src
    for prop in ("contributedNode", "presenterAPI", "robotSettings", "programTree", "applicationContext"):
        assert src.count(f"set {prop}(") == 1, prop  # the row element takes every property PolyScope sets
    # the screen is a PolyScope custom dialog: it takes what PolyScope sets on one and is opened by the row
    for prop in ("inputData", "presenterApi", "afterOpen"):
        assert f"set {prop}(" in src, prop
    assert "dialogService.openCustomDialog(DIALOG_TAG" in src
    for route in ('"depth" : "color"}.png', "/api/pick/scene?opts="):  # the picture or the depth, one poll
        assert route in src, route
    # nothing in the dialog scrolls, and its options are two tabs
    assert "overflow: auto" not in src and "overflow-y" not in src and "overflow: scroll" not in src
    assert 'data-tab="part"' in src and 'data-tab="approach"' in src
    for gone in ("Gripper", "gripper-fields", "motion-fields", "Popup when nothing"):
        assert gone not in src, gone
    for member in (
        "programNodeService.updateNode",
        "applicationService.getApplicationNode(APP_TAG)",
        "variableService",
        'service("robotPositionService")',
        'service("robotMoveService")',
        "rms.autoMove(",
    ):
        assert member in src, member


# -- the cockpit side the page depends on ---------------------------------------------------------


@pytest.fixture
def cockpit():
    app = ViewerApp(
        SyntheticRgbdCamera(width=64, height=48, fps=0),
        config=PerceptionConfig(),
        cors=["http://localhost:8000"],
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    import time

    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield f"http://127.0.0.1:{srv.server_address[1]}", app
    srv.shutdown()
    srv.server_close()
    app.stop()


def _fetch(url, *, method="GET", origin=None, body=None):
    headers = {}
    if origin:
        headers["Origin"] = origin
    if body is not None:
        headers["Content-Type"] = "application/json"
    req = urllib.request.Request(url, method=method, headers=headers, data=body)
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


def test_color_png_is_a_png_with_a_sequence_header(cockpit):
    base, app = cockpit
    status, headers, body = _fetch(base + "/api/color.png")
    assert status == 200 and headers["Content-Type"] == "image/png" and body[:8] == b"\x89PNG\r\n\x1a\n"
    assert int(headers["X-Seq"]) >= 1 and "X-Fps" in headers
    status, _, body = _fetch(base + "/api/color.png?after=x")
    assert status == 400


def test_cors_headers_only_for_allowed_origins(cockpit):
    base, app = cockpit
    ok = _fetch(base + "/api/info", origin="http://localhost:8000")[1]
    assert ok["Access-Control-Allow-Origin"] == "http://localhost:8000" and ok["Vary"] == "Origin"
    assert "X-Seq" in ok["Access-Control-Expose-Headers"]
    other = _fetch(base + "/api/info", origin="http://evil.example")[1]
    assert "Access-Control-Allow-Origin" not in other
    none = _fetch(base + "/api/info")[1]
    assert "Access-Control-Allow-Origin" not in none
    # the colour feed and a POST carry them too, and the preflight answers 204
    png = _fetch(base + "/api/color.png", origin="http://localhost:8000")[1]
    assert png["Access-Control-Allow-Origin"] == "http://localhost:8000"
    status, headers, _ = _fetch(base + "/api/clear", method="OPTIONS", origin="http://localhost:8000")
    assert status == 204 and "POST" in headers["Access-Control-Allow-Methods"]
    status, headers, _ = _fetch(
        base + "/api/clear", method="POST", origin="http://localhost:8000", body=b"{}"
    )
    assert status == 200 and headers["Access-Control-Allow-Origin"] == "http://localhost:8000"


def test_cors_star_and_off():
    app = ViewerApp(SyntheticRgbdCamera(width=8, height=8, fps=0), config=PerceptionConfig(), cors=["*"])
    assert app.cors_origins == ["*"]
    app = ViewerApp(SyntheticRgbdCamera(width=8, height=8, fps=0), config=PerceptionConfig())
    assert app.cors_origins == []


def test_cors_from_args_reads_flag_then_env(monkeypatch):
    from perceptronics.webapp import cors_from_args

    class A:
        cors = None

    monkeypatch.delenv("PERCEPTRONICS_CORS", raising=False)
    assert cors_from_args(A()) == []
    monkeypatch.setenv("PERCEPTRONICS_CORS", "http://a:1, http://b:2")
    assert cors_from_args(A()) == ["http://a:1", "http://b:2"]
    A.cors = "*"
    assert cors_from_args(A()) == ["*"]


def test_refused_origin_is_named_once_and_sanitized(cockpit, capsys):
    base, app = cockpit
    for _ in range(3):
        headers = _fetch(base + "/api/color.png", origin="http://192.168.3.10:8000")[1]
        assert "Access-Control-Allow-Origin" not in headers
    err = capsys.readouterr().err
    assert err.count("refused a page from http://192.168.3.10:8000") == 1
    assert "--cors http://192.168.3.10:8000" in err
    info = json.loads(_fetch(base + "/api/info")[2])
    assert info["cors"] == {"allowed": ["http://localhost:8000"], "refused": ["http://192.168.3.10:8000"]}
    # an allowed origin, no Origin, the cockpit's own origin and the opaque "null" are not refusals
    _fetch(base + "/api/info", origin="http://localhost:8000")
    _fetch(base + "/api/clear", method="POST", origin=base, body=b"{}")
    _fetch(base + "/api/info", origin="null")
    assert app.cors_refused == {"http://192.168.3.10:8000"}
    # a hostile Origin can't put control characters or ANSI escapes in the log
    _fetch(base + "/api/info", origin="http://x\x1b[31m.example")
    err = capsys.readouterr().err
    assert "\x1b" not in err and "\\x1b[31m" in err


def test_refused_origins_are_capped(cockpit, capsys):
    base, app = cockpit
    for i in range(40):
        _fetch(base + "/api/info", origin=f"http://h{i}.example")
    assert len(app.cors_refused) == 32
    capsys.readouterr()


def test_private_network_preflight_is_answered_only_for_allowed_origins(cockpit):
    base, _ = cockpit
    pna = {"Access-Control-Request-Method": "POST", "Access-Control-Request-Private-Network": "true"}

    def preflight(origin):
        headers = {"Origin": origin, **pna}
        req = urllib.request.Request(base + "/api/segment", method="OPTIONS", headers=headers)
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, dict(r.headers)

    status, headers = preflight("http://localhost:8000")
    assert status == 204 and headers["Access-Control-Allow-Private-Network"] == "true"
    status, headers = preflight("http://evil.example")
    assert "Access-Control-Allow-Private-Network" not in headers
    assert "Access-Control-Allow-Origin" not in headers


COCKPIT_BASE_HARNESS = r"""
let defined = null;
globalThis.HTMLElement = class {};
globalThis.window = { customElements: { get: () => undefined, define: (tag, cls) => { defined = cls; } } };
require(process.argv[2]);
const loc = { protocol: "http:", hostname: "localhost", origin: "http://localhost:8001" };
const cases = JSON.parse(process.argv[3]);
process.stdout.write(JSON.stringify(cases.map((c) => defined.cockpitBase(c, loc))));
"""


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_cockpit_field_shorthand_is_never_fetched_relative_to_polyscope(tmp_path):
    # Regression (2026-09-27): a saved ":7621" was fetched as a path under PolyScope's
    # own page; PolyScope's 404 was then reported as "the cockpit predates the URCap routes".
    harness = tmp_path / "harness.js"
    harness.write_text(COCKPIT_BASE_HARNESS, encoding="utf-8")
    cases = {
        "": "http://localhost:7621",
        None: "http://localhost:7621",
        ":7621": "http://localhost:7621",
        "7621": "http://localhost:7621",
        " :7622/ ": "http://localhost:7622",
        "127.0.0.1:7621": "http://127.0.0.1:7621",
        "192.168.3.10": "http://192.168.3.10:7621",
        "jetson.local/": "http://jetson.local:7621",
        "http://127.0.0.1:7621//": "http://127.0.0.1:7621",
        "https://cam.example:8443": "https://cam.example:8443",
    }
    proc = subprocess.run(
        [NODE, str(harness), str(FRONTEND / "main.js"), json.dumps(list(cases))],
        capture_output=True,
        text=True,
        timeout=30,
        check=True,
    )
    got = json.loads(proc.stdout)
    assert dict(zip(cases, got, strict=True)) == cases
    assert all("://" in g for g in got)


POLYSCOPE_TARGET_HARNESS = r"""
let defined = null;
globalThis.HTMLElement = class {};
globalThis.window = { customElements: { get: () => undefined, define: (tag, cls) => { defined = cls; } } };
require(process.argv[2]);
const { flange, dh, t0s } = JSON.parse(process.argv[3]);
process.stdout.write(JSON.stringify(t0s.map((t0) => defined.polyScopeTarget(flange, dh, t0))));
"""

# PolyScope X 10.13 sim (UR3 config), robotPositionService.getKinematicInfo(), 2026-09-27.
SIM_DH = [
    {"DHTheta": 0, "DHa": 0, "DHd": 0.15185, "DHAlpha": 1.570796327},
    {"DHTheta": 0, "DHa": -0.24355, "DHd": 0, "DHAlpha": 0},
    {"DHTheta": 0, "DHa": -0.2132, "DHd": 0, "DHAlpha": 0},
    {"DHTheta": 0, "DHa": 0, "DHd": 0.13105, "DHAlpha": 1.570796327},
    {"DHTheta": 0, "DHa": 0, "DHd": 0.08535, "DHAlpha": -1.570796327},
    {"DHTheta": 0, "DHa": 0, "DHd": 0.0921, "DHAlpha": 0},
]
# ... and convertJointPositionsToTcpPose(all zeros) with the sim's (flange) TCP.
SIM_FLANGE_AT_ZERO = [-0.45675, -0.22315, 0.0665, 1.570796327, 0.0, 0.0]


@pytest.mark.skipif(NODE is None, reason="node is not installed")
def test_move_polyscope_targets_polyscopes_own_tcp(tmp_path):
    # Regression (2026-09-27): Move (PolyScope) handed PolyScope's IK the cockpit's
    # approach_pose — a TCP pose under the *cockpit controller's* 220 mm training offset —
    # so the sim (TCP = flange) was asked for a flange 17 cm past reach on every click.
    from urctl.pose import pose_trans

    flange = [-0.2316, 0.3103, -0.2155, -2.5834, 0.5663, 0.0012]
    offsets = [[0, 0, 0, 0, 0, 0], [0.0, -0.035, 0.22, 0.257, -0.41, 1.432], [0.01, 0.02, 0.15, 0, 0, 3.1]]
    t0s = [pose_trans(SIM_FLANGE_AT_ZERO, off) for off in offsets]
    harness = tmp_path / "harness.js"
    harness.write_text(POLYSCOPE_TARGET_HARNESS, encoding="utf-8")
    proc = subprocess.run(
        [
            NODE,
            str(harness),
            str(FRONTEND / "main.js"),
            json.dumps({"flange": flange, "dh": SIM_DH, "t0s": t0s}),
        ],
        capture_output=True,
        text=True,
        timeout=30,
        check=True,
    )
    got = json.loads(proc.stdout)
    assert got[0] == pytest.approx(flange, abs=1e-6)  # the sim: TCP is the flange
    for off, pose in zip(offsets[1:], got[1:], strict=True):
        want = pose_trans(flange, off)
        assert pose[:3] == pytest.approx(want[:3], abs=1e-6)
        # compare orientations as matrices (a rotation vector near pi has two spellings)
        from urctl.pose import rotvec_to_matrix

        a, b = rotvec_to_matrix(pose[3:]), rotvec_to_matrix(want[3:])
        assert [v for r in a for v in r] == pytest.approx([v for r in b for v in r], abs=1e-6)


def test_page_tells_a_cors_refusal_from_an_unreachable_cockpit():
    main_js = (FRONTEND / "main.js").read_text(encoding="utf-8")
    assert 'mode: "no-cors"' in main_js and "refuses this page" in main_js
    assert "nothing answers at" in main_js and "--bind 0.0.0.0" in main_js
    # an HTTP status is not re-diagnosed as a network failure
    assert "err.http" in main_js


def test_cors_never_echoes_the_request_origin_raw(cockpit):
    """CodeQL py/http-response-splitting: the Allow-Origin value is the configured entry,
    never the request's own bytes. An obs-folded Origin smuggles a CRLF into the header's
    value; it must not come back as a header of its own."""
    import socket
    from urllib.parse import urlparse

    base, _ = cockpit
    u = urlparse(base)
    raw = (
        b"GET /api/info HTTP/1.1\r\nHost: 127.0.0.1\r\nConnection: close\r\n"
        b"Origin: http://localhost:8000\r\n X-Injected: 1\r\n\r\n"
    )
    with socket.create_connection((u.hostname, u.port), timeout=5) as s:
        s.sendall(raw)
        resp = b""
        while chunk := s.recv(65536):
            resp += chunk
    head = resp.split(b"\r\n\r\n", 1)[0].lower()
    assert b"\r\nx-injected" not in head and b"access-control-allow-origin" not in head


def test_cors_drops_entries_that_are_not_an_origin(capsys):
    """Only ``*`` or scheme://host[:port] can ever match a browser's Origin; anything else
    (control characters, a path, whitespace) is dropped with a warning, not sent."""
    app = ViewerApp(
        SyntheticRgbdCamera(width=8, height=8, fps=0),
        config=PerceptionConfig(),
        cors=[
            "http://localhost:8000",
            "http://a\r\nX-Evil: 1",
            "http://b:80/path",
            "https://[::1]:8443",
            "ht tp://x",
        ],
    )
    assert app.cors_origins == ["http://localhost:8000", "https://[::1]:8443"]
    err = capsys.readouterr().err
    assert "\r" not in err and err.count("CORS: ignoring") == 3  # logged escaped, never raw
