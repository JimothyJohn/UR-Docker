"""urcap/track.py: finding UR's newest PolyScope X release and the SDK that pairs with
it, deciding when the pin is behind, and checking the URCap against that SDK.

The network half runs against a real local HTTP server that impersonates the four
sources (release notes, GitHub, Docker Hub, UR's npm feed) with synthetic
fixtures shaped like the real responses (captured 2026-09-27). The live sources
are exercised by `.github/workflows/urcap-track.yml`, not here."""

from __future__ import annotations

import datetime as dt
import io
import json
import random
import re
import sys
import tarfile
import threading
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "urcap"))
import track  # noqa: E402

FRONTEND = ROOT / "urcap" / "perceptronic" / "perceptronic-frontend"


# -- fixtures: a synthetic UR ---------------------------------------------------------------------


def notes_html(*releases: tuple[str, str, str | None], extra: str = "") -> str:
    """A release-notes article: ``(version, date, sdk)`` newest first, like UR's."""
    body = ["<html><head><script>var x = 'SW 9.9.9 Release Notes';</script></head><body>"]
    body.append("<h1>Release note Software version 10.x</h1>")
    for version, date, sdk in releases:
        body.append(f"<p>Date of release: {date}.</p>")
        body.append(f"<h2>SW {version} Release Notes</h2><p>Release Versions:</p><ul>")
        body.append(f"<li>PolyScope X robot image: {version}</li>")
        if sdk:
            body.append(f"<li>URCap X SDK: {sdk}</li>")
        body.append("</ul>")
    body.append(extra)
    body.append("</body></html>")
    return "\n".join(body)


def sdk_zip_bytes(version: str, contribution_api: str, threads: str, spec_version: str, spec: dict) -> bytes:
    root = f"sdk-polyscopex-{version}/urcap-generator/templates/frontend_and_backend/{{{{ urcapId }}}}"
    cond = "{% if includeFrontend and frontendType == 'javascript' %}"
    js = f"{root}/{cond}{{{{ urcapId }}}}-frontend{{% endif %}}"
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        # Jinja inside JSON, as in the real templates: not parseable as JSON
        z.writestr(
            f"{js}/package.json",
            '{\n  "name": "{{ frontendId }}-frontend",\n  {% if x %}"a": 1,{% endif %}\n'
            f'  "dependencies": {{ "threads": "{threads}" }},\n'
            f'  "devDependencies": {{ "@universal-robots/contribution-api": "{contribution_api}" }}\n}}\n',
        )
        z.writestr(
            f"{root}/package.json", '{ "devDependencies": { "@universal-robots/urcap-utils": "2.2.5" } }'
        )
        z.writestr(f"sdk-polyscopex-{version}/manifest-spec-{spec_version}.json", json.dumps(spec))
    return buf.getvalue()


DTS = """
/** a comment that mentions interface RobotMoveService { teleport(): void } */
declare class CommonBehaviorAPI {
    applicationService: ApplicationService;
    dialogService: DialogService;
    robotPositionService: RobotPositionService;
    robotInfoService: RobotInfoService;
    variableService: VariableService;
    symbolService: SymbolService;
}
declare class CommonPresenterAPI extends CommonBehaviorAPI {
    applicationNodeService: ApplicationNodeService;
}
declare class ApplicationPresenterAPI extends CommonPresenterAPI {
    robotMoveService: RobotMoveService;
    constructor(target: UREventTarget | CommunicationChannel);
}
declare class ProgramPresenterAPI extends CommonPresenterAPI {
    programNodeService: ProgramNodeService;
    robotMoveService: RobotMoveService;
    constructor(target: EventTargetOrCommChannel, selectedNodeId: string);
}
declare class ProgramNodeService {
    updateNode(node: ProgramNode): Promise<void>;
}
declare class ApplicationService {
    getApplicationNode(name: string): Promise<ApplicationNode>;
}
declare class VariableService {
    createVariable(name: string, variableType: string): Promise<VariableDeclaration>;
}
declare class RobotInfoService {
    getRobotType(): Promise<string>;
}
declare class SymbolService {
    generateVariable(name: string, valueType: VariableValueType): Promise<URVariable>;
}
declare class DialogService {
    openCustomDialog<P = any, R = P>(componentTag: string, initialData: P, options?: object): Promise<R>;
}
interface ProgramPresenter {
    robotSettings?: RobotSettings;
    contributedNode?: ProgramNode;
    programTree?: TreeContext;
    applicationContext?: ApplicationContext;
    presenterAPI?: ProgramPresenterAPI;
}
type ProgramBehaviors<ProgramNodeSubtype extends ProgramNode> = BaseBehavior<ProgramNodeSubtype> & {
    programNodeLabel: ProgramNodeLabel<ProgramNodeSubtype>;
    generateCodeBeforeChildren?: CodeGenerator<ProgramNodeSubtype>;
    generateCodeAfterChildren?: CodeGenerator<ProgramNodeSubtype>;
    validator?: Validator<ProgramNodeSubtype>;
    allowsChild?: ChildInsertionRule;
    upgradeNode?: ProgramVersionController<ProgramNodeSubtype>;
    onLifeCycleHook?: LifeCycleEvent<ProgramNodeLifeCycleEventType, SubtreeNode>;
};
declare class ApplicationNodeService {
    updateNode(node: ApplicationNode): Promise<void>;
}
declare class RobotPositionService {
    readonly eventTarget: EventTargetOrCommChannel;
    getKinematicInfo(): Promise<KinematicInfo>;
    getInverseKinematics(pose: Pose, qNear: JointPositions): Promise<JointPositions>;
    getJointPositions(): Observable<JointPositions>;
    convertJointPositionsToTcpPose(jointPositions: JointPositions): Promise<Pose>;
}
declare class RobotMoveService {
    autoMove(targetPosition: JointPositions | Waypoint): Promise<void>;
}
interface ApplicationPresenter {
    applicationNode: ApplicationNode;
    applicationAPI: ApplicationPresenterAPI;
    robotSettings?: RobotSettings;
}
type BaseBehavior<T extends ProgramNode | ApplicationNode = ApplicationNode> = {
    factory: BaseFactory<T>;
    upgradeNode?: (loaded: T, defaults: T) => OptionalPromise<T>;
    downgradeNode?: (loaded: T, defaults: T) => OptionalPromise<T>;
};
type ApplicationBehaviors = BaseBehavior<ApplicationNode> & {
    generatePreamble?: ApplicationCodeGenerator;
    nested: { notAMember: string };
};
"""

SPEC = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "type": "object",
    "additionalProperties": False,
    "required": ["metadata"],
    "properties": {"metadata": {"$ref": "#/$defs/metadata"}, "artifacts": {"$ref": "#/$defs/artifacts"}},
    "$defs": {
        "metadata": {
            "type": "object",
            "additionalProperties": False,
            "required": ["urcapID", "vendorID", "urcapName", "vendorName", "version"],
            "properties": {
                "vendorID": {"type": "string", "pattern": "^[a-z][a-z0-9._-]*[a-z0-9]$", "minLength": 2},
                "urcapID": {"type": "string", "pattern": "^[a-z][a-z0-9_-]*[a-z0-9]$", "maxLength": 50},
                "version": {"type": "string", "pattern": r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)$"},
                "vendorName": {"type": "string"},
                "urcapName": {"type": "string"},
            },
        },
        "artifacts": {
            "type": "object",
            "additionalProperties": False,
            "properties": {"webArchives": {"type": "array", "items": {"$ref": "#/$defs/webArchive"}}},
        },
        "webArchive": {
            "type": "object",
            "additionalProperties": False,
            "required": ["id", "folder"],
            "properties": {"id": {"type": "string"}, "folder": {"type": "string"}},
        },
    },
}

DIGEST = "sha256:" + "a" * 64


class FakeUR:
    """Routes → (status, body); a real server serves them."""

    def __init__(self) -> None:
        self.routes: dict[str, tuple[int, bytes]] = {}
        self.hits: list[str] = []

    def set(self, path: str, body, status: int = 200) -> None:
        if isinstance(body, (dict, list)):
            body = json.dumps(body)
        self.routes[path] = (status, body.encode() if isinstance(body, str) else body)

    def notes(self, minor: str, html: str) -> None:
        self.set(f"/notes/{track.notes_slug(minor)}/", html)

    def standard(self, minor: str = "10.14", release: str = "10.14.0", sdk: str = "6.6.66") -> None:
        self.notes(
            "10.13", notes_html(("10.13.1", "July 15th 2026", None), ("10.13.0", "June 2nd 2026", None))
        )
        self.notes(minor, notes_html((release, "August 27th 2026", "0.20.49")))
        self.set(
            f"/hub/{track.IMAGE_REPO}/tags?page_size=100&name={minor}.",
            {
                "next": None,
                "results": [
                    {"name": f"{minor}.0-0.10.777", "digest": DIGEST},
                    {"name": f"{minor}.0-0.10.703-preview-1", "digest": "sha256:" + "b" * 64},
                    {
                        "name": release,
                        "digest": DIGEST,
                        "images": [{"architecture": "arm64"}, {"architecture": "amd64"}],
                    },
                ],
            },
        )
        self.set(f"/hub/{track.IMAGE_REPO}/tags/0.19.135", {"name": "0.19.135", "digest": DIGEST})
        self.set(
            f"/gh/repos/{track.SDK_REPO}/releases?per_page=50",
            [
                {
                    "tag_name": "SDK-v0.22-rc",
                    "name": f"PolyScope X {minor} - SDK-v0.22",
                    "prerelease": True,
                    "published_at": "2026-09-01",
                },
                {
                    "tag_name": "SDK-v0.21",
                    "name": f"PolyScope X {minor} - SDK-v0.21",
                    "published_at": "2026-08-28",
                },
                {
                    "tag_name": "SDK-v0.20",
                    "name": "PolyScope X 10.13 - SDK-v0.20",
                    "published_at": "2026-06-02",
                },
                {"tag_name": "SDK-v0.1", "name": "PolyScope X 10.1 - SDK-v0.1", "published_at": "2024-01-01"},
            ],
        )
        self.set(
            f"/raw/{track.SDK_REPO}/SDK-v0.21/.devcontainer/Dockerfile",
            "FROM node:24\nENV PS_VERSION=0.21.64 \\\n    URSIM_VERSION=0.19.135 \\\n"
            f"    SDK_VERSION={sdk}\n",
        )
        self.set(
            f"/raw/{track.SDK_REPO}/SDK-v0.21/.devcontainer/devcontainer.json",
            '{"settings": {"yaml.schemas": {"manifest-spec-19.11.25.json": ["manifest.yaml"]}}}',
        )
        self.set(
            f"/raw/{track.SDK_REPO}/SDK-v0.21/.devcontainer/web-sdk-external-{sdk}.zip",
            sdk_zip_bytes(sdk, "22.14.184", "1.7.0", "19.11.25", SPEC),
        )
        buf = io.BytesIO()
        with tarfile.open(fileobj=buf, mode="w:gz") as tar:
            data = DTS.encode()
            info = tarfile.TarInfo("package/types/universal-robots-contribution-api.d.ts")
            info.size = len(data)
            tar.addfile(info, io.BytesIO(data))
        self.set("/npm/@universal-robots/contribution-api/-/contribution-api-22.14.184.tgz", buf.getvalue())


@pytest.fixture
def ur():
    fake = FakeUR()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):  # noqa: N802
            fake.hits.append(self.path)
            status, body = fake.routes.get(self.path, (404, b"not found"))
            self.send_response(status)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):
            pass

    server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{server.server_address[1]}"
    fake.src = track.Sources(
        notes=base + "/notes/{slug}/",
        github_api=base + "/gh",
        github_raw=base + "/raw",
        docker_hub=base + "/hub",
        npm_feed=base + "/npm",
    )
    yield fake
    server.shutdown()


# -- release notes --------------------------------------------------------------------------------


def test_notes_slug_is_urs_article_naming():
    assert track.notes_slug("10.14") == "1014x"
    assert track.notes_slug("10.7") == "107x"
    assert track.notes_slug("11.0") == "110x"


def test_parse_notes_takes_the_newest_patch_and_ignores_scripts():
    html = notes_html(("10.13.1", "July 15th 2026", None), ("10.13.0", "June 2nd 2026", "0.19.10"))
    notes = track.parse_notes(html)
    assert notes["release"] == "10.13.1" and notes["robot_image"] == "10.13.1"
    assert notes["released"] == "2026-07-15"
    # the SDK line belongs to 10.13.0's block, below the newest release — still found, it's the page's
    assert notes["notes_sdk"] == "0.19.10"


def test_parse_notes_surfaces_breaking_changes_and_urcap_sections():
    extra = (
        "<h2>URCap API</h2><p>Program type in context.</p><h3>allowedInProgramTypes</h3><p>MAIN or LOGIC.</p>"
        "<h2>Port 80 Handling</h2><p>Breaking change: port 80 is closed by default.</p><h2>Other</h2><p>x</p>"
    )
    notes = track.parse_notes(notes_html(("10.14.0", "August 27th 2026", None), extra=extra))
    assert notes["breaking"] == ["Breaking change: port 80 is closed by default."]
    (sec,) = notes["urcap_sections"]
    assert sec["heading"] == "URCap API"
    assert "Program type in context." in sec["text"] and "MAIN or LOGIC." in sec["text"]
    assert "port 80" not in sec["text"]  # stops at the next same-level heading


def test_urcap_sections_do_not_repeat_a_nested_match():
    html = "<h2>API</h2><p>intro</p><h3>Robot API</h3><p>popups</p><h2>URCap API</h2><p>types</p>"
    secs = track.urcap_sections(html)
    assert [s["heading"] for s in secs] == ["API", "URCap API"]
    assert "popups" in secs[0]["text"]


def test_parse_notes_rejects_a_page_without_a_release():
    with pytest.raises(track.TrackError, match="no 'SW x.y.z Release Notes'"):
        track.parse_notes("<html><h1>Page moved</h1></html>")


def test_parse_notes_tolerates_a_missing_or_odd_date():
    notes = track.parse_notes("<h2>SW 10.15.0 Release Notes</h2><p>Date of release: Smarch 40th 2026.</p>")
    assert notes["release"] == "10.15.0" and notes["released"] is None


def test_parse_notes_fuzz_never_crashes_with_anything_but_trackerror():
    rng = random.Random(1014)
    alphabet = [
        "<h2>", "</h2>", "<p>", "</p>", "SW ", "10.", "14", ".0", " Release Notes", "Date of release:",
        " August 27th 2026", "PolyScope X robot image:", "<script>", "</script>", "&amp;", "\x00", "é",
        "\ud7ff", "<li>", "Breaking change", "URCap", "<h1>", "API",
    ]  # fmt: skip
    for _ in range(400):
        html = "".join(rng.choice(alphabet) for _ in range(rng.randint(0, 40)))
        try:
            notes = track.parse_notes(html)
        except track.TrackError:
            continue
        assert re.fullmatch(r"\d+\.\d+\.\d+", notes["release"])


# -- discovery against the fake UR ----------------------------------------------------------------


def test_newest_minor_walks_forward_until_an_article_is_missing(ur):
    ur.standard()
    minor, html = track.newest_minor(ur.src, "10.13")
    assert minor == "10.14" and "SW 10.14.0" in html
    assert "/notes/1015x/" in ur.hits and "/notes/110x/" in ur.hits


def test_newest_minor_crosses_a_major(ur):
    ur.standard()
    ur.notes("11.0", notes_html(("11.0.0", "January 5th 2027", None)))
    assert track.newest_minor(ur.src, "10.13")[0] == "11.0"


def test_newest_minor_refuses_when_the_pinned_article_is_gone(ur):
    with pytest.raises(track.TrackError, match="pinned minor"):
        track.newest_minor(ur.src, "10.14")


def test_image_tags_keeps_releases_only_newest_first(ur):
    ur.set(
        f"/hub/{track.IMAGE_REPO}/tags?page_size=100&name=10.14.",
        {
            "next": ur.src.docker_hub + "/page2",
            "results": [
                {"name": "10.14.0", "digest": DIGEST},
                {"name": "10.14.0-0.10.777"},
                {"name": "10.140.1"},
            ],
        },
    )
    ur.set(
        "/hub/page2", {"next": None, "results": [{"name": "10.14.2", "digest": "d2"}, {"name": "10.14.10"}]}
    )
    assert [t["tag"] for t in track.image_tags(ur.src, "10.14")] == ["10.14.10", "10.14.2", "10.14.0"]


def test_sdk_release_matches_the_minor_exactly(ur):
    ur.standard()
    assert track.sdk_release(ur.src, "10.14")["tag_name"] == "SDK-v0.21"  # not the prerelease
    assert track.sdk_release(ur.src, "10.1")["tag_name"] == "SDK-v0.1"  # not 10.13 / 10.14
    assert track.sdk_release(ur.src, "10.15") is None


def test_parse_devcontainer_needs_all_three_versions():
    with pytest.raises(track.TrackError, match="SDK_VERSION"):
        track.parse_devcontainer("ENV PS_VERSION=1 URSIM_VERSION=2", "{}")


def test_template_versions_reads_jinja_templated_package_json():
    z = sdk_zip_bytes("6.6.66", "22.14.184", "1.7.0", "19.11.25", SPEC)
    assert track.template_versions(z) == {
        "contribution_api": "22.14.184",
        "urcap_utils": "2.2.5",
        "threads": "1.7.0",
    }


def test_resolve_builds_the_whole_target(ur):
    ur.standard()
    target, notes, pending = track.resolve(ur.src, "10.13")
    assert pending == []
    assert target["polyscope_x"] == {
        "minor": "10.14",
        "release": "10.14.0",
        "released": "2026-08-27",
        "notes": ur.src.notes.format(slug="1014x"),
        "notes_sdk": "0.20.49",
    }
    assert target["simulator"] == {
        "image": f"{track.IMAGE_REPO}:10.14.0",
        "digest": DIGEST,
        "architectures": ["amd64", "arm64"],
        "matches_sdk": True,
    }
    assert target["sdk"]["tag"] == "SDK-v0.21" and target["sdk"]["version"] == "6.6.66"
    assert target["sdk"]["manifest_spec"] == "19.11.25" and target["sdk"]["threads"] == "1.7.0"


def test_resolve_tests_a_patch_without_its_own_sim_on_the_minors_newest(ur):
    ur.standard(release="10.14.1")
    ur.set(
        f"/hub/{track.IMAGE_REPO}/tags?page_size=100&name=10.14.",
        {"next": None, "results": [{"name": "10.14.0", "digest": DIGEST}]},
    )
    target, _, pending = track.resolve(ur.src, "10.14")
    assert target["polyscope_x"]["release"] == "10.14.1"
    assert target["simulator"]["image"].endswith(":10.14.0")
    assert pending == ["no simulator image for 10.14.1; testing against 10.14.0"]


def _held_at_1013(ur) -> None:
    """A 10.13 world with a newer minor (10.14) published beside it."""
    ur.standard(minor="10.13", release="10.13.0", sdk="6.5.65")
    ur.notes("10.14", notes_html(("10.14.0", "August 27th 2026", "0.20.49")))


def test_resolve_holds_a_minor_instead_of_walking_forward(ur):
    _held_at_1013(ur)
    target, notes, _ = track.resolve(ur.src, "10.13", hold="10.13")
    assert notes["minor"] == "10.13" and target["polyscope_x"]["release"] == "10.13.0"
    assert target["simulator"]["image"].endswith(":10.13.0")
    assert not any("1014x" in h for h in ur.hits)  # never even looked at 10.14
    assert track.newest_minor(ur.src, "10.13")[0] == "10.14"  # without the hold it moves on


def test_cli_update_keeps_the_hold(ur, tmp_path, monkeypatch, capsys):
    """Held at 10.13 to match the local simulator image (Nick, 2026-09-27): the weekly
    track must neither move the pin to 10.14 nor drop the hold when it rewrites."""
    _held_at_1013(ur)
    target_file, doc = tmp_path / "target.json", tmp_path / "README.md"
    doc.write_text("<!-- urcap-target --><!-- /urcap-target -->\n", encoding="utf-8")
    target_file.write_text(
        json.dumps({**_target(polyscope_x__minor="10.13", polyscope_x__release="10.13.0"), "hold": "10.13"}),
        encoding="utf-8",
    )
    monkeypatch.setattr(track, "TARGET", target_file)
    monkeypatch.setattr(track, "DOCS", (doc,))
    monkeypatch.setattr(track, "Sources", lambda: ur.src)
    assert track.main(["update"]) == 0
    written = json.loads(target_file.read_text(encoding="utf-8"))
    assert written["hold"] == "10.13" and written["polyscope_x"]["release"] == "10.13.0"
    assert "held at 10.13" in doc.read_text(encoding="utf-8")
    assert "held at 10.13" in capsys.readouterr().err
    assert track.main(["check"]) == 0


def test_resolve_waits_for_the_sdk_and_the_image(ur):
    ur.standard()
    ur.set(f"/gh/repos/{track.SDK_REPO}/releases?per_page=50", [])
    ur.set(f"/hub/{track.IMAGE_REPO}/tags?page_size=100&name=10.14.", {"next": None, "results": []})
    target, notes, pending = track.resolve(ur.src, "10.13")
    assert target is None and notes["minor"] == "10.14"
    assert len(pending) == 2 and any("SDK" in p for p in pending)


def test_a_sim_that_is_not_the_sdks_is_flagged(ur):
    ur.standard()
    ur.set(f"/hub/{track.IMAGE_REPO}/tags/0.19.135", {"digest": "sha256:" + "c" * 64})
    target, _, _ = track.resolve(ur.src, "10.14")
    assert target["simulator"]["matches_sdk"] is False
    assert any("disagree" in p for p in track.compat(target, ur.src))


def test_overdue_after_the_grace_period():
    notes = {"released": "2026-08-27"}
    assert not track.overdue(notes, dt.date(2026, 9, 10))
    assert track.overdue(notes, dt.date(2026, 9, 27))
    assert not track.overdue({"released": None}, dt.date(2030, 1, 1))


# -- drift ----------------------------------------------------------------------------------------


def _target(**over) -> dict:
    t = {
        "polyscope_x": {"minor": "10.14", "release": "10.14.0", "notes": "u"},
        "simulator": {"image": "i:10.14.0", "digest": DIGEST},
        "sdk": {"tag": "SDK-v0.21", "version": "6.6.66", "contribution_api": "22", "urcap_utils": "2",
                "threads": "1.7.0", "manifest_spec": "19"},
    }  # fmt: skip
    for path, value in over.items():
        section, key = path.split("__")
        t[section] = {**t[section], key: value}
    return t


def test_drift_is_empty_when_current():
    assert track.drift(_target(), _target()) == []


@pytest.mark.parametrize(
    ("change", "expect"),
    [
        ({"polyscope_x__minor": "10.15"}, "new PolyScope X minor"),
        ({"polyscope_x__release": "10.14.1"}, "new PolyScope X release"),
        ({"simulator__image": "i:10.14.1"}, "simulator image"),
        ({"simulator__digest": "sha256:moved"}, "moved"),
        ({"sdk__contribution_api": "23"}, "SDK contribution_api"),
        ({"sdk__threads": "1.8.0"}, "SDK threads"),
    ],
)
def test_drift_names_each_kind_of_change(change, expect):
    reasons = track.drift(_target(), _target(**change))
    assert len(reasons) == 1 and expect in reasons[0]


def test_drift_from_nothing_lists_everything():
    assert len(track.drift({}, _target())) >= 3


# -- the docs' target lines -----------------------------------------------------------------------


def test_patch_docs_rewrites_only_the_marked_line_and_is_idempotent(tmp_path, ur):
    ur.standard()
    target, _, _ = track.resolve(ur.src, "10.14")
    doc = tmp_path / "README.md"
    doc.write_text(
        "# x\n\n<!-- urcap-target -->old 10.13<!-- /urcap-target -->\n\nrest 10.13\n", encoding="utf-8"
    )
    assert track.patch_docs(target, (doc,)) == [doc]
    text = doc.read_text(encoding="utf-8")
    assert "PolyScope X 10.14.0" in text and "old 10.13" not in text and "rest 10.13" in text
    assert track.patch_docs(target, (doc,)) == []


def test_the_real_docs_carry_the_marker_and_match_target_json():
    target = track.load_target()
    for doc in track.DOCS:
        (m,) = track.DOC_MARK.findall(doc.read_text(encoding="utf-8"))
        assert m[1] == track.doc_line(target), f"{doc.name}: run `python3 urcap/track.py update`"


# -- compat: API surface --------------------------------------------------------------------------


def test_members_follow_extends_intersections_and_generics():
    dts = track._strip_comments(DTS)
    assert {"robotMoveService", "robotPositionService", "applicationNodeService"} <= track.members(
        dts, "ApplicationPresenterAPI"
    )
    behaviors = track.members(dts, "ApplicationBehaviors")
    assert {"factory", "upgradeNode", "downgradeNode", "generatePreamble", "nested"} <= behaviors
    assert "notAMember" not in behaviors
    assert track.members(dts, "Nope") is None


def test_api_surface_passes_on_the_fixture_and_names_what_is_gone():
    assert track.check_api_surface(DTS) == []
    gone = DTS.replace("autoMove(", "autoMoveTo(").replace(
        "interface ApplicationPresenter {", "interface Other {"
    )
    problems = track.check_api_surface(gone)
    assert "contribution-api: RobotMoveService.autoMove is gone" in problems
    assert "contribution-api no longer declares ApplicationPresenter" in problems


def test_a_commented_out_member_does_not_count():
    dts = DTS.replace("    autoMove(targetPosition", "    // autoMove(targetPosition")
    assert "contribution-api: RobotMoveService.autoMove is gone" in track.check_api_surface(dts)


def test_api_surface_covers_every_call_the_urcap_makes():
    """The list is only a gate if it is complete: every service member main.js
    touches and every behavior the worker implements must be in it."""
    main_js = (FRONTEND / "main.js").read_text(encoding="utf-8")
    worker = (FRONTEND / "perceptronic-node.worker.js").read_text(encoding="utf-8")
    surface = track.API_SURFACE
    for m in re.finditer(r"\brps\.(\w+)", main_js):
        assert m.group(1) in surface["RobotPositionService"], m.group(0)
    for m in re.finditer(r"\b(?:api|this\._api)\.(\w+Service)\b(?:\.(\w+))?", main_js):
        service, member = m.groups()
        assert service in surface["ApplicationPresenterAPI"], m.group(0)
        cls = service[0].upper() + service[1:]
        if member and cls in surface:
            assert member in surface[cls], m.group(0)
    behaviors = re.search(r"const behaviors = \{(.*?)\n\};", worker, re.S).group(1)
    for name in re.findall(r"^\s{2}(\w+):", behaviors, re.M):
        assert name in surface["ApplicationBehaviors"], name
    for prop in ("applicationNode", "applicationAPI", "robotSettings"):
        assert f"set {prop}(" in main_js and prop in surface["ApplicationPresenter"]
    # the program nodes: pick.js against ProgramPresenterAPI, the two workers against ProgramBehaviors
    pick_js = (FRONTEND / "pick.js").read_text(encoding="utf-8")
    for m in re.finditer(r"\brps\.(\w+)", pick_js):
        assert m.group(1) in surface["RobotPositionService"], m.group(0)
    for m in re.finditer(r"\b(?:api|this\._api)\.(\w+Service)\b(?:\.(\w+))?", pick_js):
        service, member = m.groups()
        assert service in surface["ProgramPresenterAPI"], m.group(0)
        cls = service[0].upper() + service[1:]
        if member and cls in surface:
            assert member in surface[cls], m.group(0)
    for prop in surface["ProgramPresenter"]:
        assert f"set {prop}(" in pick_js, prop
    for name in ("pick-node.worker.js", "after-node.worker.js"):
        w = (FRONTEND / name).read_text(encoding="utf-8")
        behaviors = re.search(r"const behaviors = \{(.*?)\n\};", w, re.S).group(1)
        for member in re.findall(r"^\s{2}(\w+):", behaviors, re.M):
            assert member in surface["ProgramBehaviors"], f"{name}: {member}"


# -- compat: manifest -----------------------------------------------------------------------------


def test_read_yaml_reads_the_real_manifest():
    manifest = track.read_yaml((ROOT / "urcap/perceptronic/manifest.yaml").read_text(encoding="utf-8"))
    assert manifest["metadata"]["vendorID"] == "nickarmenta"
    assert manifest["metadata"]["version"] == "0.3.0"
    assert manifest["artifacts"]["webArchives"] == [
        {"id": "perceptronic-frontend", "folder": "perceptronic-frontend"}
    ]
    assert track.validate(manifest, SPEC) == []


def test_read_yaml_lists_scalars_and_comments():
    text = "# c\na:\n  - x\n  - 'y'\nb:\n- k: 1\n  j: \"2\"\n- k: 3\nc:\n"
    assert track.read_yaml(text) == {"a": ["x", "y"], "b": [{"k": "1", "j": "2"}, {"k": "3"}], "c": None}


@pytest.mark.parametrize("bad", ["just words", "a: 1\n  b: 2\n", "- top: list\n"])
def test_read_yaml_rejects_what_it_cannot_read(bad):
    with pytest.raises(track.TrackError):
        track.read_yaml(bad)


def test_read_yaml_fuzz_raises_only_trackerror():
    rng = random.Random(27)
    pieces = ["a", ":", " ", "  ", "- ", "\n", '"', "'", "#", "x: y", "\t", "é", "\x00", "::"]
    for _ in range(500):
        text = "".join(rng.choice(pieces) for _ in range(rng.randint(0, 30)))
        try:
            out = track.read_yaml(text)
        except track.TrackError:
            continue
        assert isinstance(out, dict)


@pytest.mark.parametrize(
    ("mutate", "expect"),
    [
        (lambda m: m["metadata"].pop("urcapName"), "missing required 'urcapName'"),
        (lambda m: m["metadata"].update(vendorID="Olympus Controls"), "does not match"),
        (lambda m: m["metadata"].update(version="v1"), "does not match"),
        (lambda m: m.update(extra={}), "unknown key 'extra'"),
        (lambda m: m["artifacts"]["webArchives"][0].pop("folder"), "missing required 'folder'"),
        (lambda m: m["artifacts"].update(webArchives="x"), "expected array"),
        (lambda m: m["metadata"].update(urcapID="u" * 51), "length 51"),
    ],
)
def test_validate_catches_what_a_new_spec_would_reject(mutate, expect):
    manifest = track.read_yaml((ROOT / "urcap/perceptronic/manifest.yaml").read_text(encoding="utf-8"))
    mutate(manifest)
    assert any(expect in e for e in track.validate(manifest, SPEC))


def test_validate_handles_enum_const_not_items_and_numbers():
    schema = {
        "type": "object",
        "properties": {
            "e": {"enum": ["a", "b"]},
            "c": {"const": 1},
            "n": {"not": {"type": "string"}},
            "l": {"type": "array", "items": {"type": "integer", "minimum": 0, "maximum": 5}},
        },
    }
    errs = track.validate({"e": "z", "c": 2, "n": "s", "l": [1, 9, "x"]}, schema)
    assert len(errs) == 5, errs
    assert track.validate({"e": "a", "c": 1, "n": 3, "l": [0, 5]}, schema) == []


def test_compat_passes_on_the_fixture_sdk(ur):
    ur.standard()
    target, _, _ = track.resolve(ur.src, "10.14")
    assert track.compat(target, ur.src) == []


def test_compat_flags_a_threads_bump(ur):
    ur.standard()
    ur.set(
        f"/raw/{track.SDK_REPO}/SDK-v0.21/.devcontainer/web-sdk-external-6.6.66.zip",
        sdk_zip_bytes("6.6.66", "22.14.184", "2.0.0", "19.11.25", SPEC),
    )
    target, _, _ = track.resolve(ur.src, "10.14")
    assert any("threads 2.0.0" in p for p in track.compat(target, ur.src))


# -- report + CLI ---------------------------------------------------------------------------------


def test_report_neutralises_scraped_text():
    notes = {
        "breaking": ["Breaking change: ping @dependabot <img src=x onerror=alert(1)>\n\n## heading"],
        "urcap_sections": [{"heading": "URCap API @team", "text": "<script>x</script>"}],
    }
    out = track.report(None, None, notes, [], [])
    assert "@dependabot" not in out and "@team" not in out and "@​dependabot" in out
    assert "<img" not in out and "<script>" not in out
    assert "\n## heading" not in out


def test_cli_update_then_check(ur, tmp_path, monkeypatch, capsys):
    ur.standard()
    target_file, doc, out = tmp_path / "target.json", tmp_path / "README.md", tmp_path / "gh.out"
    doc.write_text("<!-- urcap-target --><!-- /urcap-target -->\n", encoding="utf-8")
    target_file.write_text(
        json.dumps(_target(polyscope_x__minor="10.13", polyscope_x__release="10.13.0")), encoding="utf-8"
    )
    monkeypatch.setattr(track, "TARGET", target_file)
    monkeypatch.setattr(track, "DOCS", (doc,))
    monkeypatch.setattr(track, "Sources", lambda: ur.src)

    assert track.main(["check"]) == 1
    assert "new PolyScope X minor: 10.13 → 10.14" in capsys.readouterr().err

    report = tmp_path / "report.md"
    assert track.main(["update", "--report", str(report), "--github-output", str(out)]) == 0
    assert json.loads(target_file.read_text(encoding="utf-8"))["polyscope_x"]["release"] == "10.14.0"
    assert "PolyScope X 10.14.0" in doc.read_text(encoding="utf-8")
    outputs = dict(line.split("=", 1) for line in out.read_text(encoding="utf-8").splitlines())
    assert outputs["changed"] == "true" and outputs["release"] == "10.14.0"
    assert "PolyScope X 10.14.0" in report.read_text(
        encoding="utf-8"
    ) and "| contribution_api | 22 | 22.14.184 |" in report.read_text(encoding="utf-8")

    assert track.main(["check"]) == 0
    out.write_text("", encoding="utf-8")
    assert track.main(["update", "--github-output", str(out)]) == 0
    assert "changed=false" in out.read_text(encoding="utf-8")


def test_cli_pending_is_quiet_until_overdue(ur, tmp_path, monkeypatch):
    ur.standard()
    ur.set(f"/gh/repos/{track.SDK_REPO}/releases?per_page=50", [])
    target_file = tmp_path / "target.json"
    target_file.write_text(json.dumps(_target()), encoding="utf-8")
    monkeypatch.setattr(track, "TARGET", target_file)
    monkeypatch.setattr(track, "Sources", lambda: ur.src)
    monkeypatch.setattr(track, "overdue", lambda notes, today=None: False)
    assert track.main(["check"]) == 0
    monkeypatch.setattr(track, "overdue", lambda notes, today=None: True)
    assert track.main(["check"]) == 1
    assert json.loads(target_file.read_text(encoding="utf-8")) == _target()  # never written while pending


def test_cli_network_failure_is_exit_2(tmp_path, monkeypatch):
    target_file = tmp_path / "target.json"
    target_file.write_text(json.dumps(_target()), encoding="utf-8")
    monkeypatch.setattr(track, "TARGET", target_file)
    real = track.Sources
    monkeypatch.setattr(track, "Sources", lambda: real(notes="http://127.0.0.1:9/{slug}"))
    assert track.main(["check"]) == 2


def test_the_committed_target_is_well_formed():
    t = track.load_target()
    assert re.fullmatch(r"\d+\.\d+", t["polyscope_x"]["minor"])
    assert t["polyscope_x"]["release"].startswith(t["polyscope_x"]["minor"] + ".")
    assert re.fullmatch(r"sha256:[0-9a-f]{64}", t["simulator"]["digest"])
    assert "amd64" in t["simulator"]["architectures"]  # CI runs the sim on amd64
    assert t["simulator"]["matches_sdk"] is True
    assert t["sdk"]["threads"] == track.WORKER_THREADS


# -- e2e.py: the synthetic cockpit must never reach a robot ---------------------------------------


def test_e2e_cockpit_has_no_robot_link_whatever_the_shell_exports():
    """Regression: the e2e's cockpit inherited the shell's robot target, so with
    UR_CELL=ur3 exported a click in the simulator's node made the cockpit send a
    Primary script (locate) to the real UR3e."""
    import e2e

    cmd = e2e.cockpit_command(7999, "http://localhost:8014")
    assert "--no-robot" in cmd and "--fake" in cmd
    env = e2e.cockpit_env(
        {"PATH": "/bin", "HOME": "/h", "UR_CELL": "ur3", "UR_HOST": "192.168.3.3", "UR_PRIMARY_PORT": "30001",
         "PERCEPTRONICS_T_FLANGE_CAMERA": "1,2,3", "PERCEPTRONICS_CORS": "*"}
    )  # fmt: skip
    assert env["PATH"] == "/bin" and env["HOME"] == "/h"
    assert not any(k.startswith(("UR_", "PERCEPTRONICS_")) for k in env)


def test_e2e_teardown_removes_the_sims_anonymous_volume():
    """Regression: `docker rm -f` without -v left each run's inner /var/lib/docker
    (~9 GB) behind; seven runs filled Docker Desktop's disk and the next sim died
    at boot with 'no space left on device'."""
    import e2e

    assert e2e.teardown_command("docker", "urcap-e2e-x") == ["docker", "rm", "-f", "-v", "urcap-e2e-x"]


def test_e2e_boot_wait_fails_fast_when_the_sim_exits():
    """Regression: a simulator that died during boot was waited on for the whole
    boot timeout; the wait now stops as soon as the container is gone."""
    import e2e

    t0 = __import__("time").monotonic()
    with pytest.raises(e2e.E2EError, match="exited"):
        e2e.wait_for("the web UI", lambda: False, 600, 0.01, alive=lambda: False)
    assert __import__("time").monotonic() - t0 < 5


# -- e2e.py: install only once the simulator says it is ready -------------------------------------

# the web-bootstrapper's last lines in a CI run where installing earlier broke PolyScope X's start
# (run 36523255181, 2026-09-29)
_BOOT_MIDWAY = """\
web-bootstrapper-1  | 2026/09/29 04:50:48 [      INFO] Installed URCapX urconnect-main
web-bootstrapper-1  | 2026/09/29 04:50:48 [      INFO] Installing URCapX urconnect
web-bootstrapper-1  | 2026/09/29 04:50:48 [     ERROR] Failed finding URCapX urconnect in urcaps folder
"""
_BOOT_DONE = (
    _BOOT_MIDWAY
    + """\
web-bootstrapper-1  | 2026/09/29 04:50:48 [      INFO] URService replies 404 to deleting urcapID:java-backend
web-bootstrapper-1  | 2026/09/29 04:50:48 [      INFO] Done, time to sleep forever
"""
)


def test_the_simulator_is_ready_only_when_its_bootstrapper_has_finished():
    import e2e

    assert not e2e.bootstrapped(_BOOT_MIDWAY)
    assert e2e.bootstrapped(_BOOT_DONE)


def test_the_urcap_is_installed_only_after_the_simulator_is_ready(monkeypatch, tmp_path):
    import e2e

    events: list[str] = []
    polls = {"n": 0}

    def ready():
        polls["n"] += 1
        events.append(f"ready?{polls['n']}")
        return polls["n"] >= 3  # the bootstrapper finishes on the third look

    monkeypatch.setattr(e2e, "http", lambda url, timeout=10: (200, b'{"state":"done"}'))
    monkeypatch.setattr(e2e.time, "sleep", lambda s: None)

    class Installed(Exception):
        """the fake stops the run at the install: what follows needs a real simulator"""

    def install(*a, **k):
        events.append("install")
        raise Installed

    monkeypatch.setattr(e2e.urcapx, "install", install)
    checks = e2e.Checks()
    with pytest.raises(Installed):
        e2e.install_checks(checks, tmp_path / "x.urcapx", 8000, 30, ready=ready)
    assert events.index("install") > events.index("ready?3"), events
    assert any(c["check"] == "simulator ready" and c["ok"] for c in checks.items)
