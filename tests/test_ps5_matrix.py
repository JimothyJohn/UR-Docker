"""The PolyScope 5 version matrix's pure parts (urcap/ps5_matrix.py): the port scheme, the
generated compose file and the workflow held to MATRIX, the add-only Docker Hub tag
checker, and the URCap evidence parsers
(polyscope.log errors, the Felix shell's ps / inspect output — real captures under
tests/fixtures/ps5_matrix/)."""

from __future__ import annotations

import json
import random
import re
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "urcap"))

import ps5_matrix as m  # noqa: E402

COMPOSE = (REPO / "docker-compose.ps5-matrix.yml").read_text(encoding="utf-8")
DEFAULT_COMPOSE = (REPO / "docker-compose.yml").read_text(encoding="utf-8")
WORKFLOW = (REPO / ".github" / "workflows" / "urcap5-matrix.yml").read_text(encoding="utf-8")
FIXTURES = REPO / "tests" / "fixtures" / "ps5_matrix"


# -- ports ------------------------------------------------------------------------------------


def test_matrix_host_ports_never_collide_with_each_other_or_the_default_sims():
    seen: dict[int, str] = {}
    for v in m.MATRIX.values():
        for port in m.host_ports(v):
            assert port not in seen, f"{v.tag} reuses host port {port} of {seen.get(port)}"
            assert port not in m.DEFAULT_PORTS, f"{v.tag} host port {port} is a default-sim/cockpit port"
            seen[port] = v.tag


@pytest.mark.parametrize("minor", range(0, m.MAX_MINOR + 1))
def test_the_port_scheme_scales_to_every_minor_it_accepts(minor):
    """Any minor UR may publish (0..98) gets its own block, clear of every default port and
    below both the Linux (32768) and macOS (49152) ephemeral ranges."""
    v = m._version(f"5.{minor}.0")
    ports = m.host_ports(v)
    assert len(set(ports)) == len(ports)
    assert all(m.PORT_BASE <= p < 29999 and p not in m.DEFAULT_PORTS for p in ports)
    assert {p // 100 for p in ports} == {(m.PORT_BASE + 100 * minor) // 100}  # its own block
    assert v.dashboard % 100 == 99 and v.primary % 100 == 1 and v.rtde % 100 == 4 and v.novnc % 100 == 80


@pytest.mark.parametrize("tag", ["5.99.0", "5.100.1", "5.-1.0"])
def test_the_port_scheme_refuses_a_minor_it_cannot_place(tag):
    with pytest.raises(ValueError):
        m._version(tag)


def test_default_ports_cover_every_host_port_docker_compose_yml_publishes():
    published = {int(a) for a, _ in re.findall(r'"(\d+):(\d+)"', DEFAULT_COMPOSE)}
    assert published, "docker-compose.yml publishes nothing? the regex drifted"
    assert published <= m.DEFAULT_PORTS, f"add {sorted(published - m.DEFAULT_PORTS)} to DEFAULT_PORTS"


# -- the compose file and the workflow are MATRIX's -------------------------------------------


def test_the_committed_compose_file_is_the_generated_one():
    assert COMPOSE == m.render_compose(), (
        "docker-compose.ps5-matrix.yml is stale: "
        "`python3 urcap/ps5_matrix.py compose > docker-compose.ps5-matrix.yml`"
    )


def _service_block(name: str) -> str:
    match = re.search(rf"^  {re.escape(name)}:\n((?:    .*\n|\n)+)", COMPOSE, re.M)
    assert match, f"{name} is not a service in docker-compose.ps5-matrix.yml"
    return match.group(1)


@pytest.mark.parametrize("minor", list(m.MATRIX))
def test_compose_file_publishes_exactly_the_table(minor):
    v = m.MATRIX[minor]
    block = _service_block(v.service)
    assert f"image: {m.IMAGE}:{v.tag}\n" in block
    mapped = dict((int(c), int(h)) for h, c in re.findall(r'"(\d+):(\d+)"', block))
    assert mapped == {29999: v.dashboard, 30001: v.primary, 30004: v.rtde, 6080: v.novnc}


def test_compose_file_has_no_service_the_matrix_does_not_name():
    services = set(re.findall(r"^  (ursim-[\w-]+):$", COMPOSE, re.M))
    assert services == {v.service for v in m.MATRIX.values()}


def test_compose_file_header_table_matches_the_matrix():
    for v in m.MATRIX.values():
        cells = [v.service, re.escape(v.tag), v.dashboard, v.primary, v.rtde, v.novnc, v.pick]
        row = r"#\s+" + r"\s+".join(map(str, cells)) + r"\n"
        assert re.search(row, COMPOSE), f"header row for {v.service} is stale"


def test_the_workflow_takes_its_versions_from_ps5_matrix_list():
    assert "python3 urcap/ps5_matrix.py list" in WORKFLOW
    assert "polyscope: ${{ fromJSON(needs.setup.outputs.versions) }}" in WORKFLOW
    assert "fail-fast: false" in WORKFLOW and re.search(r"max-parallel: \d+", WORKFLOW)
    assert not re.search(r"polyscope: \[", WORKFLOW), "a hand-listed version list is back"


def test_list_prints_the_minors_as_json(capsys):
    assert m.main(["list"]) == 0
    assert json.loads(capsys.readouterr().out) == list(m.MATRIX)


def test_compose_prints_the_generated_file(capsys):
    assert m.main(["compose"]) == 0
    assert capsys.readouterr().out == m.render_compose()


def test_matrix_is_the_newest_image_of_every_minor_oldest_first():
    tags = [v.tag for v in m.MATRIX.values()]
    assert tags == sorted(tags, key=m.tag_key)
    assert len({v.minor for v in m.MATRIX.values()}) == len(tags)
    minors = [int(v.minor.split(".")[1]) for v in m.MATRIX.values()]
    assert minors == list(range(minors[0], minors[-1] + 1)), (
        "a minor between the oldest and newest is missing"
    )


def test_every_excluded_minor_says_why_and_is_not_in_the_matrix():
    for minor, why in m.EXCLUDED.items():
        assert minor not in m.MATRIX and len(why) > 20, minor


# -- Docker Hub tags --------------------------------------------------------------------------

# The listing on 2026-09-28 (hub.docker.com, universalrobots/ursim_e-series, 51 tags).
HUB_2026_09_28 = (
    "latest 5.26 5.25 5.24 5.23 5.22 5.21 5.20 5.19 5.18 5.17 5.16 5.15 5.14 5.13 5.12 5.11 "
    "5.10 5.9 5.26.1 5.26.0 5.25.2 5.25.1 5.25.0 5.24.0 5.23.0 5.22.2 5.22.0 5.21.3 5.21.0 "
    "5.20.0 5.19.0 5.18.1 5.18.0 5.17.3 5.16.1 5.15.2 5.14.6 5.13.1 5.12.8 5.12.7 5.12.6 "
    "5.12.5 5.11.11 5.10.2 5.9.4 5.8 5.7 5.6 5.5 5.4"
).split()
MATRIX_TAGS = [v.tag for v in m.MATRIX.values()]


@pytest.mark.parametrize(
    "tag, parsed",
    [
        ("5.26.1", (5, 26, 1)),
        ("5.9.4", (5, 9, 4)),
        ("5.26", None),
        ("latest", None),
        ("5.26.1-rc1", None),
        ("6.0.0", None),
        ("05.26.1", None),
        ("5.026.1", None),
        (" 5.26.1", None),
        ("5.26.1\n", None),
        ("", None),
        ("٥.٢٦.١", None),  # Arabic-Indic digits: \d matches them, int() would accept them
    ],
)
def test_parse_tag(tag, parsed):
    assert m.parse_tag(tag) == parsed


@pytest.mark.parametrize(
    "tag, key",
    [
        ("5.26.1", (5, 26, 1)),
        ("5.8", (5, 8, -1)),
        ("5.4", (5, 4, -1)),
        ("6.1", None),
        ("5.08", None),
        ("5", None),
        ("5.8.", None),
        ("latest", None),
        ("٥.٨", None),
    ],
)
def test_tag_key_orders_a_bare_minor_before_its_patches(tag, key):
    assert m.tag_key(tag) == key


def test_the_real_listing_gives_the_committed_matrix():
    assert m.newest_per_minor(HUB_2026_09_28) == MATRIX_TAGS
    assert m.tag_drift(HUB_2026_09_28, MATRIX_TAGS) == []


def test_a_new_minor_says_add_it_and_never_drop_one():
    tags = [*HUB_2026_09_28, "5.27.0"]
    assert m.tag_drift(tags, MATRIX_TAGS) == [f"PolyScope 5.27 exists ({m.IMAGE}:5.27.0): add it"]


def test_an_older_minor_appearing_is_added_too():
    tags = [*HUB_2026_09_28, "5.3"]
    assert m.tag_drift(tags, MATRIX_TAGS) == [f"PolyScope 5.3 exists ({m.IMAGE}:5.3): add it"]


def test_a_new_patch_says_bump_it():
    tags = [*HUB_2026_09_28, "5.25.10"]
    assert m.tag_drift(tags, MATRIX_TAGS) == ["PolyScope 5.25.10 exists: bump 5.25.2 to it"]


def test_a_first_patch_tag_of_a_bare_minor_says_bump_it():
    tags = [*HUB_2026_09_28, "5.8.2"]
    assert m.tag_drift(tags, MATRIX_TAGS) == ["PolyScope 5.8.2 exists: bump 5.8 to it"]


def test_an_excluded_minor_is_not_asked_for():
    tags = [*HUB_2026_09_28, "5.27.0"]
    assert m.tag_drift(tags, MATRIX_TAGS, {"5.27": "does not boot: ..."}) == []


def test_a_matrix_tag_gone_from_the_hub_is_reported_not_dropped():
    tags = [t for t in HUB_2026_09_28 if t != "5.25.2"]
    problems = m.tag_drift(tags, MATRIX_TAGS)
    assert problems == [f"{m.IMAGE}:5.25.2 is not on Docker Hub (the matrix can't pull it)"]
    assert not any("drop" in p for p in problems)


def test_a_smaller_matrix_is_told_to_add_every_minor_it_lacks():
    problems = m.tag_drift(HUB_2026_09_28, ["5.24.0", "5.25.2", "5.26.1"])
    assert len(problems) == len(MATRIX_TAGS) - 3
    assert all(p.startswith("PolyScope 5.") and p.endswith(": add it") for p in problems)
    assert not any("drop" in p for p in problems)


def test_non_polyscope5_tags_never_count():
    tags = [*HUB_2026_09_28, "6.0.0", "6.1", "5.30.0-beta", "latest"]
    assert m.tag_drift(tags, MATRIX_TAGS) == []


@pytest.mark.parametrize("seed", range(200))
def test_newest_per_minor_is_order_independent_and_picks_each_minors_newest(seed):
    rng = random.Random(seed)
    versions = {(5, rng.randint(0, 40), rng.randint(0, 30)) for _ in range(rng.randint(0, 40))}
    bare = {rng.randint(0, 40) for _ in range(rng.randint(0, 6))}
    junk = ["latest", "6.1.0", "5.2.3-rc", "x", "6.2"]
    tags = [".".join(map(str, v)) for v in versions] + [f"5.{b}" for b in bare]
    tags += rng.sample(junk, rng.randint(0, len(junk)))
    rng.shuffle(tags)
    got = m.newest_per_minor(tags)
    # Reference: per minor the max patch; a bare tag only where the minor has no patch tag.
    by_minor: dict[int, str] = {}
    for _, minor, patch in sorted(versions):
        by_minor[minor] = f"5.{minor}.{patch}"
    for b in bare:
        by_minor.setdefault(b, f"5.{b}")
    assert got == [by_minor[mi] for mi in sorted(by_minor)]
    assert m.newest_per_minor(list(reversed(tags))) == got
    # A matrix equal to the answer never drifts; any matrix missing one is told to add it.
    assert m.tag_drift(tags, got) == []
    if got:
        drop = rng.randrange(len(got))
        assert m.tag_drift(tags, got[:drop] + got[drop + 1 :]) == [
            f"PolyScope {m.minor_of(got[drop])} exists ({m.IMAGE}:{got[drop]}): add it"
        ]


def test_the_patch_comparison_is_numeric_not_lexical():
    assert m.newest_per_minor(["5.25.9", "5.25.10", "5.25.2"]) == ["5.25.10"]
    assert m.newest_per_minor(["5.9.4", "5.10.2", "5.100.0"]) == ["5.9.4", "5.10.2", "5.100.0"]


# -- the URCap's evidence: polyscope.log errors, Felix shell ps / services ----------------


def test_log_errors_ignore_a_log_that_never_names_the_urcap():
    log = "INFO Starting PolyScope\njava.lang.NullPointerException\n\tat com.ur.foo.Bar(Bar.java:1)\n"
    assert m.log_errors(log) == []


def test_log_errors_on_a_real_clean_load_are_empty():
    """tests/fixtures/ps5_matrix/polyscope-5.24.0-clean.log is a real excerpt (CI matrix
    run 36510721492, 2026-09-28): our jar added and located by URCapHelper, next to UR's
    own ERROR lines and a licence-omitted URCap that is not ours."""
    log = (FIXTURES / "polyscope-5.24.0-clean.log").read_text(encoding="utf-8")
    assert "realsense-pilot-ps5.jar" in log and "ERROR" in log and "Omitted licenced URCap" in log
    assert m.log_errors(log) == []


def test_log_errors_flag_a_stack_frame_in_our_package_with_its_exception():
    # The shape of the one PolyScope-5-only bug so far (README: setBorder refused).
    log = (
        "12:00:01 INFO something\n"
        "com.ur.urcap.api.domain.AuthorizationException: Method not supported from URCaps\n"
        "\tat com.ur.polyscope.Guard.check(Guard.java:10)\n"
        "\tat com.nickarmenta.perceptronic.PilotView.buildUI(PilotView.java:42)\n"
    )
    errors = m.log_errors(log)
    assert errors[0].startswith("com.ur.urcap.api.domain.AuthorizationException")
    assert any("PilotView.buildUI" in e for e in errors)


@pytest.mark.parametrize(
    "line",
    [
        "ERROR Could not start bundle com.olympuscontrols.realsensepilot [42]: Unresolved constraint",
        "INFO Omitted licenced URCap: com.olympuscontrols.realsensepilot due to missing license.",
        "WARN URCap realsense-pilot-ps5.jar rejected: incompatible API version",
    ],
)
def test_log_errors_flag_a_failure_line_naming_the_urcap(line):
    assert m.log_errors(f"INFO fine\n{line}\nINFO fine\n") == [line]


def test_log_errors_ignore_other_bundles_that_merely_look_similar():
    log = "ERROR bundle com.example.realsense_other failed\nERROR com.olympus.other failed\n"
    assert m.log_errors(log) == []


PS = """
START LEVEL 1
   ID   State         Level  Name
[   0] [Active     ] [    0] System Bundle (7.0.3)
[   1] [Active     ] [    1] aopalliance (1.0)
[ 164] [Active     ] [    1] RealSense Pilot (0.4.0)
[ 165] [Resolved   ] [    1] Other Thing (1.0)
-> """


def test_parse_ps_reads_every_row():
    rows = m.parse_ps(PS)
    assert [r["id"] for r in rows] == [0, 1, 164, 165]
    assert rows[2] == {"id": 164, "state": "Active", "level": 1, "name": "RealSense Pilot (0.4.0)"}


@pytest.mark.parametrize("fixture", sorted(FIXTURES.glob("felix-ps-*.txt")))
def test_the_urcap_is_active_in_real_shell_output(fixture):
    """A real ``ps`` from each version's Felix remote shell (CI matrix run 36510721492,
    2026-09-28, URCap 0.4.0): every row parses and ours is Active."""
    text = fixture.read_text(encoding="utf-8")
    rows = m.parse_ps(text)
    assert rows[0] == {"id": 0, "state": "Active", "level": 0, "name": "System Bundle (7.0.3)"}
    assert len(rows) == sum(1 for line in text.splitlines() if line.startswith("["))
    ours = m.find_bundle(rows, "RealSense Pilot", "0.4.0")
    assert ours and ours["state"] == "Active"


@pytest.mark.parametrize(
    "name, version, found",
    [
        ("RealSense Pilot", "0.4.0", 164),
        ("RealSense Pilot", "0.5.0", None),  # a stale jar in the image is not the one under test
        ("RealSense", "0.4.0", None),
        ("realsense pilot", "0.4.0", None),
    ],
)
def test_find_bundle_matches_name_and_version_exactly(name, version, found):
    row = m.find_bundle(m.parse_ps(PS), name, version)
    assert (row["id"] if row else None) == found


def test_missing_services_names_what_the_activator_did_not_register():
    both = (
        "RealSense Pilot (164) provides:\n-------------------------------\n"
        "objectClass = com.ur.urcap.api.contribution.installation.swing.SwingInstallationNodeService\n"
        "service.id = 301\n----\n"
        "objectClass = [com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService]\n"
        "service.id = 302\n"
    )
    toolbar = "com.ur.urcap.api.contribution.toolbar.swing.SwingToolbarService"
    assert m.missing_services(both) == [toolbar]
    assert m.missing_services(both + "----\nobjectClass = " + toolbar + "\nservice.id = 303\n") == []
    one = both.split("service.id = 301")[0]
    assert m.missing_services(one) == [
        "com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService",
        toolbar,
    ]
    assert m.missing_services("") == list(m.NODE_SERVICES)
    # Named in prose, not as an objectClass: not registered.
    assert m.missing_services("could not register SwingProgramNodeService") == list(m.NODE_SERVICES)


@pytest.mark.parametrize("fixture", sorted(FIXTURES.glob("felix-services-*.txt")))
def test_missing_services_on_real_listings(fixture):
    """Real ``inspect service capability <id>`` listings of the URCap, one per version (CI
    matrix run 36512194205, 2026-09-28, URCap 0.4.0 — before the toolbar service)."""
    assert m.missing_services(fixture.read_text(encoding="utf-8"), m.NODE_SERVICES[:2]) == [], fixture.name


@pytest.mark.parametrize(
    "reply, ready",
    [
        ("Robotmode: POWER_OFF", True),
        ("Robotmode: IDLE", True),
        ("Robotmode: RUNNING\n", True),
        ("Robotmode: NO_CONTROLLER", False),
        ("Robotmode: DISCONNECTED", False),  # PolyScope 5.5 answered this, then dropped `power on`
        ("Robotmode: BOOTING", False),
        ("", False),
        ("Connected: Universal Robots Dashboard Server", False),
        ("Robotmode: POWER_OFF; rm -rf /", False),
    ],
)
def test_the_controller_is_ready_only_once_polyscope_is_connected_to_it(reply, ready):
    assert m.controller_ready(reply) is ready


# -- a tag re-pushed with other contents is drift (Nick, 2026-09-29) --------------------------------


def test_every_matrix_image_has_its_recorded_version():
    assert list(m.IMAGE_VERSIONS) == list(m.TAGS)
    for tag, version in m.IMAGE_VERSIONS.items():
        assert version.startswith(tag if tag.count(".") == 1 else tag), (tag, version)
        if tag.count(".") == 2:  # a patch tag carries exactly that patch
            assert version == tag
        else:  # a bare minor tag carries some patch of that minor
            assert re.fullmatch(re.escape(tag) + r"\.[0-9]+", version), (tag, version)


def test_unchanged_images_are_no_drift():
    assert m.version_drift(m.IMAGE_VERSIONS, dict(m.IMAGE_VERSIONS)) == []


def test_a_bare_tag_re_pushed_with_a_new_patch_is_drift():
    actual = dict(m.IMAGE_VERSIONS, **{"5.4": "5.4.4"})
    (p,) = m.version_drift(m.IMAGE_VERSIONS, actual)
    assert "5.4" in p and "5.4.4" in p and "was 5.4.3" in p and "re-pushed" in p


def test_an_image_with_no_readable_version_is_reported_not_ignored():
    actual = dict(m.IMAGE_VERSIONS, **{"5.26.1": None})
    (p,) = m.version_drift(m.IMAGE_VERSIONS, actual)
    assert "5.26.1" in p and "no readable VERSION" in p


def test_check_tags_reports_a_re_pushed_image(monkeypatch, capsys):
    newest = list(m.TAGS)
    monkeypatch.setattr(m, "fetch_tags", lambda *a, **k: newest)
    monkeypatch.setattr(m, "fetch_versions", lambda tags: dict(m.IMAGE_VERSIONS, **{"5.8": "5.8.3"}))
    assert m.main(["check-tags"]) == 1
    assert "5.8.3" in capsys.readouterr().err
