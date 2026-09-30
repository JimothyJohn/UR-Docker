"""The stand-alone pick server beside a running cockpit (``perceptronics pick-server``).

Contract: against a real cockpit HTTP server on the synthetic block scene, the
sidecar's pick socket answers exactly what the cockpit's own planner answers, its
teach routes match the cockpit's, and every other request reaches the cockpit
unchanged. Then the hostile side: request targets that would pick another
upstream, oversized and malformed bodies, the cockpit going away, and the one
thing it must never do — send a script to the controller.
"""

from __future__ import annotations

import json
import math
import os
import random
import socket
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer

import pytest

from perceptronics.config import PerceptionConfig
from perceptronics.picknode import PickServer, parse_preview_request
from perceptronics.picksidecar import CockpitFrames, Sidecar, SidecarServer, _say
from perceptronics.robotlink import RobotLink
from perceptronics.webapp import ViewerApp, ViewerHandler
from tests.test_pickcycle import SceneCamera
from urctl import Robot, RobotConfig
from urctl.pose import pose_trans

FLANGE_LINE = "FIND p[0.4, 0.0, 0.5, 0.0, 3.141592653589793, 0.0]\n"


@pytest.fixture
def cockpit():
    app = ViewerApp(
        SceneCamera(),
        config=PerceptionConfig(),
        robot=RobotLink(RobotConfig(host="fake-ur.invalid"), dry_run=True),
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield app, f"http://127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()
    app.stop()


def _sidecar(base: str, robot: Robot | None = None):
    robot = robot or Robot(RobotConfig(host="fake-ur.invalid"), dry_run=True)
    side = Sidecar(CockpitFrames(base, timeout_s=5.0), lambda: robot.get_flange_pose(), tip_m=0.163)
    pick = PickServer("127.0.0.1", 0, side.planner)
    side.pick_port = pick.server_address[1]
    http = SidecarServer("127.0.0.1", 0, side)
    pick.start()
    http.start()
    return side, pick, http


@pytest.fixture
def sidecar(cockpit):
    app, base = cockpit
    side, pick, http = _sidecar(base)
    yield app, side, pick.server_address[1], f"http://127.0.0.1:{http.server_address[1]}"
    http.stop()
    pick.stop()


def _ask(port: int, line: str) -> str:
    with socket.create_connection(("127.0.0.1", port), timeout=10) as s:
        s.sendall(line.encode())
        return s.makefile("r").readline()


def _call(url: str, body: bytes | None = None, *, method: str | None = None) -> tuple[int, bytes]:
    req = urllib.request.Request(url, data=body, method=method, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=15) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read()


def _raw(port: int, request: bytes) -> bytes:
    with socket.create_connection(("127.0.0.1", port), timeout=10) as s:
        s.sendall(request)
        chunks = []
        while chunk := s.recv(65536):
            chunks.append(chunk)
    return b"".join(chunks)


# -- the same answers as the cockpit's own ----------------------------------------------------


def test_the_pick_socket_answers_what_the_cockpits_own_planner_answers(sidecar):
    app, _, port, _ = sidecar
    got = _ask(port, FLANGE_LINE)
    assert float(got[1:].split(",")[0]) == 1.0, got
    assert got == app.pick_planner().answer(FLANGE_LINE)


def test_detect_lists_the_cockpits_blocks_and_names_the_sidecars_socket(sidecar):
    app, _, port, url = sidecar
    status, body = _call(url + "/api/pick/detect")
    out = json.loads(body)
    assert status == 200 and out["ok"] and out["pick_port"] == port and out["handeye"]
    assert out["blocks"] == app.pick_detect()["blocks"] and len(out["blocks"]) == 1


def test_preview_puts_the_fingertips_over_and_into_the_block(sidecar):
    _, side, _, url = sidecar
    status, body = _call(
        url + "/api/pick/preview", json.dumps({"grip_below_mm": 15, "hover_mm": 40}).encode()
    )
    out = json.loads(body)
    assert status == 200 and out["ok"], out
    tips = lambda pose: pose_trans(pose, [0.0, 0.0, side.tip_m, 0.0, 0.0, 0.0])[:3]  # noqa: E731
    assert math.dist(tips(out["hover_pose"]), out["centre"]) == pytest.approx(0.040, abs=1e-6)
    assert math.dist(tips(out["grip_pose"]), out["centre"]) == pytest.approx(0.015, abs=1e-6)


def test_parallel_controllers_each_get_the_answer(sidecar):
    app, _, port, _ = sidecar
    want = app.pick_planner().answer(FLANGE_LINE)
    results, errors = [], []

    def one():
        try:
            results.append(_ask(port, FLANGE_LINE))
        except Exception as exc:  # pragma: no cover - reported below
            errors.append(exc)

    threads = [threading.Thread(target=one) for _ in range(8)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(30)
    assert not errors and results == [want] * 8


# -- everything else is the cockpit's -----------------------------------------------------------


def test_other_routes_reach_the_cockpit_unchanged(sidecar, cockpit):
    _, base = cockpit
    _, _, _, url = sidecar
    status, body = _call(url + "/api/info")
    assert status == 200 and set(json.loads(body)) == set(json.loads(_call(base + "/api/info")[1]))
    status, body = _call(url + "/api/rgbd")
    assert status == 200 and body[:4] == b"RGBD"
    # the cockpit's own refusals come back with its status and body
    assert _call(url + "/api/no-such-route", b"{}") == _call(base + "/api/no-such-route", b"{}")
    assert _call(url + "/api/no-such-route", b"{}")[0] == 404


# -- hostile requests ---------------------------------------------------------------------------


@pytest.mark.parametrize(
    "target",
    [
        b"http://169.254.169.254/latest/meta-data/",
        b"http://localhost:22/",
        b"//evil.example/api/info",
        b"*",
    ],
)
def test_a_request_target_that_names_another_host_is_refused(sidecar, target):
    """Absolute URIs are refused; ``//host/...`` the stdlib server already collapses to a
    path, which then only ever reaches the configured cockpit (its 404)."""
    _, _, _, url = sidecar
    port = int(url.rsplit(":", 1)[1])
    reply = _raw(port, b"GET " + target + b" HTTP/1.1\r\nHost: x\r\nConnection: close\r\n\r\n")
    status = int(reply.split(b" ", 2)[1])
    assert status == (404 if target.startswith(b"//") else 400), reply[:120]
    if status == 404:
        assert b"no route" in reply  # the cockpit's own answer, not another host's


def test_an_oversized_body_is_refused_before_it_is_read(sidecar):
    _, _, _, url = sidecar
    port = int(url.rsplit(":", 1)[1])
    reply = _raw(
        port,
        b"POST /api/pick/preview HTTP/1.1\r\nHost: x\r\n"
        b"Content-Length: 999999999\r\nConnection: close\r\n\r\n{}",
    )
    assert b" 413 " in reply.split(b"\r\n", 1)[0]


@pytest.mark.parametrize(
    "body",
    [b"not json", b"[1, 2]", b'{"grip_below_mm": 99}', b'{"hover_mm": "NaN"}', b'{"u": "left"}', b'"x"'],
)
def test_malformed_previews_are_refused_with_a_reason(sidecar, body):
    _, _, _, url = sidecar
    status, out = _call(url + "/api/pick/preview", body)
    assert status == 400 and json.loads(out)["error"].startswith("bad request")


def test_preview_fields_fuzz_to_a_value_or_a_refusal():
    rng = random.Random(20260927)
    pool = [0, -1, 5, 60, 61, 300, 301, 1e308, -0.0, "15", "x", None, [], {}, "nan", "inf", True, 2**70]
    for _ in range(2000):
        payload = {k: rng.choice(pool) for k in ("u", "v", "grip_below_mm", "hover_mm") if rng.random() < 0.7}
        try:
            pixel, grip, hover = parse_preview_request(payload)
        except ValueError:
            continue
        assert 0.0 <= grip <= 60.0 and 0.0 <= hover <= 300.0
        assert pixel is None or (pixel[0] >= 0 and pixel[1] >= 0)


def test_with_the_cockpit_gone_every_answer_is_a_refusal_not_a_hang():
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        dead = f"http://127.0.0.1:{s.getsockname()[1]}"  # bound, never listening: refused
    side, pick, http = _sidecar(dead)
    try:
        url = f"http://127.0.0.1:{http.server_address[1]}"
        reply = _ask(pick.server_address[1], FLANGE_LINE)
        assert float(reply[1:].split(",")[0]) in (-3.0, -4.0), reply  # no hand-eye / no frame
        status, body = _call(url + "/api/pick/detect")
        assert status == 200 and not json.loads(body)["ok"]
        status, body = _call(url + "/api/info")
        assert status == 502 and "cockpit unreachable" in json.loads(body)["error"]
    finally:
        http.stop()
        pick.stop()


def test_log_lines_stay_one_line(capsys):
    _say("pick FIND: found\n[pick-server] forged line\r\x1b[2J")
    err = capsys.readouterr().err
    assert err.count("\n") == 1 and "\r" not in err


# -- it never sends the controller a script ------------------------------------------------------


def test_the_preview_flange_never_falls_back_to_a_primary_script():
    """With the state broadcast unreachable, ``script_fallback=False`` answers ok: False
    and opens nothing on Primary (a script there would replace the operator's program)."""
    primary = socket.socket()
    primary.bind(("127.0.0.1", 0))
    primary.listen(4)
    primary.settimeout(0.5)
    with socket.socket() as closed:
        closed.bind(("127.0.0.1", 0))
        secondary_port = closed.getsockname()[1]
    robot = Robot(
        RobotConfig(
            host="127.0.0.1",
            primary_port=primary.getsockname()[1],
            secondary_port=secondary_port,
            timeout=1.0,
        )
    )
    try:
        out = robot.get_flange_pose(script_fallback=False)
        assert out["ok"] is False and out.get("error")
        with pytest.raises(socket.timeout):
            primary.accept()
    finally:
        primary.close()


def test_an_unwritable_log_falls_back_to_the_user_log_dir(tmp_path, monkeypatch):
    import perceptronics.picksidecar as sidecar

    locked = tmp_path / "captures"
    locked.mkdir()
    locked.chmod(0o500)  # what a sudo cockpit run leaves behind: not ours to write
    monkeypatch.setattr(sidecar, "user_log_dir", lambda: tmp_path / "user-logs")
    try:
        got = sidecar.writable_log(locked / "pick-server.log")
        if os.access(locked, os.W_OK):  # running as root: nothing is locked
            pytest.skip("the test runs as root; the directory is writable")
        assert got == tmp_path / "user-logs" / "pick-server.log" and got.exists()
        free = tmp_path / "free" / "pick-server.log"
        assert sidecar.writable_log(free) == free  # a writable preference is kept
    finally:
        locked.chmod(0o700)


# -- the part's rough size on the teach screen's routes ----------------------------------------


@pytest.mark.parametrize("via", ["cockpit", "sidecar"])
def test_detect_and_preview_take_the_part_size(sidecar, cockpit, via):
    _, _, _, side_url = sidecar
    url = side_url if via == "sidecar" else cockpit[1]
    status, body = _call(url + "/api/pick/detect?part=54x43x40&tol=15")
    out = json.loads(body)
    assert status == 200 and out["part"]["length_mm"] == 54.0 and len(out["blocks"]) == 1
    status, body = _call(url + "/api/pick/detect?part=200x150")
    out = json.loads(body)
    assert status == 200 and out["blocks"] == [] and out["rejected"][0]["why"] == "too short"
    status, body = _call(url + "/api/pick/preview", json.dumps({"part": "200x150"}).encode())
    assert status == 200 and json.loads(body)["status"] == -7
    status, body = _call(url + "/api/pick/preview", json.dumps({"part": "54x43x40", "tol": 15}).encode())
    assert status == 200 and json.loads(body)["ok"]


@pytest.mark.parametrize("via", ["cockpit", "sidecar"])
@pytest.mark.parametrize("query", ["part=60", "part=60x40&tol=0", "part=../../etc", "part=60x40&tol=%00"])
def test_a_malformed_part_size_is_refused_with_a_reason(sidecar, cockpit, via, query):
    url = sidecar[3] if via == "sidecar" else cockpit[1]
    status, body = _call(url + "/api/pick/detect?" + query)
    assert status == 400 and json.loads(body)["error"].startswith("bad request")
    status, body = _call(url + "/api/pick/preview", json.dumps({"part": query.split("=", 1)[1]}).encode())
    assert status == 400 and json.loads(body)["error"].startswith("bad request")
