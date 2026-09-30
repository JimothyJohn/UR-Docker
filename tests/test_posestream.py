"""perceptronics.posestream: flange from RTDE's TCP + offset, nearest-sample lookup,
reconnect, and the cockpit putting the pose in each frame header."""

from __future__ import annotations

import threading
import time

import pytest

from perceptronics.posestream import PoseStream, flange_from
from urctl.config import RobotConfig
from urctl.pose import Transform

# the 2026-09-27 UR3e reading: RTDE sample and the get_flange_pose script's answer
SAMPLE = {
    "timestamp": 2503.948,
    "actual_TCP_pose": [
        -0.20701026765920397,
        0.3589818037540808,
        -0.1800107492390808,
        3.0765100105844523,
        -0.6359312912467168,
        -2.0745060682249473e-05,
    ],
    "tcp_offset": [0.0, 0.0, 0.163, 0.0, 0.0, 0.0],
}
SCRIPT_FLANGE = [-0.207, 0.359, -0.017, 3.0765, -0.636, 0.0]


def test_flange_matches_the_controllers_script_on_the_ur3e():
    got = flange_from(SAMPLE)
    assert got == pytest.approx(SCRIPT_FLANGE, abs=6e-4)


def test_flange_round_trips_an_arbitrary_offset():
    flange = [0.3, -0.2, 0.1, 0.4, -1.2, 2.0]
    off = [0.01, -0.035, 0.22, 0.25, -0.41, 1.43]
    tcp = Transform.from_pose(flange).compose(Transform.from_pose(off)).to_pose()
    assert flange_from({"actual_TCP_pose": tcp, "tcp_offset": off}) == pytest.approx(flange, abs=1e-9)


@pytest.mark.parametrize(
    "bad", [{}, {"actual_TCP_pose": [0] * 6}, {"actual_TCP_pose": [0] * 5, "tcp_offset": [0] * 6}]
)
def test_incomplete_samples_give_none(bad):
    assert flange_from(bad) is None


def test_at_picks_the_nearest_sample_and_refuses_stale_ones():
    now = [100.0]
    ps = PoseStream(RobotConfig(host="x"), clock=lambda: now[0], history_s=1.0)
    for i in range(10):
        ps.add(100.0 + i * 0.1, [float(i)] * 6)
    pose, age = ps.at(100.52)
    assert pose[0] == 5.0 and age == pytest.approx(0.02)
    assert ps.at(100.0)[0][0] == 0.0
    assert ps.at(99.0) == (None, None)  # a second before the history
    assert ps.at(102.0) == (None, None)  # far after the newest sample
    ps.add(102.0, [9.0] * 6)  # history trimmed to the last second
    assert ps.at(100.0) == (None, None)


class _FakeClient:
    def __init__(self, samples, fail=False):
        self.samples, self.fail = samples, fail

    def __enter__(self):
        if self.fail:
            raise OSError("connection refused")
        return self

    def __exit__(self, *exc):
        return False

    def stream(self):
        yield from self.samples
        while True:  # a live link: nothing new, but open
            time.sleep(0.01)
            yield {}


def test_the_stream_reconnects_after_a_refused_connection():
    attempts = []

    def factory():
        attempts.append(1)
        return _FakeClient([SAMPLE], fail=len(attempts) == 1)

    ps = PoseStream(RobotConfig(host="x"), client_factory=factory)
    ps._stop.wait = lambda s: time.sleep(0.01)  # no real back-off in a test
    ps.start()
    deadline = time.time() + 3
    while ps.latest().get("ok") is not True and time.time() < deadline:
        time.sleep(0.01)
    ps.stop()
    assert len(attempts) >= 2
    assert ps.latest()["flange_pose"] == pytest.approx(SCRIPT_FLANGE, abs=6e-4)


def test_concurrent_adds_and_lookups_never_tear():
    ps = PoseStream(RobotConfig(host="x"), history_s=10.0)
    stop = threading.Event()

    def writer():
        i = 0
        while not stop.is_set():
            ps.add(time.time(), [float(i)] * 6)
            i += 1

    ths = [threading.Thread(target=writer) for _ in range(4)]
    [t.start() for t in ths]
    try:
        for _ in range(2000):
            pose, _ = ps.at(time.time(), max_age_s=1.0)
            assert pose is None or len(set(pose)) == 1  # one sample's values, never mixed
    finally:
        stop.set()
        [t.join() for t in ths]


# -- the cockpit side --------------------------------------------------------------------


def test_frames_carry_the_pose_they_were_taken_at_and_objects_list():
    import json
    import urllib.request
    from http.server import ThreadingHTTPServer

    from perceptronics.config import PerceptionConfig
    from perceptronics.realsense import SyntheticRgbdCamera
    from perceptronics.rgbd import unpack_rgbd
    from perceptronics.webapp import ViewerApp, ViewerHandler

    ps = PoseStream(RobotConfig(host="x"), history_s=60.0)
    app = ViewerApp(
        SyntheticRgbdCamera(width=64, height=48, fps=0), config=PerceptionConfig(), pose_stream=ps
    )
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    base = f"http://127.0.0.1:{srv.server_address[1]}"
    try:
        # no pose yet → the header has none (and nothing invented)
        app.start()
        deadline = time.monotonic() + 5
        while app.latest()[1] is None and time.monotonic() < deadline:
            time.sleep(0.01)
        header, _, _ = unpack_rgbd(urllib.request.urlopen(base + "/api/rgbd", timeout=5).read())
        assert "flange_pose" not in header
        assert json.load(urllib.request.urlopen(base + "/api/robot/pose", timeout=5))["ok"] is False
        # a live pose beside every frame arrival
        stop = threading.Event()

        def feed():
            while not stop.is_set():
                ps.add(time.time(), SCRIPT_FLANGE)
                time.sleep(0.01)

        threading.Thread(target=feed, daemon=True).start()
        time.sleep(0.1)
        seq = header["seq"]
        header, _, _ = unpack_rgbd(
            urllib.request.urlopen(f"{base}/api/rgbd?after={seq}&timeout_ms=2000", timeout=5).read()
        )
        stop.set()
        assert header["flange_pose"] == pytest.approx(SCRIPT_FLANGE) and header["pose_age_s"] < 0.25
        pose = json.load(urllib.request.urlopen(base + "/api/robot/pose", timeout=5))
        assert pose["ok"] and pose["flange_pose"] == pytest.approx(SCRIPT_FLANGE)
        req = urllib.request.Request(
            base + "/api/objects", data=b"{}", headers={"Content-Type": "application/json"}
        )
        objs = json.load(urllib.request.urlopen(req, timeout=10))
        assert objs["ok"] and isinstance(objs["objects"], list)
    finally:
        srv.shutdown()
        srv.server_close()
        app.stop()
