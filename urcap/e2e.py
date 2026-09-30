#!/usr/bin/env python3
"""The URCap in the simulator it is built for: boot, install, load, click — end to end.

Runs the pinned PolyScope X simulator (``urcap/target.json``, by digest) in a
throwaway container, installs a fresh build of ``urcap/perceptronic`` through
the System Manager endpoint (``urcapx.install``), checks nginx serves exactly the
files that were packaged, then drives PolyScope's own UI headlessly:

* Application → Perceptronic: the node's element renders with its i18n title,
  the behavior worker and presenter load without a page error from our files;
* the node goes live against a synthetic cockpit (``perceptronics gui --fake --no-robot``,
  with none of the shell's ``UR_*`` / ``PERCEPTRONICS_*``: it never reaches a robot);
  hover reads depth, a click segments (``POST /api/segment`` ok) and asks to locate;
* the PolyScope services Move (PolyScope) relies on answer on this release:
  ``getKinematicInfo`` (6 DH rows), ``getJointPositions``,
  ``convertJointPositionsToTcpPose`` and an FK → ``getInverseKinematics`` round
  trip that lands back on the current joints (``autoMove`` itself is not called —
  it opens the hold-to-move screen);
* the cockpit URL saved through ``applicationNodeService.updateNode`` survives a reload.

The browser half needs Playwright (``uv run --with playwright==1.63.0 python
urcap/e2e.py``; ``playwright install chromium`` once) and the ``perceptronics``
package; ``--no-browser`` stops after the install checks. Needs Docker, and the
simulator needs ``--privileged``.

    uv run --with playwright==1.63.0 python urcap/e2e.py                   # the pinned image
    uv run --with playwright==1.63.0 python urcap/e2e.py --image universalrobots/ursim_polyscopex:10.14.0
    python3 urcap/e2e.py --no-browser --keep                               # leave the sim running
"""

from __future__ import annotations

import argparse
import contextlib
import json
import os
import platform
import re
import shutil
import socket
import subprocess
import sys
import tarfile
import tempfile
import time
import urllib.error
import urllib.request
import uuid
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import track  # noqa: E402
import urcapx  # noqa: E402

REPO = HERE.parent
VENDOR, URCAP_ID, ARCHIVE = "nickarmenta", "perceptronic", "perceptronic-frontend"
TAG = "nickarmenta-perceptronic"


class E2EError(RuntimeError):
    pass


class Checks:
    """An ordered pass/fail record; the first failure stops the run."""

    def __init__(self) -> None:
        self.items: list[dict] = []

    def ok(self, name: str, detail: str = "") -> None:
        self.items.append({"check": name, "ok": True, "detail": detail})
        print(f"  ok    {name}" + (f" — {detail}" if detail else ""), flush=True)

    def fail(self, name: str, detail: str) -> None:
        self.items.append({"check": name, "ok": False, "detail": detail})
        print(f"  FAIL  {name} — {detail}", flush=True)
        raise E2EError(f"{name}: {detail}")

    def expect(self, cond: bool, name: str, detail: str = "") -> None:
        (self.ok if cond else self.fail)(name, detail)

    @property
    def passed(self) -> bool:
        return bool(self.items) and all(i["ok"] for i in self.items)


def free_port() -> int:
    with socket.socket() as s:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]


def http(url: str, timeout: float = 10) -> tuple[int, bytes]:
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            return r.status, r.read()
    except urllib.error.HTTPError as exc:
        return exc.code, b""
    except (urllib.error.URLError, OSError):
        return 0, b""


def wait_for(what: str, fn, timeout: float, every: float = 3.0, alive=None):
    """Poll ``fn`` until truthy; ``alive`` (optional) stops the wait early when
    the thing being waited on is gone."""
    deadline = time.monotonic() + timeout
    while True:
        value = fn()
        if value:
            return value
        if alive is not None and not alive():
            raise E2EError(f"the simulator exited while waiting for {what}")
        if time.monotonic() > deadline:
            raise E2EError(f"timed out after {timeout:.0f} s waiting for {what}")
        time.sleep(every)


# -- the simulator --------------------------------------------------------------------------------


def docker_arch() -> str:
    return "arm64" if platform.machine().lower() in ("arm64", "aarch64") else "amd64"


@contextlib.contextmanager
def simulator(image: str, port: int, *, keep: bool, docker: str = "docker"):
    name = f"urcap-e2e-{uuid.uuid4().hex[:8]}"
    cmd = [
        docker, "run", "-d", "--name", name, "--privileged",
        "-e", f"HOST_ARCH={docker_arch()}", "-e", "ROBOT_TYPE=UR10",
        "-p", f"127.0.0.1:{port}:80",
        "--add-host", "host.docker.internal:host-gateway",
        image,
    ]  # fmt: skip
    print(f"starting {image} as {name} on 127.0.0.1:{port}", flush=True)
    run = subprocess.run(cmd, capture_output=True, text=True)
    if run.returncode != 0:
        raise E2EError(f"docker run failed: {run.stderr.strip()[-800:]}")
    sim = {
        "name": name,
        "logs": "",
        "alive": lambda: running(docker, name),
        # the log line, or the bootstrapper's own status (on a GitHub runner, 2026-09-30, the
        # status said finished while the line never reached `docker logs`)
        "ready": lambda: bootstrapped(docker_logs(name, docker, tail=100_000)) or bootstrap_finished(port),
    }
    try:
        yield sim
    except BaseException:
        sim["logs"] = docker_logs(name, docker)  # before the container goes
        raise
    finally:
        if keep:
            print(f"keeping {name} (docker rm -f {name})", flush=True)
        else:
            subprocess.run(teardown_command(docker, name), capture_output=True)


def teardown_command(docker: str, name: str) -> list[str]:
    # -v: the sim's inner /var/lib/docker is an anonymous volume of ~9 GB per run
    return [docker, "rm", "-f", "-v", name]


def running(docker: str, name: str) -> bool:
    out = subprocess.run(
        [docker, "inspect", "-f", "{{.State.Running}}", name], capture_output=True, text=True
    )
    return out.stdout.strip() == "true"


# The simulator's own "ready": its web-bootstrapper installs UR's URCaps after the web UI is
# already answering, then logs this line and the "Simulator Is Ready" banner. Installing ours
# before it raced that last pass: in every failing CI run PolyScope X's splash then said "An
# error occurred while starting the application" (2026-09-29: installs at 04:00:06 / 04:50:10,
# bootstrapper done at 04:00:53 / 04:50:48; runs 36519457991, 36523255181).
BOOTSTRAP_DONE = "Done, time to sleep forever"
BOOTSTRAP_STATUS = "/universal-robots/bootstrapper/status"


def bootstrapped(log_text: str) -> bool:
    """Has the simulator's web-bootstrapper finished (its log says so)?"""
    return BOOTSTRAP_DONE in log_text


def bootstrap_finished(port: int) -> bool:
    """… or its status endpoint says so (``{"inProgress": false, "isFinished": true, …}``)."""
    try:
        status, body = http(f"http://127.0.0.1:{port}{BOOTSTRAP_STATUS}", timeout=5)
        if status != 200:
            return False
        d = json.loads(body)
        return bool(d.get("isFinished")) and not d.get("inProgress") and not d.get("errorExitMessage")
    except (OSError, ValueError):
        return False


def docker_logs(name: str, docker: str = "docker", tail: int = 150) -> str:
    out = subprocess.run([docker, "logs", "--tail", str(tail), name], capture_output=True, text=True)
    return (out.stdout + out.stderr)[-12000:]


# -- the synthetic cockpit ------------------------------------------------------------------------


def cockpit_command(port: int, origin: str) -> list[str]:
    """A synthetic cockpit with **no robot link**: the e2e clicks in the node, and a
    click on a linked cockpit sends a Primary script (locate) to whatever robot it
    points at."""
    return [
        sys.executable, "-m", "perceptronics", "gui", "--fake", "--no-robot", "--no-browser",
        "--port", str(port), "--cors", origin,
    ]  # fmt: skip


def cockpit_env(environ: dict[str, str]) -> dict[str, str]:
    """The caller's environment without the robot / cell / camera settings a
    developer's shell may export (UR_CELL=ur3 names a real arm)."""
    return {k: v for k, v in environ.items() if not k.startswith(("UR_", "PERCEPTRONICS_"))}


@contextlib.contextmanager
def fake_cockpit(port: int, origin: str):
    cmd = cockpit_command(port, origin)
    proc = subprocess.Popen(
        cmd,
        cwd=REPO,
        env=cockpit_env(dict(os.environ)),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        wait_for(
            "the synthetic cockpit", lambda: http(f"http://127.0.0.1:{port}/api/info")[0] == 200, 60, 0.5
        )
        yield f"http://localhost:{port}"
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=10)
        except subprocess.TimeoutExpired:
            proc.kill()


# -- the checks -----------------------------------------------------------------------------------


def install_checks(
    checks: Checks, package: Path, port: int, boot_timeout: float, alive=None, ready=None
) -> None:
    base = f"http://127.0.0.1:{port}"
    t0 = time.monotonic()
    wait_for(
        "the urservice URCap endpoint",
        lambda: http(f"{base}{urcapx.API_PATH}")[0] == 200,
        boot_timeout,
        5,
        alive,
    )
    # PolyScope's own web app is a URCap too; the page 404s until it is in.
    wait_for("the PolyScope X web UI", lambda: http(f"{base}/")[0] == 200, boot_timeout, 5, alive)
    checks.ok("simulator up", f"{time.monotonic() - t0:.0f} s to the web UI")
    if ready is not None:
        # the web UI answers before the simulator has finished installing its own URCaps
        wait_for("the simulator's bootstrapper to finish", ready, boot_timeout, 3, alive)
        status, body = http(f"{base}{BOOTSTRAP_STATUS}")
        detail = body[:120].decode("utf-8", "replace") if status == 200 else f"HTTP {status}"
        checks.ok("simulator ready", f"{time.monotonic() - t0:.0f} s; bootstrapper status {detail}")
    res = urcapx.install(package, "127.0.0.1", port, replace=True)
    checks.expect(res["ok"], "install accepted", f"HTTP {res['status']} {res.get('hint', '')}".strip())
    listed = [
        it for it in urcapx.list_urcaps("127.0.0.1", port) if (it.get("id") or {}).get("urcapID") == URCAP_ID
    ]
    meta = urcapx.manifest_from_urcapx(package)
    checks.expect(
        bool(listed) and listed[0].get("version") == meta["version"],
        "listed",
        f"{VENDOR}/{URCAP_ID} {listed[0].get('version') if listed else '—'}",
    )
    # nginx picks the web archive up a few seconds after the 201; then it must
    # serve exactly the bytes that were packaged.
    with tarfile.open(package, "r:gz") as tar:
        packed = {
            m.name.removeprefix(f"{ARCHIVE}/"): tar.extractfile(m).read()
            for m in tar.getmembers()
            if m.isfile() and m.name.startswith(f"{ARCHIVE}/")
        }
    contribution = json.loads(packed["contribution.json"])
    node = contribution["applicationNodes"][0]
    files = [
        "contribution.json",
        node["presenterURI"],
        node["behaviorURI"],
        f"{node['translationPath']}en.json",
        "pickscript.js",
    ]
    for prog in contribution["programNodes"]:
        files += [prog["presenterURI"], prog["behaviorURI"], prog["iconURI"]]
    files = list(dict.fromkeys(files))
    url = f"{base}/{VENDOR}/{URCAP_ID}/{ARCHIVE}/"
    wait_for("the web archive to be served", lambda: http(url + files[0])[0] == 200, 120, 2)
    for rel in files:
        status, body = http(url + rel)
        checks.expect(
            status == 200 and body == packed.get(rel), f"serves {rel}", f"HTTP {status}, as packaged"
        )


RUNTIME_PROBE = """async (tag) => {
  const el = document.querySelector(tag);
  const api = el && el.applicationAPI;
  if (!api) return { error: "no applicationAPI on the element" };
  const rps = api.robotPositionService, rms = api.robotMoveService;
  const withTimeout = (p, ms, why) =>
    Promise.race([p, new Promise((_, rej) => setTimeout(() => rej(new Error(why)), ms))]);
  const first = (obs) => new Promise((res, rej) => {
    let sub = null, done = false;
    sub = obs.subscribe({ next: (v) => { if (done) return; done = true; res(v);
      setTimeout(() => sub && sub.unsubscribe && sub.unsubscribe(), 0); }, error: rej });
  });
  const out = {
    services: {
      updateNode: typeof (api.applicationNodeService || {}).updateNode,
      getKinematicInfo: typeof (rps || {}).getKinematicInfo,
      getJointPositions: typeof (rps || {}).getJointPositions,
      convertJointPositionsToTcpPose: typeof (rps || {}).convertJointPositionsToTcpPose,
      getInverseKinematics: typeof (rps || {}).getInverseKinematics,
      autoMove: typeof (rms || {}).autoMove,
    },
  };
  try {
    const dh = await withTimeout(rps.getKinematicInfo(), 10000, "getKinematicInfo timed out");
    out.dh = Array.isArray(dh) ? dh.length : null;
    out.dhKeys = Array.isArray(dh) && dh[0] ? Object.keys(dh[0]).sort() : [];
    const q = await withTimeout(first(rps.getJointPositions()), 10000, "getJointPositions timed out");
    out.q = q;
    if (typeof rps.convertJointPositionsToTcpPose === "function") {
      const pose = await withTimeout(rps.convertJointPositionsToTcpPose(q), 10000, "FK timed out");
      out.pose = pose;
      const back = await withTimeout(rps.getInverseKinematics(pose, q), 10000, "IK timed out");
      out.ik = back;
    }
  } catch (e) {
    out.error = String(e && e.message || e);
  }
  return out;
}"""

JOINTS = ("base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3")
OPTIONAL_SERVICES = ("convertJointPositionsToTcpPose",)
PICK_TAG = f"{TAG}-pick"
DIALOG_TAG = f"{TAG}-pick-dialog"


def open_toolbox(page) -> None:
    """The + under Main Program opens the toolbox: the tree's small call-to-action icon
    button on 10.13 / 10.14; older releases style it differently, so fall back to the first
    button drawn where that + sits (the tree column, just under the program's first row)."""
    cta = page.locator("button.ur-icon-button-small.ur-icon-button-cta")
    if cta.count():
        cta.first.click(timeout=30_000)
        return
    buttons = page.locator("button")
    for i in range(buttons.count()):
        box = buttons.nth(i).bounding_box()
        if box and 100 < box["x"] < 300 and 90 < box["y"] < 200 and box["width"] < 60:
            buttons.nth(i).click(timeout=30_000)
            return
    raise E2EError("no toolbox + button found under Main Program")


def program_node_checks(checks: Checks, page, shot) -> None:
    """The Perceptronic Pick program node: from the toolbox into the tree, its row, the
    dialog it opens (the feed live, a picture point taught from PolyScope's joint
    positions), and the row's verdict once the node can generate a program. The
    application node's cockpit URL was set by the checks before this."""
    page.get_by_text("Program", exact=True).first.click(timeout=30_000)
    page.wait_for_timeout(2000)
    open_toolbox(page)
    page.get_by_text("Perceptronic Pick", exact=True).first.click(timeout=30_000)
    page.wait_for_selector(PICK_TAG, state="attached", timeout=60_000)
    row = page.locator(PICK_TAG)
    page.wait_for_function(
        f"() => /picture/.test((document.querySelector('{PICK_TAG} [data-pk=txt]') || {{}}).textContent"
        " || '')",
        timeout=30_000,
    )
    checks.ok("pick node in the tree", row.locator('[data-pk="txt"]').inner_text()[:120])
    verdict = row.locator('[data-pk="verdict"]').inner_text()
    checks.expect("picture point" in verdict, "pick node invalid without a picture point", verdict[:160])
    row.locator('[data-pk="open"]').click(force=True)
    page.wait_for_selector(DIALOG_TAG, state="attached", timeout=30_000)
    dlg = page.locator(DIALOG_TAG)
    try:
        page.wait_for_function(f"() => !!document.querySelector('{DIALOG_TAG} .dot.live')", timeout=45_000)
        checks.ok("pick dialog feed live", dlg.locator('[data-pk="cap-bottom"]').inner_text()[:120])
    except Exception:
        checks.fail("pick dialog feed live", dlg.locator('[data-pk="cap-top"]').inner_text()[:200])
    shot("pick-dialog")
    dlg.locator('[data-pk="add"]').click()
    page.wait_for_function(
        # the verdict before the click says "add a picture point: …" — wait for the teach, or its error
        f"() => /taught at joints|^picture point:/.test("
        f"document.querySelector('{DIALOG_TAG} [data-pk=status]').textContent)",
        timeout=30_000,
    )
    status = dlg.locator('[data-pk="status"]').inner_text()
    checks.expect("taught at joints" in status, "picture point from PolyScope's joints", status[:160])
    page.get_by_role("button", name="Done").first.click(timeout=10_000)
    page.wait_for_selector(DIALOG_TAG, state="detached", timeout=30_000)
    # the variables are declared asynchronously (and re-attached if PolyScope's insertion
    # overwrote them — 10.10 and older): give the row a moment
    with contextlib.suppress(Exception):
        page.wait_for_function(
            f"() => {{ const p = document.querySelector('{PICK_TAG}').contributedNode.parameters;"
            " return !!(p.foundVariable && p.locVariable); }",
            timeout=15_000,
        )
    params = page.evaluate(f"() => document.querySelector('{PICK_TAG}').contributedNode.parameters")
    points = params.get("points") or []
    checks.expect(
        len(points) == 1 and bool(params.get("foundVariable")) and bool(params.get("locVariable")),
        "node saved: the picture point and the result variables",
        f"{len(points)} point(s), variables {bool(params.get('foundVariable'))}/"
        f"{bool(params.get('locVariable'))}",
    )
    page.wait_for_function(
        f"() => /^ready/.test(document.querySelector('{PICK_TAG} [data-pk=verdict]').textContent)",
        timeout=30_000,
    )
    checks.ok("pick node ready", row.locator('[data-pk="verdict"]').inner_text()[:160])
    shot("pick-row")


# PolyScope dialogs the simulator raises when its web UI comes up before the
# controller does — a boot race, not the URCap (seen on a CI runner 2026-09-27:
# "No kinematics info available" over the Operator screen, before the node was
# opened). These are acknowledged and the page reloaded; any other error dialog
# fails the run with its text.
BOOT_RACE_DIALOGS = (
    "No kinematics info available",
    # 10.6 / 10.7 on a GitHub runner, 2026-09-30: "An error occurred while starting the
    # application … RETRY Internal software communication error" over the first page, gone on reload
    "Internal software communication error",
)


def clear_boot_dialogs(page) -> list[str]:
    """Acknowledge known boot-race dialogs; raise on any other error dialog."""
    seen = []
    for _ in range(10):
        if not page.get_by_text("An error occurred").first.is_visible():
            return seen
        known = next((t for t in BOOT_RACE_DIALOGS if page.get_by_text(t).first.is_visible()), None)
        if known is None:
            body = page.locator("body").inner_text()
            at = body.find("An error occurred")
            raise E2EError(f"PolyScope error dialog: {body[at : at + 200]!r}")
        seen.append(known)
        # its button is "OK" on 10.13; the start-up error on 10.6 / 10.7 offers a retry — take
        # whatever is there, else reload the page (the dialog is gone on the next load)
        button = page.get_by_role("button", name=re.compile(r"^(ok|retry|try again|close)$", re.I)).first
        if button.is_visible():
            button.click(timeout=10_000)
        else:
            page.reload(wait_until="networkidle", timeout=180_000)
        # a slow runner's 10.6 / 10.7 raise it ("no elements in sequence") until their services
        # settle — well over a minute after the bootstrapper said done (2026-09-30)
        page.wait_for_timeout(8000)
    raise E2EError(f"PolyScope keeps raising {seen[-1]!r}")


def browser_checks(checks: Checks, port: int, cockpit_port: int, shots: Path | None) -> None:
    try:
        from playwright.sync_api import sync_playwright
    except ImportError:
        checks.fail("playwright", "not importable — run under `uv run --with playwright==1.63.0`")
        return
    origin = f"http://localhost:{port}"
    with fake_cockpit(cockpit_port, origin) as cockpit, sync_playwright() as p:
        browser = p.chromium.launch()
        page = browser.new_page(viewport={"width": 1280, "height": 800})
        ours = f"/{VENDOR}/{URCAP_ID}/"
        errors: list[str] = []
        page.on("pageerror", lambda e: errors.append(f"{e}\n{getattr(e, 'stack', '')}"))
        segment: list[dict] = []
        boot_dialogs: list[str] = []

        def on_response(r):
            if r.url.startswith(cockpit) and "/api/segment" in r.url:
                with contextlib.suppress(Exception):
                    segment.append(r.json())

        page.on("response", on_response)

        def open_node() -> None:
            nav = page.get_by_text("Application", exact=True).first
            for _attempt in range(4):  # the UI's first load after boot can come up partial
                page.goto(origin + "/", wait_until="networkidle", timeout=180_000)
                with contextlib.suppress(Exception):
                    nav.wait_for(state="visible", timeout=60_000)
                page.wait_for_timeout(2000)  # a boot-race dialog lands just after the page
                if clear_boot_dialogs(page):
                    boot_dialogs.extend(BOOT_RACE_DIALOGS)
                    page.wait_for_timeout(5000)
                    continue  # reload once the controller has caught up
                if nav.is_visible():
                    break
            nav.click(timeout=30_000)
            page.get_by_text("Perceptronic").first.click(timeout=60_000)
            page.wait_for_selector(TAG, state="attached", timeout=60_000)

        def shot(name: str) -> None:
            if shots:
                page.screenshot(path=str(shots / f"{name}.png"))

        try:
            open_node()
            node = page.locator(TAG)
            checks.expect(
                "Perceptronic" in node.inner_text(timeout=30_000), "node renders", "i18n title shown"
            )
            url_box = node.locator('[data-rsp="url"]')
            url_box.fill(cockpit)
            node.locator('[data-rsp="save"]').click()
            status = node.locator('[data-rsp="status"]')
            try:
                page.wait_for_function(
                    f"() => /live from/.test(document.querySelector('{TAG} [data-rsp=status]').textContent)",
                    timeout=45_000,
                )
                checks.ok("feed live", status.inner_text())
            except Exception:
                checks.fail("feed live", status.inner_text())
            img = node.locator('[data-rsp="img"]')
            box = img.bounding_box()
            if not box:
                checks.fail("feed image", "no bounding box")
            cx, cy = box["x"] + box["width"] / 2, box["y"] + box["height"] / 2
            page.mouse.move(cx, cy)
            page.wait_for_function(
                f"() => /m away|no depth/.test(document.querySelector('{TAG} [data-rsp=hover]').textContent)",
                timeout=15_000,
            )
            checks.ok("hover reads depth", node.locator('[data-rsp="hover"]').inner_text())
            page.mouse.click(cx, cy)
            page.wait_for_function(
                f"() => !/segmenting/.test(document.querySelector('{TAG} [data-rsp=status]').textContent)",
                timeout=30_000,
            )
            checks.expect(
                bool(segment) and segment[-1].get("ok") is True,
                "click segments",
                f"then: {status.inner_text()[:160]}",  # locate fails on a robot-less cockpit
            )
            shot("node-live")

            probe = page.evaluate(RUNTIME_PROBE, TAG)
            services = probe.get("services", {})
            # convertJointPositionsToTcpPose is 10.10+: without it the node's Move (PolyScope)
            # takes the flange target as the TCP and says so (10.6–10.9 in the release matrix)
            missing = [k for k, v in services.items() if v != "function" and k not in OPTIONAL_SERVICES]
            optional_missing = [k for k in OPTIONAL_SERVICES if services.get(k) != "function"]
            checks.expect(
                not missing,
                "presenter services present",
                ", ".join(missing)
                or (
                    "all functions"
                    if not optional_missing
                    else f"all but {', '.join(optional_missing)} (optional)"
                ),
            )
            checks.expect(
                probe.get("dh") == 6 and {"DHa", "DHd", "DHAlpha"} <= set(probe.get("dhKeys", [])),
                "getKinematicInfo",
                probe.get("error") or f"{probe.get('dh')} rows {probe.get('dhKeys')}",
            )
            q, back, pose = probe.get("q") or {}, probe.get("ik") or {}, probe.get("pose") or {}
            checks.expect(
                all(isinstance(q.get(j), (int, float)) for j in JOINTS),
                "joint positions",
                probe.get("error") or f"{q}",
            )
            if "convertJointPositionsToTcpPose" in optional_missing:
                checks.ok(
                    "FK + IK round trip", "skipped: no convertJointPositionsToTcpPose on this PolyScope"
                )
            else:
                checks.expect(
                    len(pose.get("position", [])) == 3 and len(pose.get("orientation", [])) == 3,
                    "FK",
                    probe.get("error") or f"pose {pose}",
                )
                err = max((abs(back.get(j, 99) - q[j]) for j in JOINTS), default=99) if back else 99
                checks.expect(
                    err < 1e-3, "IK round trip", probe.get("error") or f"max joint error {err:.2e} rad"
                )

            open_node()
            # the field fills when PolyScope hands the node its saved state, a moment after it renders
            with contextlib.suppress(Exception):
                page.wait_for_function(
                    f"() => !!document.querySelector('{TAG} [data-rsp=url]').value", timeout=30_000
                )
            saved = page.locator(TAG).locator('[data-rsp="url"]').input_value(timeout=30_000)
            checks.expect(saved == cockpit, "cockpit URL persists", saved)
            shot("node-reloaded")
            program_node_checks(checks, page, shot)
            if boot_dialogs:
                checks.ok("simulator boot race", f"acknowledged {len(boot_dialogs)} × {boot_dialogs[0]!r}")
            ours_errors = [e for e in errors if ours in e]
            checks.expect(not ours_errors, "no page errors from the URCap", "; ".join(ours_errors)[:400])
        finally:
            with contextlib.suppress(Exception):
                shot("last")
            browser.close()


def run(args) -> int:
    checks = Checks()
    image = args.image
    if not image:
        sim = track.load_target()["simulator"]
        image = f"{sim['image']}@{sim['digest']}"
    port = args.port or free_port()
    shots = Path(args.artifacts) if args.artifacts else None
    if shots:
        shots.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as tmp:
        package = Path(args.package) if args.package else urcapx.package(HERE / URCAP_ID, tmp)
        sim = None
        try:
            with simulator(image, port, keep=args.keep, docker=args.docker) as sim:
                install_checks(checks, package, port, args.boot_timeout, sim["alive"], sim["ready"])
                if not args.no_browser:
                    browser_checks(checks, port, args.cockpit_port or free_port(), shots)
                if not args.keep:
                    urcapx.delete("127.0.0.1", port, VENDOR, URCAP_ID)
        except Exception as exc:  # playwright timeouts too: every failure is a reported check
            if not checks.items or checks.items[-1]["ok"]:
                checks.items.append({"check": "run", "ok": False, "detail": str(exc)})
            print(f"e2e: {exc}", file=sys.stderr)
            if sim and sim["logs"]:
                logs = sim["logs"]
                print("---- simulator log (tail) ----\n" + logs, file=sys.stderr)
                if shots:
                    (shots / "simulator.log").write_text(logs, encoding="utf-8")
    summary = {"image": image, "passed": checks.passed, "checks": checks.items}
    if args.report:
        Path(args.report).write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print("e2e: " + ("passed" if checks.passed else "FAILED") + f" against {image}")
    return 0 if checks.passed else 1


def main(argv: list[str] | None = None) -> int:
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")
    ap = argparse.ArgumentParser(
        prog="e2e", description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--image", help="simulator image (default: target.json's, by digest)")
    ap.add_argument("--package", help="a built .urcapx (default: build urcap/perceptronic now)")
    ap.add_argument(
        "--port", type=int, default=0, help="host port for the sim's web UI (default: a free one)"
    )
    ap.add_argument(
        "--cockpit-port", type=int, default=0, help="port for the synthetic cockpit (default: free)"
    )
    ap.add_argument(
        "--boot-timeout", type=float, default=900, help="seconds to wait for the sim (default 900)"
    )
    ap.add_argument("--no-browser", action="store_true", help="stop after install + serve checks")
    ap.add_argument("--keep", action="store_true", help="leave the sim (and the URCap) running")
    ap.add_argument("--artifacts", help="directory for screenshots + the sim log on failure")
    ap.add_argument("--report", help="write the check list here as JSON")
    ap.add_argument("--docker", default=os.environ.get("DOCKER", "docker"), help=argparse.SUPPRESS)
    args = ap.parse_args(argv)
    if not shutil.which(args.docker.split()[0]):
        print(f"e2e: {args.docker} not found", file=sys.stderr)
        return 2
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
