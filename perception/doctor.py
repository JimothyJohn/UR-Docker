"""``perception doctor`` — the pilot's pre-flight checklist for a cell.

One command answers "why doesn't it work?" for the four things that break
independently: the SDK, the camera, the network path to the controller, and
the controller's own state. Every check carries a **fix** in words (the
troubleshooting tables in ``docs/realsense.md`` and ``CLAUDE.md``, encoded),
so the report is actionable on its own — on a Windows laptop at the cell, in
the cockpit's Health panel, or read by an agent through the ``cell_doctor``
tool.

    perception --cell ur20 doctor            # SDK, camera, robot reachability + state
    perception --cell ur20 doctor --stream   # also open the camera and check frames
    perception doctor --no-robot             # camera side only
    perception doctor --json                 # machine-readable

A check is ``ok=True`` (pass), ``ok=False`` (fail, with a fix) or ``ok=None``
(skipped/informational). ``severity`` says whether a failure blocks the demo
(``critical``: nothing works), motion (``motion``: state reads work, moves
won't) or is a warning (``warn``: works, but you should know).
"""

from __future__ import annotations

import os
import platform
import socket
import sys
import time
from collections.abc import Callable
from dataclasses import asdict, dataclass, field

from urctl.config import RobotConfig

from .cell import ENV_CELL, describe_cell
from .config import PerceptionConfig
from .handeye import ENV_BRACKET, ENV_T_FLANGE_CAMERA, HandEye

PROBE_TIMEOUT_S = 2.0
STREAM_FRAMES = 15


@dataclass
class Check:
    name: str
    ok: bool | None
    detail: str
    fix: str = ""
    severity: str = "critical"  # critical | motion | warn | info
    data: dict = field(default_factory=dict)


@dataclass
class Report:
    checks: list[Check] = field(default_factory=list)
    started_at: float = field(default_factory=time.time)

    def add(self, check: Check) -> Check:
        self.checks.append(check)
        return check

    @property
    def ok(self) -> bool:
        return not any(c.ok is False and c.severity == "critical" for c in self.checks)

    @property
    def motion_ok(self) -> bool:
        return self.ok and not any(c.ok is False and c.severity == "motion" for c in self.checks)

    def as_dict(self) -> dict:
        failed = [c.name for c in self.checks if c.ok is False]
        return {
            "ok": self.ok,
            "motion_ok": self.motion_ok,
            "failed": failed,
            "elapsed_s": round(time.time() - self.started_at, 2),
            "checks": [asdict(c) for c in self.checks],
        }

    def render(self) -> str:
        """Human-readable, one line per check, fixes indented under failures."""
        lines = []
        for c in self.checks:
            mark = {True: "ok  ", False: "FAIL", None: "--  "}[c.ok]
            lines.append(f"[{mark}] {c.name}: {c.detail}")
            if c.ok is False and c.fix:
                lines.append(f"       fix: {c.fix}")
        verdict = "READY" if self.motion_ok else ("STATE ONLY (motion gated)" if self.ok else "NOT READY")
        lines.append(f"verdict: {verdict}")
        return "\n".join(lines)


# ----- individual checks --------------------------------------------------------


def _tcp_probe(host: str, port: int, timeout: float = PROBE_TIMEOUT_S) -> tuple[bool, float, str]:
    t0 = time.monotonic()
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True, (time.monotonic() - t0) * 1000.0, ""
    except OSError as exc:
        return False, (time.monotonic() - t0) * 1000.0, str(exc)


def check_host(report: Report, env=None) -> None:
    env = os.environ if env is None else env
    cell = describe_cell(env)
    report.add(
        Check(
            "host",
            None,
            f"{platform.system()} {platform.release()} {platform.machine()}, "
            f"python {platform.python_version()}",
            severity="info",
            data={"platform": sys.platform, "python": platform.python_version()},
        )
    )
    name = env.get(ENV_CELL)
    report.add(
        Check(
            "cell",
            None if name else False,
            f"{name} → {cell}" if name else "no cell selected (env vars only)",
            fix="pass --cell sim|ur3|ur20 (or UR_CELL=…) so host/ports/bracket come from one profile",
            severity="warn",
            data=cell,
        )
    )


def check_sdk(report: Report, library: str | None = None) -> bool:
    from .realsense import RealSenseError, load_api

    try:
        api = load_api(library)
    except RealSenseError as exc:
        report.add(
            Check(
                "sdk",
                False,
                str(exc).splitlines()[0],
                fix=_sdk_fix(),
            )
        )
        return False
    report.add(
        Check(
            "sdk",
            True,
            f"librealsense {sdk_version_text(api.version)} at {api.path}",
            data={"path": api.path, "api_version": api.version},
        )
    )
    return True


def sdk_version_text(version) -> str:
    """``RS2_API_VERSION`` packs major.minor.patch as ``major*10000 + minor*100 + patch``
    (25804 → 2.58.4); anything else is shown as-is."""
    try:
        v = int(version)
    except (TypeError, ValueError):
        return str(version)
    if v < 10000:
        return str(version)
    return f"{v // 10000}.{(v // 100) % 100}.{v % 100}"


def _sdk_fix() -> str:
    if sys.platform == "win32":
        return (
            "install Intel RealSense SDK 2.0 (RealSense.SDK-WIN10-<ver>.exe from the librealsense GitHub "
            "releases; scripts/setup-windows.ps1 does it), then set REALSENSE_LIB to "
            r"C:\Program Files (x86)\Intel RealSense SDK 2.0\bin\x64\realsense2.dll if it is not found"
        )
    if sys.platform == "darwin":
        return "brew install librealsense (and run the camera commands under sudo)"
    return "apt install librealsense2 (or build with -DFORCE_RSUSB_BACKEND=ON) and install the udev rules"


def check_devices(report: Report, library: str | None = None) -> list[dict]:
    from .realsense import RealSenseError, list_devices, platform_hint

    try:
        devices = list_devices(library)
    except RealSenseError as exc:
        report.add(Check("camera", False, str(exc), fix=platform_hint(exc) or "check the USB cable"))
        return []
    if not devices:
        report.add(
            Check(
                "camera",
                False,
                "no RealSense device enumerated",
                fix="plug the D435 into a USB 3 port with a short cable (≤ 2 m, no hub); on macOS run under "
                "sudo; on Linux install the udev rules; on Windows check Device Manager → Cameras",
            )
        )
        return []
    for d in devices:
        usb = str(d.get("usb_type") or "?")
        fw = d.get("firmware") or "?"
        slow = usb.startswith("2")
        report.add(
            Check(
                "camera",
                not slow,
                f"{d.get('name')} sn {d.get('serial')} fw {fw} usb {usb}",
                fix="USB 2 link: move to a direct USB 3 port (blue) or the stream caps at 15 fps",
                severity="warn",
                data=dict(d),
            )
        )
    return devices


def check_stream(report: Report, camera, frames: int = STREAM_FRAMES) -> None:
    """Open ``camera`` (any :class:`RgbdCamera`), read ``frames`` and judge them:
    colour not black, depth mostly valid, fps sane. Closes it afterwards."""
    from .realsense import RealSenseError, platform_hint

    t0 = time.monotonic()
    try:
        camera.open()
        frame = None
        for _ in range(max(1, frames)):
            frame = camera.read()
        assert frame is not None
    except Exception as exc:
        hint = platform_hint(exc) if isinstance(exc, RealSenseError) else ""
        report.add(
            Check("stream", False, f"{type(exc).__name__}: {exc}", fix=hint or "see docs/realsense.md")
        )
        return
    finally:
        try:
            camera.close()
        except Exception:
            pass
    elapsed = time.monotonic() - t0
    stats = frame.depth.stats()
    px = frame.color.data
    step = max(1, len(px) // 3000)
    sample = px[::step]
    mean_rgb = sum(sample) / max(1, len(sample))
    valid = stats.get("valid_fraction")
    black = mean_rgb < 3.0
    sparse = valid is not None and valid < 0.2
    detail = (
        f"{frames} frames in {elapsed:.1f}s ({frames / elapsed:.1f} fps), colour mean {mean_rgb:.0f}/255, "
        f"depth valid {valid:.0%}"
        if valid is not None
        else f"{frames} frames in {elapsed:.1f}s"
    )
    fix = ""
    if black:
        fix = "black colour: depth and colour must stream at the same size (848x480 both); drop --width"
    elif sparse:
        fix = "little valid depth: nothing under ~28 cm, no direct sun, matte surfaces; check the lens cover"
    report.add(
        Check(
            "stream",
            not (black or sparse),
            detail,
            fix=fix,
            severity="critical" if black else "warn",
            data={"fps": frames / elapsed, "mean_rgb": mean_rgb, "depth": stats, "frame": frame.summary()},
        )
    )


def check_robot_reachability(report: Report, config: RobotConfig) -> bool:
    if not config.host:
        report.add(
            Check(
                "robot.host",
                False,
                "UR_HOST is empty",
                fix="set the controller IP in the cell file (perception/cells/<cell>.env) or UR_HOST",
            )
        )
        return False
    orchestration = (
        ("robot-api", config.robot_api_port)
        if config.is_polyscopex()
        else ("dashboard", config.dashboard_port)
    )
    other = (
        ("dashboard", config.dashboard_port)
        if config.is_polyscopex()
        else ("robot-api", config.robot_api_port)
    )
    ok_orch, ms, err = _tcp_probe(config.host, orchestration[1])
    ok_other, _, _ = _tcp_probe(config.host, other[1]) if not ok_orch else (False, 0.0, "")
    if ok_orch:
        report.add(
            Check(
                "robot.reach",
                True,
                f"{orchestration[0]} {config.host}:{orchestration[1]} answered in {ms:.0f} ms",
            )
        )
    elif ok_other:
        wanted = "e-series" if config.is_polyscopex() else "polyscopex"
        report.add(
            Check(
                "robot.reach",
                False,
                f"{orchestration[0]} port {orchestration[1]} closed but {other[0]} port {other[1]} is open",
                fix=f"platform mismatch: this controller looks like {wanted}; set UR_PLATFORM={wanted} "
                f"(cell file or env)",
            )
        )
        return False
    else:
        report.add(
            Check(
                "robot.reach",
                False,
                f"{config.host}:{orchestration[1]} unreachable ({err})",
                fix="same subnet as the controller? ping it; PolyScope X: the Robot-API is HTTP on port 80 "
                "of the robot; the sim publishes it on 8000 (UR_ROBOT_API_PORT)",
            )
        )
        return False
    ok_p, ms_p, err_p = _tcp_probe(config.host, config.primary_port)
    report.add(
        Check(
            "robot.primary",
            ok_p,
            f"primary {config.host}:{config.primary_port} {'open' if ok_p else 'closed'} ({ms_p:.0f} ms)",
            fix="PolyScope X: enable Primary Client Interface under Settings → Security → Services (admin "
            "password); URSim-PX publishes it on 31001 (UR_PRIMARY_PORT)",
            severity="motion",
        )
    )
    ok_r, ms_r, _ = _tcp_probe(config.host, config.rtde_port)
    report.add(
        Check(
            "robot.rtde",
            ok_r,
            f"rtde {config.host}:{config.rtde_port} {'open' if ok_r else 'closed'} ({ms_r:.0f} ms)",
            fix="enable RTDE in Settings → Security → Services; without it state reads fall back to the "
            "slower Primary path",
            severity="warn",
        )
    )
    return True


def check_robot_state(report: Report, robot) -> dict | None:
    try:
        state = robot.get_state()
    except Exception as exc:
        report.add(Check("robot.state", False, f"{type(exc).__name__}: {exc}", fix="see robot.reach"))
        return None
    rm, sm, cm, ps = (state.get(k) for k in ("robot_mode", "safety_mode", "control_mode", "program_state"))
    report.add(
        Check(
            "robot.mode",
            "RUNNING" in str(rm),
            f"robot_mode {rm}, program {ps}",
            fix="bring the robot up: cockpit → Bring up, or `urctl bring-up` (needs Remote on PolyScope X)",
            severity="motion",
            data={"robot_mode": rm, "program_state": ps},
        )
    )
    report.add(
        Check(
            "robot.safety",
            "NORMAL" in str(sm) or "REDUCED" in str(sm),
            f"safety_mode {sm}",
            fix="PROTECTIVE_STOP: `unlock protective stop` / cockpit Bring up clears it; a movel from a "
            "singular pose (C154A0) is the usual cause — movej to an elbow-bent pose first",
            severity="motion",
            data={"safety_mode": sm},
        )
    )
    report.add(
        Check(
            "robot.control",
            str(cm) == "REMOTE",
            f"control_mode {cm}",
            fix="switch to Remote on the pendant (top-right on e-Series; Safety screen on PolyScope X). "
            "There is no network way to do it. State reads work in Local; motion and bring-up do not.",
            severity="motion",
            data={"control_mode": cm},
        )
    )
    if state.get("joints") is None:
        report.add(
            Check(
                "robot.telemetry",
                False,
                "no joint/TCP telemetry (RTDE off and robot not RUNNING)",
                fix="enable RTDE, or bring the robot up so the Primary broadcast carries joints",
                severity="warn",
            )
        )
    else:
        tcp = state.get("tcp") or []
        report.add(
            Check(
                "robot.telemetry",
                True,
                "joints + TCP live" + (f", TCP z {tcp[2]:.3f} m" if len(tcp) == 6 else ""),
                severity="info",
            )
        )
    return state


def check_flange_and_handeye(report: Report, robot, handeye: HandEye, env=None) -> None:
    env = os.environ if env is None else env
    bracket = env.get(ENV_BRACKET, "eseries")
    model = (env.get("UR_ROBOT_MODEL") or "").upper()
    expected = "ur20" if model.startswith(("UR20", "UR30")) else "eseries"
    report.add(
        Check(
            "handeye",
            handeye.calibrated or None,
            f"{handeye.source}"
            + ("" if handeye.calibrated else " (uncalibrated seed: expect mm + ~1° error)"),
            fix=f"run a hand-eye calibration and set {ENV_T_FLANGE_CAMERA}",
            severity="warn",
            data=handeye.as_dict(),
        )
    )
    if model and not handeye.calibrated and bracket != expected:
        report.add(
            Check(
                "handeye.bracket",
                False,
                f"bracket seed '{bracket}' but robot model {model} takes the '{expected}' print",
                fix=f"set {ENV_BRACKET}={expected} (the cell files do this)",
                severity="warn",
            )
        )
    try:
        fp = robot.get_flange_pose()
    except Exception as exc:
        report.add(Check("robot.tcp", False, f"{type(exc).__name__}: {exc}", severity="warn"))
        return
    if not fp.get("ok"):
        report.add(
            Check(
                "robot.tcp",
                False,
                str(fp.get("error") or "flange pose not available"),
                fix="needs the Primary interface (and Remote on PolyScope X) — see robot.primary",
                severity="warn",
            )
        )
        return
    off = fp.get("tcp_offset") or [0.0] * 6
    mag = sum(v * v for v in off[:3]) ** 0.5
    report.add(
        Check(
            "robot.tcp",
            True,
            f"active TCP offset {mag * 1000:.1f} mm; flange z {fp['flange'][2]:.3f} m",
            severity="info",
            data={"tcp_offset": off, "flange": fp.get("flange")},
        )
    )
    if mag > 0.001:
        report.add(
            Check(
                "robot.tcp.nodes",
                None,
                f"non-flange TCP ({mag * 1000:.0f} mm): author motion as script nodes, "
                "not MoveJ/Waypoint nodes",
                severity="info",
            )
        )


def check_cockpit(report: Report, url: str) -> None:
    import json
    import urllib.request

    try:
        with urllib.request.urlopen(url.rstrip("/") + "/api/info", timeout=PROBE_TIMEOUT_S) as r:
            info = json.loads(r.read())
    except Exception:
        report.add(
            Check("cockpit", None, f"no cockpit at {url} (start one: perception gui)", severity="info")
        )
        return
    cam = info.get("camera", {}) or {}
    report.add(
        Check(
            "cockpit",
            True,
            f"{url}: {cam.get('kind')} camera, {info.get('fps')} fps, seq {info.get('seq')}"
            + (f", last error: {info['last_error']}" if info.get("last_error") else ""),
            severity="info",
            data={"fps": info.get("fps"), "seq": info.get("seq"), "last_error": info.get("last_error")},
        )
    )


# ----- the whole thing ------------------------------------------------------------


def run_doctor(
    *,
    robot_config: RobotConfig | None = None,
    perception_config: PerceptionConfig | None = None,
    camera: bool = True,
    stream: bool = False,
    robot: bool = True,
    library: str | None = None,
    camera_factory: Callable[[], object] | None = None,
    robot_factory: Callable[[RobotConfig], object] | None = None,
    cockpit_url: str | None = None,
    env=None,
) -> Report:
    """Run every applicable check and return the :class:`Report`.

    ``camera_factory`` builds the camera for ``--stream`` (defaults to the real
    RealSense with the perception config; the synthetic camera when
    ``PERCEPTION_FAKE`` is set). ``robot_factory`` builds the controller client
    (defaults to :class:`urctl.Robot`) — both exist so tests can inject fakes.
    """
    env = os.environ if env is None else env
    report = Report()
    check_host(report, env)
    fake = str(env.get("PERCEPTION_FAKE", "")).strip().lower() in ("1", "true", "yes", "on")
    if camera and not fake:
        if check_sdk(report, library):
            devices = check_devices(report, library)
            if stream and devices:
                cam = camera_factory() if camera_factory else _default_camera(perception_config, library)
                check_stream(report, cam)
    elif camera and fake:
        report.add(
            Check(
                "camera", None, "PERCEPTION_FAKE set: synthetic RGB-D scene, no SDK needed", severity="info"
            )
        )
        if stream:
            cam = (
                camera_factory() if camera_factory else _default_camera(perception_config, library, fake=True)
            )
            check_stream(report, cam)
    if cockpit_url:
        check_cockpit(report, cockpit_url)
    if robot:
        cfg = robot_config or RobotConfig.from_env()
        cfg = RobotConfig(**{**cfg.__dict__, "timeout": min(cfg.timeout, 5.0)})
        if check_robot_reachability(report, cfg):
            if robot_factory is not None:
                rb = robot_factory(cfg)
            else:
                from urctl.robot import Robot

                rb = Robot(cfg)
            try:
                if check_robot_state(report, rb) is not None:
                    check_flange_and_handeye(report, rb, HandEye.from_env(env), env)
            finally:
                close = getattr(rb, "close", None)
                if close:
                    close()
    return report


def _default_camera(config: PerceptionConfig | None, library: str | None, fake: bool = False):
    from .realsense import open_camera

    cfg = config or PerceptionConfig.from_env()
    return open_camera(
        fake=fake,
        fps=cfg.rs_fps or None,
        serial=cfg.rs_serial or None,
        library=library,
        depth_width=cfg.rs_depth_width,
        depth_height=cfg.rs_depth_height,
    )
