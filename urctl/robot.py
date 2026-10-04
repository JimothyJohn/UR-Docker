"""Robot — the high-level facade humans and agents drive.

Combines the Dashboard (orchestration) and Primary (execution) clients behind
one object, runs every mutating action through the :class:`SafetyEnvelope`, and
records each one in the :class:`AuditLog`. This is the single class the CLI,
the tool registry, and the MCP server all sit on top of.

    from urctl import Robot, RobotConfig
    robot = Robot(RobotConfig(host="10.0.0.5"))
    robot.bring_up()
    robot.move_joints([0, -1.57, 0, -1.57, 0, 0])
    print(robot.get_state())

Set ``dry_run=True`` to validate, audit, and log every action *without* sending
anything to the controller — useful for previewing what an agent would do.
"""

from __future__ import annotations

import math
from dataclasses import replace

from .audit import AuditLog
from .config import RobotConfig
from .dashboard import DashboardClient
from .primary import PrimaryBusyError, PrimaryClient
from .robotapi import RobotAPIClient
from .safety import SafetyEnvelope, SafetyVerdict, normalize_model, reach_for_model

# A safe candle-ish home pose for a UR10 — straight up, wrists folded. Far from
# singularities, well within joint limits, no self-collision.
HOME_JOINTS = [0.0, -1.5708, 0.0, -1.5708, 0.0, 0.0]

DEFAULT_VELOCITY = 0.5  # rad/s
DEFAULT_ACCELERATION = 0.8  # rad/s^2
# Each joint must land within this many radians (~3 deg) of target to count.
LANDING_TOLERANCE = 0.05

# Cartesian (movel) move defaults — deliberately gentle.
DEFAULT_TCP_VELOCITY = 0.25  # m/s
DEFAULT_TCP_ACCELERATION = 1.2  # m/s^2
# For an absolute move, the TCP must land within this many metres (~2 mm) of
# the commanded XYZ to count as arrived.
TCP_LANDING_TOLERANCE = 0.002
# How long a cockpit/CLI "freedrive on" keeps the arm hand-guidable before the
# hold program releases it on its own (see :meth:`Robot.freedrive`).
DEFAULT_FREEDRIVE_HOLD_S = 600.0
# Robotiq gripper URCap: its daemon serves the ASCII GET/SET protocol on the
# controller's loopback only (verified on the UR3e + Hand-E, 2026-09-25: closed
# from the network, open from URScript), so the gripper is driven by a script.
GRIPPER_DAEMON_HOST, GRIPPER_DAEMON_PORT = "127.0.0.1", 63352
GRIPPER_ACTIONS = ("status", "open", "close", "move", "activate")
GRIPPER_STATUS_VARS = ("STA", "ACT", "POS", "PRE", "OBJ", "FLT")


def _sanitize_prompt(text: str, *, max_len: int = 200) -> str:
    """Make ``text`` safe to embed in a URScript double-quoted string literal.

    URScript string literals have no escape sequence for an embedded ``"``, and
    a newline would split the submission into separate programs — so collapse
    all whitespace to single spaces, drop double-quotes, and cap the length
    (pendant popups are narrow). Defensive, not security-critical: the prompt is
    operator-facing text, not a trust boundary.
    """
    return " ".join(text.split()).replace('"', "'")[:max_len]


def _parse_gripper_status(captured: list[str]) -> dict[str, int]:
    """``urctl/rq/VAR=<reply>`` lines → ``{VAR: int}`` (the daemon answers a bare
    value; a ``VAR value`` form is accepted too — the last integer wins)."""
    import re

    out: dict[str, int] = {}
    for line in captured:
        for var in GRIPPER_STATUS_VARS:
            tag = f"urctl/rq/{var}="
            if tag in line:
                m = re.search(r"(-?\d+)\s*$", line.split(tag, 1)[1].strip())
                if m:
                    out[var] = int(m.group(1))
    return out


class Robot:
    def __init__(
        self,
        config: RobotConfig | None = None,
        *,
        safety: SafetyEnvelope | None = None,
        audit: AuditLog | None = None,
        dry_run: bool = False,
    ):
        self.config = config or RobotConfig.from_env()
        # The reach cap is per model (a UR3e reaches 0.5 m, a UR20 1.75 m). An
        # explicit envelope is the caller's business; otherwise size it from the
        # configured model (``UR_ROBOT_MODEL``, the cell files) and, failing
        # that, ask the controller once before the first absolute TCP move.
        self._safety_explicit = safety is not None
        self.safety = safety or SafetyEnvelope.for_model(
            self.config.robot_model, max_reach=self.config.max_reach
        )
        self._model_probed = self._safety_explicit or self.safety.reach_known
        self.audit = audit or AuditLog()
        self.dry_run = dry_run
        # The orchestration client (power/brake/load/play/state). On PolyScope X
        # this is the REST Robot-API; on e-Series it is the Dashboard server.
        # Both expose the same method surface, so the rest of this class is
        # platform-agnostic. The attribute keeps the name ``dashboard`` for
        # continuity even though it may hold a RobotAPIClient.
        self.dashboard = (
            RobotAPIClient(self.config) if self.config.is_polyscopex() else DashboardClient(self.config)
        )
        self.primary = PrimaryClient(self.config)
        # RTDE (port 30004) is created lazily on first use so tests and code
        # paths that never touch it pay no socket. ``_rtde_deep`` tracks which
        # output recipe the cached client negotiated (default vs DEEP_OUTPUTS).
        self._rtde = None
        self._rtde_deep = False

    # ----- reach / model -----------------------------------------------------

    def _ensure_reach(self) -> None:
        """Size the envelope's reach cap from the controller's reported model
        (Dashboard ``get robot model``) when nothing configured it. Runs once;
        an unreachable controller or an unknown reply keeps the UR10 default,
        and the ``tcp_reach`` violation text then says the model is unknown."""
        if self._model_probed or self.dry_run:
            return
        self._model_probed = True
        try:
            reported = self.dashboard.robot_model()
        except OSError:
            return
        reach = reach_for_model(reported)
        if reach is not None:
            self.safety = replace(self.safety, max_reach=reach, model=normalize_model(reported))

    def max_reach(self) -> float:
        """The reach cap absolute TCP moves are validated against, in metres —
        after the one-time model probe, so a cockpit can show the verdict
        (``locate`` → ``reachable``) before it offers a Move."""
        self._ensure_reach()
        return self.safety.max_reach

    @property
    def gripper_device(self):
        """The attached end effector as a :class:`urctl.controller.Gripper` — the
        Robotiq-over-URCap adapter around :meth:`gripper`."""
        from .controller import RobotiqUrcapGripper

        return RobotiqUrcapGripper(self)

    def _rtde_client(self, *, deep: bool = False):
        """Lazily build (and cache) the RTDE client. Dropped on read failure so
        the next call reconnects (see :meth:`rtde_state` / :meth:`get_state`).

        ``deep=True`` upgrades the cached client to the full diagnostic recipe
        (:data:`urctl.rtde.DEEP_OUTPUTS`, tolerant mode). The deep recipe is a
        superset of the default one, so once upgraded it serves both callers.
        """
        from .rtde import DEEP_OUTPUTS, RtdeClient

        if deep and self._rtde is not None and not self._rtde_deep:
            self._rtde.close()
            self._rtde = None
        if self._rtde is None:
            if deep:
                self._rtde = RtdeClient(self.config, outputs=list(DEEP_OUTPUTS), strict=False)
            else:
                self._rtde = RtdeClient(self.config)
            self._rtde_deep = deep
        return self._rtde

    def close(self) -> None:
        """Release the RTDE socket. Optional — short-lived CLI processes can let
        the OS reclaim it, but long-lived MCP/agent hosts should call this."""
        if self._rtde is not None:
            self._rtde.close()
            self._rtde = None

    def __enter__(self) -> Robot:
        return self

    def __exit__(self, *exc) -> None:
        self.close()

    # ----- audit helper ------------------------------------------------------

    def _log(
        self,
        action: str,
        args: dict,
        *,
        ok: bool,
        result: dict | None = None,
        safety: dict | None = None,
    ) -> dict:
        rec = self.audit.record(
            action,
            host=self.config.host,
            args=args,
            ok=ok,
            result=result or {},
            safety=safety,
            dry_run=self.dry_run,
        )
        out = {"action": action, "ok": ok, "dry_run": self.dry_run, "host": self.config.host}
        if safety is not None:
            out["safety"] = safety
        out.update(result or {})
        # ts lets a caller correlate the return value with the audit line.
        out["ts"] = rec.ts
        return out

    # ----- state -------------------------------------------------------------

    def robot_mode(self) -> str:
        return self.dashboard.robot_mode()

    def safety_mode(self) -> str:
        return self.dashboard.safety_mode()

    def program_state(self) -> str:
        return self.dashboard.program_state()

    def is_running(self) -> bool:
        return self.dashboard.is_running()

    def get_state(self, *, collect_for: float = 2.0) -> dict:
        """A normalized observation: modes from Dashboard + live joints/TCP.

        This is the "multimodal observation normalization" an agent reads
        before deciding what to do. Joints/TCP come from RTDE (port 30004) when
        available — structured, and readable even when no program runs — with
        force/velocity/safety fields added. If RTDE is disabled or unreachable
        it falls back to the legacy Primary ``textmsg`` path (which only yields
        values while RUNNING).
        """
        robot_mode = self.dashboard.robot_mode()
        safety_mode = self.dashboard.safety_mode()
        program_state = self.dashboard.program_state()
        control_mode = self.dashboard.control_mode()
        running = "RUNNING" in robot_mode
        joints = tcp = None
        extra: dict = {}
        if self.config.rtde_enabled and not self.dry_run:
            try:
                raw = self._rtde_client().read_outputs()
                joints = raw.get("actual_q")
                tcp = raw.get("actual_TCP_pose")
                extra = {
                    "joint_velocities": raw.get("actual_qd"),
                    "tcp_speed": raw.get("actual_TCP_speed"),
                    "tcp_force": raw.get("actual_TCP_force"),
                    "safety_status": raw.get("safety_status_bits"),
                }
            except OSError:
                self._rtde = None  # drop the dead client so the next call reconnects
            except Exception:
                # Any RTDE protocol error (RtdeError, recipe mismatch): fall back.
                self._rtde = None
        if joints is None and running:
            # Legacy path: a textmsg *program*. Never send it while a motion
            # from this client is in flight — it would replace (kill) the
            # running program (see PrimaryBusyError). Report no joints instead.
            try:
                state = self.primary.read_state(collect_for=collect_for)
            except PrimaryBusyError:
                state = {"joints": None, "tcp": None}
                extra["primary_busy"] = True
            joints, tcp = state["joints"], state["tcp"]
        result = {
            "robot_mode": robot_mode,
            "safety_mode": safety_mode,
            "program_state": program_state,
            # REMOTE/LOCAL. On PolyScope X every mutating command requires REMOTE
            # (see bring_up / RobotAPIClient); surfacing it here makes a refusal
            # diagnosable at a glance.
            "control_mode": control_mode,
            "running": running,
            "joints": joints,
            "tcp": tcp,
            **extra,
        }
        return self._log("get_state", {}, ok=True, result=result)

    def get_flange_pose(
        self, *, collect_for: float = 3.0, script_fallback: bool = True, stand_in: bool = True
    ) -> dict:
        """The tool-flange pose in the base frame, alongside the active TCP pose
        and TCP offset it was derived from — what a camera on the flange needs
        to put its measurements into base coordinates (``perceptronics.handeye``).

        First the controller's **state broadcast** (Secondary port, read only —
        :mod:`urctl.stateframe`): the flange by forward kinematics of the actual
        joints through the robot's calibrated DH, plus the TCP pose and active
        offset from the same message. It sends no script, so it works in Local
        mode on a real e-Series (which ignores URScript there, textmsg included —
        UR3e, 2026-09-27) and never replaces a running program.
        (``source: "state_broadcast"``.)

        Fallback, when the broadcast is not reachable (a sim that doesn't publish
        the port): one Primary ``textmsg`` round-trip — the controller reports
        ``get_actual_tcp_pose()`` and ``get_tcp_offset()``; the flange is
        ``pose_trans(tcp, pose_inv(offset))`` (also computed host-side by
        :mod:`urctl.pose` and cross-checked against the controller's own
        arithmetic). (``source: "textmsg"``.) ``script_fallback=False`` skips it: a
        caller that must never replace a running program (the stand-alone pick
        server, beside an operator's program) gets ``ok: False`` instead. In
        ``dry_run`` a stand-in pose is
        returned (tool pointing down, 0.5 m out and up) so cockpits can be
        exercised without a controller. ``stand_in=False`` is for a dry run that
        measures from the arm's pose (``pick-cycle --dry-run``): it reads the
        broadcast — a read, nothing is sent — and answers ``ok: False`` when
        there is none, rather than a made-up pose or a script.
        """
        from . import stateframe
        from .pose import Transform

        if self.dry_run and stand_in:
            flange = [0.5, 0.0, 0.5, 0.0, math.pi, 0.0]
            return self._log(
                "get_flange_pose",
                {},
                ok=True,
                result={"flange": flange, "tcp": flange, "tcp_offset": [0.0] * 6, "dry_run": True},
            )
        try:
            st = stateframe.read_flange_state(
                self.config.host, self.config.secondary_port, timeout_s=min(3.0, collect_for + 1.0)
            )
        except (OSError, stateframe.StateFrameError) as exc:
            state_error = f"{self.config.host}:{self.config.secondary_port}: {exc}"
        else:
            if st.get("flange") is not None and st.get("tcp") is not None:
                consistency = st.get("consistency_m")
                result = {
                    "flange": st["flange"],
                    "tcp": st["tcp"],
                    "tcp_offset": st["tcp_offset"],
                    "joints": st["joints"],
                    "source": "state_broadcast",
                    "flange_source": st["flange_source"],
                    "consistency_m": consistency,
                    # the reported offset is the one the reported TCP pose was computed with
                    "tcp_offset_consistent": None
                    if consistency is None
                    else consistency <= stateframe.CONSISTENCY_TOLERANCE_M,
                }
                return self._log("get_flange_pose", {}, ok=True, result=result)
            state_error = "the state broadcast had no joints/TCP pose"
        if not script_fallback or self.dry_run:
            return self._log("get_flange_pose", {}, ok=False, result={"error": state_error})
        captured = self.primary.run_and_capture(
            'textmsg("urctl/flange/tcp=", get_actual_tcp_pose())\n'
            'textmsg("urctl/flange/offset=", get_tcp_offset())\n'
            'textmsg("urctl/flange/pose=", pose_trans(get_actual_tcp_pose(), pose_inv(get_tcp_offset())))\n',
            fn_name="urctl_flange_pose",
            marker="urctl/flange",
            collect_for=collect_for,
            stop_marker="urctl/flange/pose=",
        )
        from .primary import parse_vector

        tcp = parse_vector(captured, "urctl/flange/tcp")
        offset = parse_vector(captured, "urctl/flange/offset")
        reported = parse_vector(captured, "urctl/flange/pose")
        if tcp is None or offset is None:
            return self._log(
                "get_flange_pose",
                {},
                ok=False,
                result={
                    "error": "no TCP pose/offset surfaced on the Primary broadcast",
                    "state_error": state_error,
                    "captured": captured[-5:],
                },
            )
        flange = Transform.from_pose(tcp).compose(Transform.from_pose(offset).inverse()).to_pose()
        result = {
            "flange": flange,
            "tcp": tcp,
            "tcp_offset": offset,
            "source": "textmsg",
            "state_error": state_error,
        }
        if reported is not None:
            result["flange_reported"] = reported
            result["host_controller_mismatch_m"] = max(
                abs(a - b) for a, b in zip(flange[:3], reported[:3], strict=False)
            )
        return self._log("get_flange_pose", {}, ok=True, result=result)

    def get_tcp_offset(self) -> dict:
        """The controller's active TCP offset (``[x, y, z, rx, ry, rz]``, metres +
        rotation vector, flange frame) — from the state broadcast, so it works in
        Local mode and sends no script (:meth:`get_flange_pose`)."""
        fp = self.get_flange_pose(stand_in=False)
        offset = fp.get("tcp_offset") if fp.get("ok") else None
        result = {"tcp_offset": offset, "source": fp.get("source")}
        if offset is None:
            result["error"] = fp.get("error") or "no TCP offset reported"
        return self._log("get_tcp_offset", {}, ok=offset is not None, result=result)

    def set_tcp_offset(self, offset: list[float], *, collect_for: float = 5.0) -> dict:
        """Make ``offset`` the controller's active TCP (URScript ``set_tcp``) and read it
        back. It stays active until the next ``set_tcp``, an installation load or a
        program started from the pendant (which re-applies the installation's default
        TCP). On a real e-Series in Local mode URScript is ignored — the read-back then
        fails and ``ok`` is false. The same path serves PolyScope X (Primary, Remote)."""
        off = [float(v) for v in offset]
        if len(off) != 6 or not all(math.isfinite(v) for v in off):
            raise ValueError("offset must be 6 finite numbers [x, y, z, rx, ry, rz]")
        args = {"offset": off}
        if self.dry_run:
            return self._log("set_tcp_offset", args, ok=True, result={"reply": "(dry-run)"})
        literal = "p[" + ", ".join(repr(v) for v in off) + "]"
        captured = self.primary.run_and_capture(
            f'set_tcp({literal})\ntextmsg("urctl/tcp/done=", get_tcp_offset())\n',
            fn_name="urctl_set_tcp",
            marker="urctl/tcp",
            collect_for=collect_for,
            stop_marker="urctl/tcp/done=",
        )
        from .primary import parse_vector

        active = parse_vector(captured, "urctl/tcp/done")
        ok = active is not None and max(abs(a - b) for a, b in zip(active, off, strict=False)) < 1e-4
        return self._log("set_tcp_offset", args, ok=ok, result={"tcp_offset": active})

    def rtde_state(self, *, deep: bool = False) -> dict:
        """High-rate structured state via RTDE (port 30004).

        Unlike :meth:`get_state`, this is RTDE-only and works even when the
        robot is not RUNNING — RTDE reflects the controller directly, not a
        URScript ``textmsg`` we injected. Returns joints/velocities, TCP
        pose/speed/force, and decoded safety/runtime state. Returns
        ``ok=False`` if RTDE is disabled or unreachable.

        ``deep=True`` subscribes the full diagnostic recipe
        (:data:`urctl.rtde.DEEP_OUTPUTS`): per-joint currents, temperatures,
        voltages and drive modes, supply power, tool-connector telemetry,
        analog IO, and speed scaling. Fields a given firmware doesn't offer are
        dropped, not fatal (listed under ``unavailable_fields``).
        """
        from . import codes

        params = {"deep": deep} if deep else {}
        if self.dry_run:
            return self._log("rtde_state", params, ok=True, result={"reply": "(dry-run)"})
        if not self.config.rtde_enabled:
            return self._log(
                "rtde_state", params, ok=False, result={"error": "RTDE disabled (UR_RTDE_DISABLE)"}
            )
        try:
            client = self._rtde_client(deep=deep)
            raw = client.read_outputs()
        except OSError as exc:
            self._rtde = None
            return self._log("rtde_state", params, ok=False, result={"error": f"RTDE unreachable: {exc}"})
        except Exception as exc:  # RtdeError and friends
            self._rtde = None
            return self._log("rtde_state", params, ok=False, result={"error": str(exc)})
        result = codes.shape_rtde_sample(raw, deep=deep, unavailable=client.dropped_outputs if deep else None)
        return self._log("rtde_state", params, ok=True, result=result)

    # ----- power -------------------------------------------------------------

    def power_on(self, *, wait: bool = True, timeout: float = 60.0) -> dict:
        if self.dry_run:
            return self._log("power_on", {}, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.power_on()
        if wait:
            # URSim often blows through IDLE straight to RUNNING, so accept either.
            self.dashboard.wait_for(lambda r: "IDLE" in r or "RUNNING" in r, timeout=timeout)
        return self._log("power_on", {}, ok=True, result={"reply": reply})

    def power_off(self) -> dict:
        if self.dry_run:
            return self._log("power_off", {}, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.power_off()
        return self._log("power_off", {}, ok=True, result={"reply": reply})

    def brake_release(self, *, wait: bool = True, timeout: float = 60.0) -> dict:
        if self.dry_run:
            return self._log("brake_release", {}, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.brake_release()
        if wait:
            self.dashboard.wait_for(lambda r: "RUNNING" in r, timeout=timeout)
        return self._log("brake_release", {}, ok=True, result={"reply": reply})

    def bring_up(self, *, timeout: float = 120.0) -> dict:
        """Cold start -> RUNNING: clear latched safety, power on, release brakes.

        A Python port of ``scripts/poweron.sh`` so it works against any host
        without the shell script. Idempotent: safe to call when already RUNNING.
        """
        if self.dry_run:
            return self._log("bring_up", {}, ok=True, result={"reply": "(dry-run)"})
        # PolyScope X gates power-on / brake-release behind Remote control mode
        # (HTTP 403 otherwise), and there is no network way to switch to Remote.
        # Detect it up front so we return a clear message instead of polling
        # robot_mode for the full timeout while the controller silently refuses.
        if self.config.is_polyscopex() and not self.dashboard.is_remote_control():
            return self._log(
                "bring_up",
                {},
                ok=False,
                result={
                    "reply": "PolyScope X is in Local control mode; switch to Remote on the "
                    "Safety screen in the PolyScope X UI (localhost:8000) before bringing up "
                    "the robot over the network."
                },
            )
        self.dashboard.close_safety_popup()
        self.dashboard.close_popup()
        if "PROTECTIVE_STOP" in self.dashboard.safety_mode():
            self.dashboard.unlock_protective_stop()
        self.dashboard.power_on()
        self.dashboard.wait_for(lambda r: "IDLE" in r or "RUNNING" in r, timeout=timeout)
        self.dashboard.brake_release()
        final = self.dashboard.wait_for(lambda r: "RUNNING" in r, timeout=timeout)
        ok = "RUNNING" in final
        return self._log("bring_up", {}, ok=ok, result={"robot_mode": final})

    # ----- motion ------------------------------------------------------------

    def move_joints(
        self,
        target: list[float],
        *,
        velocity: float = DEFAULT_VELOCITY,
        acceleration: float = DEFAULT_ACCELERATION,
        wait: bool = True,
        timeout: float = 30.0,
    ) -> dict:
        """Move to a joint-space target via ``movej``, after safety validation.

        Raises :class:`urctl.safety.SafetyError`-equivalent rejection by
        returning ``ok=False`` with the verdict; nothing is sent to the robot
        when the verdict is unsafe (or when ``dry_run`` is set).
        """
        args = {"target": target, "velocity": velocity, "acceleration": acceleration}
        verdict: SafetyVerdict = self.safety.validate_move_joints(
            target,
            velocity=velocity,
            acceleration=acceleration,
            robot_mode=None if self.dry_run else self.dashboard.robot_mode(),
        )
        if not verdict.ok:
            return self._log("move_joints", args, ok=False, safety=verdict.as_dict())
        if self.dry_run:
            return self._log(
                "move_joints",
                args,
                ok=True,
                safety=verdict.as_dict(),
                result={"reply": "(dry-run)"},
            )

        body = (
            f"movej({target}, a={acceleration}, v={velocity})\n"
            "sync()\n"
            'textmsg("urctl/move/done=", get_actual_joint_positions())\n'
        )
        if not wait:
            self.primary.run(body, fn_name="urctl_move")
            return self._log("move_joints", args, ok=True, safety=verdict.as_dict(), result={"waited": False})

        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_move",
            marker="urctl/move",
            collect_for=timeout,
            stop_marker="urctl/move/done=",
        )
        from .primary import parse_vector

        landed = parse_vector(captured, "urctl/move/done")
        ok = (
            landed is not None
            and max(abs(a - b) for a, b in zip(landed, target, strict=False)) < LANDING_TOLERANCE
        )
        result = {"landed": landed, "waited": True}
        if landed is None and "PROTECTIVE_STOP" in self.dashboard.safety_mode():
            result["protective_stop"] = True
        return self._log("move_joints", args, ok=ok, safety=verdict.as_dict(), result=result)

    def move_home(self, **kwargs) -> dict:
        return self.move_joints(HOME_JOINTS, **kwargs)

    def ik_has_solution(
        self,
        poses: list[list[float]],
        *,
        tcp: list[float] | None = None,
        collect_for: float = 3.0,
    ) -> list[bool | None]:
        """Whether each base-frame pose has an inverse-kinematics solution on the
        controller — the verdicts of :meth:`inverse_kin`."""
        return [a["reachable"] for a in self.inverse_kin(poses, tcp=tcp, collect_for=collect_for)]

    def inverse_kin(
        self,
        poses: list[list[float]],
        *,
        tcp: list[float] | None = None,
        collect_for: float = 3.0,
    ) -> list[dict]:
        """Ask the controller to solve each base-frame pose (URScript
        ``get_inverse_kin_has_solution``, then ``get_inverse_kin`` nearest the live
        joints; both verified on a PolyScope 5.25.1 UR3e, 2026-09-27) — for the
        active TCP, or for ``tcp`` (``[0]*6`` = the flange) without changing it.
        One Primary round-trip for all poses; no motion, so it answers in Local
        mode on e-Series. Each answer is ``{"reachable": bool | None, "joints":
        [6 rad] | None}``; ``reachable`` is ``None`` when there is no answer:
        dry-run, Primary busy/unreachable, or PolyScope X in Local mode (which
        ignores Primary scripts)."""
        none = {"reachable": None, "joints": None}
        try:
            clean = [[float(v) for v in p] for p in poses]
        except (TypeError, ValueError):
            return [dict(none) for _ in poses]  # the envelope names the bad value
        if (
            self.dry_run
            or not clean
            or any(len(p) != 6 or not all(math.isfinite(v) for v in p) for p in clean)
        ):
            return [dict(none) for _ in clean]
        tcp_arg = ""
        if tcp is not None:
            t = [float(v) for v in tcp]
            if len(t) != 6 or not all(math.isfinite(v) for v in t):
                raise ValueError("tcp must be 6 finite numbers [x, y, z, rx, ry, rz]")
            tcp_arg = ", tcp=p[" + ", ".join(str(v) for v in t) + "]"
        lines = []
        for i, p in enumerate(clean):
            target = "p[" + ", ".join(str(v) for v in p) + "]" + tcp_arg
            lines.append(
                f"if get_inverse_kin_has_solution({target}):\n"
                f'  textmsg("urctl/ik/{i}=", get_inverse_kin({target}))\n'
                "else:\n"
                f'  textmsg("urctl/ik/{i}=", False)\n'
                "end\n"
            )
        body = "".join(lines) + 'textmsg("urctl/ik/done=", 1)\n'
        try:
            captured = self.primary.run_and_capture(
                body,
                fn_name="urctl_ik_check",
                marker="urctl/ik",
                collect_for=collect_for,
                stop_marker="urctl/ik/done=",
            )
        except OSError:
            return [dict(none) for _ in clean]
        from .primary import parse_vector

        answers = [dict(none) for _ in clean]
        for i in range(len(clean)):
            tag = f"urctl/ik/{i}"
            joints = parse_vector(captured, tag + "=")
            if joints is not None and len(joints) == 6:
                answers[i] = {"reachable": True, "joints": joints}
            elif any(
                (tag + "=") in line and line.split(tag + "=", 1)[1].strip().lower().startswith("false")
                for line in captured
            ):
                answers[i] = {"reachable": False, "joints": None}
        return answers

    def move_tcp(
        self,
        pose: list[float],
        *,
        relative: bool = False,
        velocity: float = DEFAULT_TCP_VELOCITY,
        acceleration: float = DEFAULT_TCP_ACCELERATION,
        wait: bool = True,
        timeout: float = 30.0,
        tcp: list[float] | None = None,
    ) -> dict:
        """Linear Cartesian move (``movel``), after safety validation.

        ``pose`` is ``[x, y, z, rx, ry, rz]`` (metres + rotation-vector radians).
        ``tcp`` overrides the controller's active TCP for this move: the program
        runs ``set_tcp(p[...])`` before the ``movel`` so ``pose`` is where *that*
        TCP lands — ``[0]*6`` targets the tool flange itself, regardless of what
        the installation/pendant says. (Verified need: 2026-09-23 the UR3e's
        active TCP carried a -35 mm Y offset the physical tool didn't, and the
        flange landed 35 mm off.) The override sticks on the controller until
        the next ``set_tcp`` / installation reload, as any URScript ``set_tcp`` does.
        With ``relative=True`` it is a **base-frame delta** added to the live TCP
        pose (e.g. ``[0, 0.05, 0, 0, 0, 0]`` nudges +50 mm along base +Y); with
        ``relative=False`` it is an absolute base-frame target.

        The URScript is wrapped in a ``def`` so it runs as a single program on
        the Primary client — a bare top-level ``movel`` gets superseded and
        silently does nothing (see CLAUDE.md). Returns ``ok=False`` with the
        verdict (and sends nothing) when the move is unsafe or in ``dry_run``.
        """
        args = {
            "pose": pose,
            "relative": relative,
            "velocity": velocity,
            "acceleration": acceleration,
        }
        if tcp is not None:
            tcp = [float(v) for v in tcp]
            if len(tcp) != 6 or not all(math.isfinite(v) for v in tcp):
                raise ValueError("tcp must be 6 finite numbers [x, y, z, rx, ry, rz]")
            args["tcp"] = tcp
        ik = None
        if not relative:
            self._ensure_reach()
            ik = self.ik_has_solution([pose], tcp=tcp)[0] if len(pose) == 6 else None
        verdict: SafetyVerdict = self.safety.validate_move_tcp(
            pose,
            velocity=velocity,
            acceleration=acceleration,
            relative=relative,
            robot_mode=None if self.dry_run else self.dashboard.robot_mode(),
            ik_reachable=ik,
        )
        if not verdict.ok:
            return self._log("move_tcp", args, ok=False, safety=verdict.as_dict())
        if self.dry_run:
            return self._log(
                "move_tcp",
                args,
                ok=True,
                safety=verdict.as_dict(),
                result={"reply": "(dry-run)"},
            )

        pose_literal = "p[" + ", ".join(str(float(x)) for x in pose) + "]"
        target_expr = f"pose_add(get_actual_tcp_pose(), {pose_literal})" if relative else pose_literal
        set_tcp = ("set_tcp(p[" + ", ".join(str(v) for v in tcp) + "])\n") if tcp is not None else ""
        body = (
            set_tcp + f"movel({target_expr}, a={acceleration}, v={velocity})\n"
            "sync()\n"
            'textmsg("urctl/move/done=", get_actual_tcp_pose())\n'
        )
        if not wait:
            self.primary.run(body, fn_name="urctl_move_tcp")
            return self._log("move_tcp", args, ok=True, safety=verdict.as_dict(), result={"waited": False})

        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_move_tcp",
            marker="urctl/move",
            collect_for=timeout,
            stop_marker="urctl/move/done=",
        )
        from .primary import parse_vector

        landed = parse_vector(captured, "urctl/move/done")
        if relative:
            # We can't know the absolute target without the (server-side) live
            # pose; a returned "done" pose proves the move ran to completion.
            ok = landed is not None
        else:
            ok = (
                landed is not None
                and max(abs(a - b) for a, b in zip(landed[:3], pose[:3], strict=False))
                < TCP_LANDING_TOLERANCE
            )
        result = {"landed": landed, "waited": True}
        if landed is None:
            # The move never confirmed. The most common cause on real hardware
            # and URSim is a protective stop mid-move (e.g. a movel through a
            # singularity, error C154A0) — surface it so the failure isn't a
            # silent ok=False with no reason.
            if "PROTECTIVE_STOP" in self.dashboard.safety_mode():
                result["protective_stop"] = True
        return self._log("move_tcp", args, ok=ok, safety=verdict.as_dict(), result=result)

    def move_tcp_path(
        self,
        legs: list[dict],
        *,
        tcp: list[float] | None = None,
        timeout: float = 120.0,
        gripper_first: int | None = None,
    ) -> dict:
        """Run several absolute ``movel`` legs as **one** program on one Primary
        connection — an approach cycle (over → down → dwell → up → back) is one
        submission instead of four reconnects (rapid reconnects wedge URControl;
        see CLAUDE.md "How fast can you actually drive it").

        Each leg is ``{"pose": [x,y,z,rx,ry,rz], "velocity"?, "acceleration"?,
        "dwell_s"?, "blend_m"?, "gripper"?}`` — the robot ``sleep``s ``dwell_s``
        after landing that leg; ``blend_m`` > 0 rounds the corner into the next
        leg (``movel(..., r=)``: the leg is not stopped at, so its landed check
        is loosened to the blend radius; never on the last leg); ``gripper``
        ``"open"``/``"close"`` runs the Robotiq daemon on the controller's
        loopback (:meth:`gripper`) after the leg lands and echoes
        ``urctl/path/grip<i>=POS OBJ`` — so a whole pick (hover → edge → grasp
        → close → lift → set → open → clear) is **one** program.
        Every leg is validated against the envelope (reach, speed) up front with
        one mode check; nothing is sent if any leg fails. ``tcp`` runs
        ``set_tcp`` first (``[0]*6`` = the flange). Each leg echoes its landed
        pose (``urctl/path/leg<i>=``); ``ok`` means every leg landed within
        :data:`TCP_LANDING_TOLERANCE` of its target and the final marker came.
        ``gripper_first`` (Robotiq position 0..255) is sent before the first leg and
        not waited for — the fingers move while the arm does (a pick's pre-open).
        """
        if not legs:
            raise ValueError("legs must not be empty")
        if gripper_first is not None and (
            isinstance(gripper_first, bool)
            or not isinstance(gripper_first, int)
            or not 0 <= gripper_first <= 255
        ):
            raise ValueError("gripper_first must be an integer position 0..255")
        norm: list[dict] = []
        for idx, leg in enumerate(legs):
            pose = [float(v) for v in leg["pose"]]
            if len(pose) != 6 or not all(math.isfinite(v) for v in pose):
                raise ValueError(f"legs[{idx}].pose must be 6 finite numbers")
            dwell = float(leg.get("dwell_s", 0.0) or 0.0)
            if not math.isfinite(dwell) or dwell < 0.0 or dwell > 60.0:
                raise ValueError(f"legs[{idx}].dwell_s must be within 0..60 s")
            blend = float(leg.get("blend_m", 0.0) or 0.0)
            if not math.isfinite(blend) or blend < 0.0 or blend > 0.1:
                raise ValueError(f"legs[{idx}].blend_m must be within 0..0.1 m")
            grip = leg.get("gripper")
            if grip not in (None, "open", "close"):
                raise ValueError(f"legs[{idx}].gripper must be open, close or absent")
            norm.append(
                {
                    "pose": pose,
                    "velocity": float(leg.get("velocity", DEFAULT_TCP_VELOCITY)),
                    "acceleration": float(leg.get("acceleration", DEFAULT_TCP_ACCELERATION)),
                    "dwell_s": dwell,
                    "blend_m": blend if idx < len(legs) - 1 else 0.0,
                    "gripper": grip,
                }
            )
        if tcp is not None:
            tcp = [float(v) for v in tcp]
            if len(tcp) != 6 or not all(math.isfinite(v) for v in tcp):
                raise ValueError("tcp must be 6 finite numbers [x, y, z, rx, ry, rz]")
        args = {"legs": norm, "tcp": tcp, "gripper_first": gripper_first}
        self._ensure_reach()
        robot_mode = None if self.dry_run else self.dashboard.robot_mode()
        iks = self.ik_has_solution([leg["pose"] for leg in norm], tcp=tcp)
        verdicts = []
        for idx, leg in enumerate(norm):
            verdict = self.safety.validate_move_tcp(
                leg["pose"],
                velocity=leg["velocity"],
                acceleration=leg["acceleration"],
                relative=False,
                robot_mode=robot_mode,
                ik_reachable=iks[idx],
            )
            if not verdict.ok:
                safety = verdict.as_dict()
                safety["leg"] = idx
                return self._log("move_tcp_path", args, ok=False, safety=safety)
            verdicts.append(verdict)
        safety = {"ok": True, "violations": [], "legs": len(norm)}
        if self.dry_run:
            return self._log("move_tcp_path", args, ok=True, safety=safety, result={"reply": "(dry-run)"})

        lines = []
        if tcp is not None:
            lines.append("set_tcp(p[" + ", ".join(str(v) for v in tcp) + "])")
        uses_gripper = any(leg["gripper"] for leg in norm) or gripper_first is not None
        if uses_gripper:
            lines.append(
                f'urctl_rq_ok = socket_open("{GRIPPER_DAEMON_HOST}", {GRIPPER_DAEMON_PORT}, "urctl_rq")'
            )
            lines.append('textmsg("urctl/path/gripper=", urctl_rq_ok)')
        if gripper_first is not None:  # fire and go: the fingers travel while the arm does
            lines += [
                "if urctl_rq_ok:",
                f'  socket_send_line("SET POS {gripper_first}", "urctl_rq")',
                '  urctl_rq_r = socket_read_string("urctl_rq", timeout=2.0)',
                '  socket_send_line("SET GTO 1", "urctl_rq")',
                '  urctl_rq_r = socket_read_string("urctl_rq", timeout=2.0)',
                "end",
            ]
        for idx, leg in enumerate(norm):
            literal = "p[" + ", ".join(str(x) for x in leg["pose"]) + "]"
            r = f", r={leg['blend_m']}" if leg["blend_m"] > 0 else ""
            lines.append(f"movel({literal}, a={leg['acceleration']}, v={leg['velocity']}{r})")
            if leg["blend_m"] <= 0:
                lines.append("sync()")
                lines.append(f'textmsg("urctl/path/leg{idx}=", get_actual_tcp_pose())')
            if leg["gripper"]:
                pos = 255 if leg["gripper"] == "close" else 0
                lines += [
                    "if urctl_rq_ok:",
                    f'  socket_send_line("SET POS {pos}", "urctl_rq")',
                    '  urctl_rq_r = socket_read_string("urctl_rq", timeout=2.0)',
                    '  socket_send_line("SET GTO 1", "urctl_rq")',
                    '  urctl_rq_r = socket_read_string("urctl_rq", timeout=2.0)',
                    "  sleep(0.2)",
                    "  urctl_rq_t = 0.0",
                    "  while (urctl_rq_t < 4.0):",
                    '    socket_send_line("GET OBJ", "urctl_rq")',
                    '    urctl_rq_o = socket_read_string("urctl_rq", timeout=2.0)',
                    '    if str_find(urctl_rq_o, "0") < 0:',
                    "      urctl_rq_t = 4.0",
                    "    else:",
                    "      sleep(0.05)",
                    "      urctl_rq_t = urctl_rq_t + 0.05",
                    "    end",
                    "  end",
                    '  socket_send_line("GET POS", "urctl_rq")',
                    '  urctl_rq_p = socket_read_string("urctl_rq", timeout=2.0)',
                    # the daemon's replies end in a newline: one value per marker line
                    f'  textmsg("urctl/path/grip{idx}/pos=", urctl_rq_p)',
                    f'  textmsg("urctl/path/grip{idx}/obj=", urctl_rq_o)',
                    "end",
                ]
            if leg["dwell_s"] > 0:
                lines.append(f"sleep({leg['dwell_s']})")
        if uses_gripper:
            lines.append("if urctl_rq_ok:")
            lines.append('  socket_close("urctl_rq")')
            lines.append("end")
        lines.append('textmsg("urctl/path/done=", get_actual_tcp_pose())')
        body = "\n".join(lines) + "\n"
        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_tcp_path",
            marker="urctl/path",
            collect_for=timeout,
            stop_marker="urctl/path/done=",
        )
        import re as _re

        from .primary import parse_vector

        leg_results = []
        for idx, leg in enumerate(norm):
            entry: dict = {"pose": leg["pose"], "dwell_s": leg["dwell_s"], "blend_m": leg["blend_m"]}
            if leg["blend_m"] > 0:
                # a blended leg is passed through, not stopped at: it lands if the program went on
                later = any(
                    f"urctl/path/leg{j}=" in line for j in range(idx + 1, len(norm)) for line in captured
                )
                entry.update({"landed": None, "ok": later or any("urctl/path/done=" in c for c in captured)})
            else:
                landed = parse_vector(captured, f"urctl/path/leg{idx}=")
                hit = landed is not None and (
                    max(abs(a - b) for a, b in zip(landed[:3], leg["pose"][:3], strict=False))
                    < TCP_LANDING_TOLERANCE
                )
                entry.update({"landed": landed, "ok": hit})
            if leg["gripper"]:
                grip: dict = {"action": leg["gripper"]}
                for key in ("pos", "obj"):
                    tag = f"urctl/path/grip{idx}/{key}="
                    for line in captured:
                        if tag in line:
                            m = _re.search(r"(-?\d+)\s*$", line.split(tag, 1)[1].strip())
                            if m:
                                grip[key.upper()] = int(m.group(1))
                if "OBJ" in grip:
                    grip["object_detected"] = grip["OBJ"] in (1, 2)
                if "POS" not in grip:
                    grip["error"] = "no gripper readback (daemon missing, or the program stopped before it)"
                entry["gripper"] = grip
            leg_results.append(entry)
        final = parse_vector(captured, "urctl/path/done=")
        ok = final is not None and all(r["ok"] for r in leg_results)
        if uses_gripper and not any("urctl/path/gripper=True" in c for c in captured):
            ok = False
        result = {
            "legs": leg_results,
            "completed_legs": sum(1 for r in leg_results if r["landed"] is not None),
            "landed": final,
            "waited": True,
        }
        if final is None and "PROTECTIVE_STOP" in self.dashboard.safety_mode():
            result["protective_stop"] = True
        return self._log("move_tcp_path", args, ok=ok, safety=safety, result=result)

    def move_trajectory(
        self,
        waypoints: list[list[float]],
        *,
        velocity: float = DEFAULT_VELOCITY,
        acceleration: float = DEFAULT_ACCELERATION,
        blend_radius: float = 0.0,
        wait: bool = True,
        timeout: float = 120.0,
    ) -> dict:
        """Run a joint-space trajectory (a sequence of ``movej`` waypoints) as a
        single program over one Primary connection.

        This is the fast, robust way to *move the robot a lot*: a confirmed
        single move costs a fixed handshake (a Dashboard mode check + a broadcast
        round-trip, ~1 s on URSim) and opens a fresh Primary socket each time, so
        issuing many moves back-to-back both wastes that handshake per move and —
        critically — wedges/powers off the controller after ~10 rapid reconnects
        (URControl logs ``Socket::OperatorOverload``). Streaming the whole path in
        one ``def`` pays the handshake once and never reconnects mid-trajectory.

        Every waypoint is validated against the safety envelope (joint limits +
        velocity/accel) before anything is sent; one unsafe waypoint rejects the
        whole trajectory. ``blend_radius`` (metres) blends consecutive segments
        for continuous motion; it is dropped on the final waypoint so the robot
        comes to rest on target (a non-zero blend on the last movej is a URScript
        error). Returns ``ok`` plus the final landed joint positions.
        """
        args = {
            "waypoints": waypoints,
            "velocity": velocity,
            "acceleration": acceleration,
            "blend_radius": blend_radius,
            "n": len(waypoints),
        }
        if not waypoints:
            return self._log("move_trajectory", args, ok=False, result={"error": "no waypoints"})

        # One mode check for the whole trajectory (vs one per move), then
        # validate every waypoint's geometry with robot_mode already confirmed.
        robot_mode = None if self.dry_run else self.dashboard.robot_mode()
        for idx, wp in enumerate(waypoints):
            verdict = self.safety.validate_move_joints(
                wp, velocity=velocity, acceleration=acceleration, robot_mode=robot_mode
            )
            if not verdict.ok:
                v = verdict.as_dict()
                v["waypoint"] = idx
                return self._log("move_trajectory", args, ok=False, safety=v)
        if self.dry_run:
            return self._log("move_trajectory", args, ok=True, result={"reply": "(dry-run)"})

        # Blend every segment except the last (a blend radius on the final movej
        # leaves the move unfinished / errors); the trailing sync() + textmsg
        # only fire once the whole path is done.
        lines = []
        for idx, wp in enumerate(waypoints):
            r = 0.0 if idx == len(waypoints) - 1 else blend_radius
            r_arg = f", r={r}" if r else ""
            lines.append(f"movej({list(wp)}, a={acceleration}, v={velocity}{r_arg})")
        body = "\n".join(lines) + '\nsync()\ntextmsg("urctl/move/done=", get_actual_joint_positions())\n'
        if not wait:
            self.primary.run(body, fn_name="urctl_trajectory")
            return self._log("move_trajectory", args, ok=True, result={"waited": False})

        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_trajectory",
            marker="urctl/move",
            collect_for=timeout,
            stop_marker="urctl/move/done=",
        )
        from .primary import parse_vector

        landed = parse_vector(captured, "urctl/move/done")
        ok = (
            landed is not None
            and max(abs(a - b) for a, b in zip(landed, waypoints[-1], strict=False)) < LANDING_TOLERANCE
        )
        result = {"landed": landed, "waited": True}
        if landed is None and "PROTECTIVE_STOP" in self.dashboard.safety_mode():
            result["protective_stop"] = True
        return self._log("move_trajectory", args, ok=ok, result=result)

    def freedrive(self, enable: bool, *, hold_s: float = DEFAULT_FREEDRIVE_HOLD_S) -> dict:
        """Enable/disable freedrive (hand-guiding). All six axes in base frame.

        Freedrive lives exactly as long as the program that called
        ``freedrive_mode()``: a real e-Series drops it the instant the script
        ends (verified on the UR3e, 2026-09-23 — a bare one-liner "worked" on
        URSim and did nothing on hardware). So enabling sends a program that
        stays running — a ``sleep`` loop bounded by ``hold_s`` (default 10 min,
        max 1 h) — and confirms the mode is up by its ``textmsg`` marker;
        ``ok`` is False when that marker never surfaces. Disabling sends
        ``end_freedrive_mode()`` as a new program, which also kills the hold.
        The hold releases itself when it expires (``urctl/freedrive=expired``
        in the broadcast), so an orphaned button press can't leave the arm
        limp indefinitely.
        """
        hold = max(1.0, min(float(hold_s), 3600.0))
        args = {"enable": enable, "hold_s": hold} if enable else {"enable": enable}
        if self.dry_run:
            return self._log("freedrive", args, ok=True, result={"reply": "(dry-run)"})
        if enable:
            body = (
                "freedrive_mode()\n"
                'textmsg("urctl/freedrive=on")\n'
                "urctl_fd_t = 0.0\n"
                f"while (urctl_fd_t < {hold:.1f}):\n"
                "  sleep(0.5)\n"
                "  urctl_fd_t = urctl_fd_t + 0.5\n"
                "end\n"
                "end_freedrive_mode()\n"
                'textmsg("urctl/freedrive=expired")\n'
            )
            marker = "urctl/freedrive=on"
        else:
            body = 'end_freedrive_mode()\ntextmsg("urctl/freedrive=off")\n'
            marker = "urctl/freedrive=off"
        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_freedrive",
            marker="urctl/freedrive",
            collect_for=5.0,
            stop_marker=marker,
        )
        confirmed = any(marker in line for line in captured)
        result = {"confirmed": confirmed}
        if enable:
            result["held_s"] = hold
        # Disabling is best-effort by design (the program may already be gone):
        # report it ok and let ``confirmed`` say whether the controller echoed.
        return self._log("freedrive", args, ok=confirmed or not enable, result=result)

    # ----- Robotiq gripper ---------------------------------------------------

    def gripper(
        self,
        action: str,
        *,
        position: int | None = None,
        speed: int = 255,
        force: int = 100,
        timeout_s: float = 5.0,
    ) -> dict:
        """Drive a Robotiq gripper (Hand-E / 2F) through its URCap daemon.

        The Robotiq URCap runs a daemon on the controller that speaks a line
        protocol — ``SET VAR value`` → ``ack``, ``GET VAR`` → the value — on
        ``127.0.0.1:63352`` (Robotiq's own ``rq_*`` script functions use the
        same socket). It listens on the loopback only, so this sends one
        Primary program that opens the socket from *inside* the controller,
        talks to the daemon, and echoes the answers as ``textmsg`` markers.

        ``action``: ``status`` reads; ``open`` / ``close`` / ``move`` (to
        ``position`` 0 = fully open … 255 = fully closed) set speed, force and
        position, trigger the motion (``GTO``) and wait until the gripper
        reports it stopped (``OBJ`` ≠ 0: 1/2 = contact = an object is held,
        3 = at the requested position, nothing in the fingers); ``activate``
        (re)runs the activation cycle and waits for ``STA`` 3. Every call ends
        with the full status readback; ``ok`` is False when the daemon is
        missing, a fault (``FLT``) is set, or a move never settled.
        Protocol from Robotiq's socket example (dof.robotiq.com #2420).
        """
        if action not in GRIPPER_ACTIONS:
            raise ValueError(f"action must be one of {GRIPPER_ACTIONS}, got {action!r}")
        if action == "open":
            position = 0
        elif action == "close":
            position = 255
        elif action == "move":
            if position is None:
                raise ValueError("move needs position (0 = open … 255 = closed)")
        if position is not None and not (0 <= int(position) <= 255):
            raise ValueError("position must be within 0..255")
        if not (0 <= int(speed) <= 255) or not (0 <= int(force) <= 255):
            raise ValueError("speed and force must be within 0..255")
        timeout = max(0.5, min(float(timeout_s), 30.0))
        args = {"action": action, "speed": int(speed), "force": int(force), "timeout_s": timeout}
        if position is not None:
            args["position"] = int(position)
        if self.dry_run:
            return self._log("gripper", args, ok=True, result={"reply": "(dry-run)", "status": {}})

        sock = "urctl_rq"
        get = lambda var: (  # noqa: E731 — one GET round-trip echoed as a marker
            f'socket_send_line("GET {var}", "{sock}")\n'
            f'textmsg("urctl/rq/{var}=", socket_read_string("{sock}", timeout=2.0))\n'
        )
        set_ = lambda var, val: (  # noqa: E731
            f'socket_send_line("SET {var} {int(val)}", "{sock}")\n'
            f'urctl_rq_r = socket_read_string("{sock}", timeout=2.0)\n'
        )
        body = (
            f'if socket_open("{GRIPPER_DAEMON_HOST}", {GRIPPER_DAEMON_PORT}, "{sock}") == False:\n'
            f'  textmsg("urctl/rq/error=", "no gripper daemon on {GRIPPER_DAEMON_HOST}:{GRIPPER_DAEMON_PORT} '
            f'(is the Robotiq URCap installed and running?)")\n'
            "else:\n"
        )
        if action == "activate":
            body += "".join("  " + line for line in (set_("ACT", 0) + set_("ACT", 1)).splitlines(True))
            body += (
                "  urctl_rq_t = 0.0\n"
                f"  while (urctl_rq_t < {timeout:.1f}):\n"
                f'    socket_send_line("GET STA", "{sock}")\n'
                f'    urctl_rq_s = socket_read_string("{sock}", timeout=2.0)\n'
                '    if str_find(urctl_rq_s, "3") >= 0:\n'
                f"      urctl_rq_t = {timeout:.1f}\n"
                "    else:\n"
                "      sleep(0.2)\n"
                "      urctl_rq_t = urctl_rq_t + 0.2\n"
                "    end\n"
                "  end\n"
            )
        elif position is not None:
            moves = set_("SPE", speed) + set_("FOR", force) + set_("POS", position) + set_("GTO", 1)
            body += "".join("  " + line for line in moves.splitlines(True))
            body += (
                "  sleep(0.2)\n"
                "  urctl_rq_t = 0.0\n"
                f"  while (urctl_rq_t < {timeout:.1f}):\n"
                f'    socket_send_line("GET OBJ", "{sock}")\n'
                f'    urctl_rq_o = socket_read_string("{sock}", timeout=2.0)\n'
                '    if str_find(urctl_rq_o, "0") < 0:\n'
                f"      urctl_rq_t = {timeout:.1f}\n"
                "    else:\n"
                "      sleep(0.1)\n"
                "      urctl_rq_t = urctl_rq_t + 0.1\n"
                "    end\n"
                "  end\n"
            )
        body += "".join("  " + line for line in "".join(get(v) for v in GRIPPER_STATUS_VARS).splitlines(True))
        body += f'  socket_close("{sock}")\n  textmsg("urctl/rq/done=", "1")\nend\n'

        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_gripper",
            marker="urctl/rq",
            collect_for=timeout + 4.0,
            stop_marker="urctl/rq/done=",
        )
        status = _parse_gripper_status(captured)
        error = next(
            (line.split("urctl/rq/error=", 1)[1].strip() for line in captured if "urctl/rq/error=" in line),
            None,
        )
        done = any("urctl/rq/done=" in line for line in captured)
        result: dict = {"status": status, "captured": captured[-8:]}
        if error:
            result["error"] = error
        elif not done:
            result["error"] = "no reply from the gripper program on the Primary broadcast"
        elif status.get("FLT"):
            result["error"] = f"gripper fault FLT={status['FLT']}"
        elif status.get("ACT") == 0 or status.get("STA") not in (None, 3):
            result["error"] = (
                f"gripper not activated (ACT={status.get('ACT')}, STA={status.get('STA')}) — run activate"
            )
        obj = status.get("OBJ")
        if obj is not None:
            result["object_detected"] = obj in (1, 2)
            result["at_position"] = obj == 3
            result["moving"] = obj == 0
        if status.get("POS") is not None:
            result["opening_est_mm"] = round((255 - status["POS"]) / 255.0 * 50.0, 1)  # Hand-E: 50 mm stroke
        if position is not None and "error" not in result and obj == 0:
            result["error"] = f"the gripper was still moving after {timeout:.1f} s"
        return self._log("gripper", args, ok="error" not in result, result=result)

    # ----- RTDE writes -------------------------------------------------------

    def set_speed_override(self, fraction: float) -> dict:
        """Set the global speed slider (0-1) over RTDE. Scales the speed of all
        subsequent motion. Validated against the safety envelope's
        ``max_speed_fraction``; returns ``ok=False`` (sending nothing) when out
        of bounds, in dry-run, or when RTDE is unavailable.
        """
        args = {"fraction": fraction}
        verdict: SafetyVerdict = self.safety.validate_speed_override(fraction)
        if not verdict.ok:
            return self._log("set_speed_override", args, ok=False, safety=verdict.as_dict())
        if self.dry_run:
            return self._log(
                "set_speed_override",
                args,
                ok=True,
                safety=verdict.as_dict(),
                result={"reply": "(dry-run)"},
            )
        try:
            self._rtde_client().set_speed_slider(fraction)
        except OSError as exc:
            self._rtde = None
            return self._log(
                "set_speed_override",
                args,
                ok=False,
                safety=verdict.as_dict(),
                result={"error": f"RTDE unreachable: {exc}"},
            )
        except Exception as exc:  # RtdeError (e.g. slider already controlled)
            self._rtde = None
            return self._log(
                "set_speed_override",
                args,
                ok=False,
                safety=verdict.as_dict(),
                result={"error": str(exc)},
            )
        return self._log(
            "set_speed_override",
            args,
            ok=True,
            safety=verdict.as_dict(),
            result={"fraction": fraction},
        )

    def set_digital_output(self, pin: int, value: bool) -> dict:
        """Set a standard digital output pin (0-7) high/low over RTDE. A discrete
        IO toggle with no kinematic meaning — bounds-checked and audited, not
        run through the motion envelope (same posture as ``popup``)."""
        args = {"pin": pin, "value": value}
        if not isinstance(pin, int) or isinstance(pin, bool) or not 0 <= pin <= 7:
            return self._log("set_digital_output", args, ok=False, result={"error": "pin must be an int 0-7"})
        if self.dry_run:
            return self._log("set_digital_output", args, ok=True, result={"reply": "(dry-run)"})
        try:
            self._rtde_client().set_standard_digital_output(pin, bool(value))
        except OSError as exc:
            self._rtde = None
            return self._log(
                "set_digital_output", args, ok=False, result={"error": f"RTDE unreachable: {exc}"}
            )
        except Exception as exc:
            self._rtde = None
            return self._log("set_digital_output", args, ok=False, result={"error": str(exc)})
        return self._log("set_digital_output", args, ok=True, result={"pin": pin, "value": bool(value)})

    # ----- scripting ---------------------------------------------------------

    def run_script(
        self,
        script: str,
        *,
        wrap: bool = True,
        capture: bool = False,
        marker: str = "",
        collect_for: float = 2.0,
    ) -> dict:
        """Run arbitrary URScript on the Primary client.

        By default (``wrap=True``) the script is wrapped in a single ``def`` so
        the controller runs it as one program. This is almost always what you
        want: URControl treats each newline-terminated top-level statement as a
        separate program and kills the previous one, so a bare top-level
        ``movel(...)`` / ``movej(...)`` is accepted but **silently never runs**.
        Pass ``wrap=False`` for verbatim delivery (e.g. a script that defines
        its own top-level functions, which can't be nested inside another def).

        NOTE: arbitrary script bypasses the joint/speed envelope — there is no
        general way to statically bound what a script does. It is still audited.
        Prefer :meth:`move_joints` / :meth:`move_tcp` for motion you want validated.
        """
        args = {"script": script, "wrap": wrap, "capture": capture}
        if self.dry_run:
            return self._log("run_script", args, ok=True, result={"reply": "(dry-run)"})
        if capture:
            if wrap:
                captured = self.primary.run_and_capture(script, marker=marker, collect_for=collect_for)
            else:
                captured = self.primary.send_and_capture(script, marker=marker, collect_for=collect_for)
            return self._log("run_script", args, ok=True, result={"captured": captured})
        if wrap:
            self.primary.run(script)
        else:
            self.primary.send(script)
        return self._log("run_script", args, ok=True)

    def popup(self, text: str) -> dict:
        args = {"text": text}
        if self.dry_run:
            return self._log("popup", args, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.popup(text)
        return self._log("popup", args, ok=True, result={"reply": reply})

    def confirm_on_pendant(self, prompt: str, *, timeout: float = 120.0) -> dict:
        """Raise a Yes/No/Cancel dialog on the PolyScope pendant; block until answered.

        Built on ``request_boolean_from_primary_client``, which PolyScope's own
        UI answers — the operator taps the choice on the robot's screen (CLAUDE.md:
        these ``request_*_from_primary_client`` calls are "useful in wizards").
        The controller's dialog (``RequestDialogCreatorImpl.popupYesNoCancelDialog``)
        carries three buttons. This is the human-in-the-loop gate behind
        :class:`urctl.guided.GuidedSession`: announce intent on the robot, let
        the operator approve it there, then act.

        ``result['confirmed']`` is ``True`` (Yes), ``False`` (No), or ``None``.
        ``None`` covers both **Cancel** (the operator dismisses the request, which
        aborts the request program so no answer marker is sent) and a genuine
        timeout — callers that need to tell them apart should rely on a long
        ``timeout`` so a ``None`` means Cancel in practice. The Primary socket
        holds open until the answer (or marker timeout) lands, so a long
        ``timeout`` is normal: the operator may deliberate. The script gates the
        answer behind a textmsg marker we read back from the broadcast.
        """
        args = {"prompt": prompt, "timeout": timeout}
        if self.dry_run:
            return self._log(
                "confirm_on_pendant", args, ok=True, result={"confirmed": None, "reply": "(dry-run)"}
            )
        safe = _sanitize_prompt(prompt)
        body = (
            f'if (request_boolean_from_primary_client("{safe}")):\n'
            '  textmsg("urctl/confirm=yes")\n'
            "else:\n"
            '  textmsg("urctl/confirm=no")\n'
            "end\n"
        )
        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_confirm",
            marker="urctl/confirm=",
            collect_for=timeout,
            stop_marker="urctl/confirm=",
        )
        confirmed: bool | None = None
        if any("urctl/confirm=yes" in line for line in captured):
            confirmed = True
        elif any("urctl/confirm=no" in line for line in captured):
            confirmed = False
        if confirmed is None:
            # Timed out with no answer: the request dialog is still up on the
            # controller, blocking the next step. Abort the confirm program and
            # dismiss the popup so the session can continue. (On Yes/No the
            # program already ran to completion and ended itself.)
            try:
                self.dashboard.stop()
                self.dashboard.close_popup()
            except OSError:
                pass
        return self._log("confirm_on_pendant", args, ok=True, result={"confirmed": confirmed})

    def reteach_in_freedrive(self, prompt: str, *, timeout: float = 300.0) -> dict:
        """Drop into freedrive, raise a Yes/No/Cancel dialog, and capture the pose.

        Mirrors :meth:`confirm_on_pendant` but wraps the pendant hold in
        ``freedrive_mode()`` / ``end_freedrive_mode()`` so the operator can
        hand-guide the robot to the *exact* spot before pressing OK. The whole
        sequence is one Primary program: enable freedrive, block on the pendant
        request, disable freedrive, then ``textmsg`` the achieved joint pose —
        so the freedrive stays live for precisely the deliberation window and the
        pose we read back is wherever the operator left the arm.

        ``result['confirmed']`` is ``True`` (OK — pose accepted), ``False``
        (operator declined) or ``None`` (no answer within ``timeout``).
        ``result['joints']`` is the achieved 6-vector on OK, else ``None``. On a
        timeout the dangling freedrive + dialog are cleared so the session can go
        on (on OK/decline the program ends itself).
        """
        args = {"prompt": prompt, "timeout": timeout}
        if self.dry_run:
            return self._log(
                "reteach_in_freedrive", args, ok=True, result={"confirmed": None, "joints": None}
            )
        safe = _sanitize_prompt(prompt)
        body = (
            "freedrive_mode()\n"
            f'reteach_ok = request_boolean_from_primary_client("{safe}")\n'
            "end_freedrive_mode()\n"
            "if (reteach_ok):\n"
            '  textmsg("urctl/reteach/pose=", get_actual_joint_positions())\n'
            "else:\n"
            '  textmsg("urctl/reteach=cancel")\n'
            "end\n"
        )
        captured = self.primary.run_and_capture(
            body,
            fn_name="urctl_reteach",
            marker="urctl/reteach",
            collect_for=timeout,
            stop_marker="urctl/reteach",
        )
        from .primary import parse_vector

        joints = parse_vector(captured, "urctl/reteach/pose")
        confirmed: bool | None = None
        if joints is not None:
            confirmed = True
        elif any("urctl/reteach=cancel" in line for line in captured):
            confirmed = False
        if confirmed is None:
            # Timed out: the program is still blocked in freedrive with the
            # dialog up. Leave freedrive, kill the program, and dismiss the popup.
            try:
                self.freedrive(False)
                self.dashboard.stop()
                self.dashboard.close_popup()
            except OSError:
                pass
        return self._log(
            "reteach_in_freedrive", args, ok=True, result={"confirmed": confirmed, "joints": joints}
        )

    # ----- programs ----------------------------------------------------------

    def load_program(self, name: str) -> dict:
        args = {"name": name}
        if self.dry_run:
            return self._log("load_program", args, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.load(name)
        state = self.dashboard.program_state()
        ok = "Loading program" in reply
        return self._log("load_program", args, ok=ok, result={"reply": reply, "program_state": state})

    def play(self) -> dict:
        if self.dry_run:
            return self._log("play", {}, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.play()
        ok = "Starting program" in reply
        return self._log("play", {}, ok=ok, result={"reply": reply})

    def stop(self) -> dict:
        if self.dry_run:
            return self._log("stop", {}, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.stop()
        return self._log("stop", {}, ok=True, result={"reply": reply})

    def pause(self) -> dict:
        if self.dry_run:
            return self._log("pause", {}, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.pause()
        return self._log("pause", {}, ok=True, result={"reply": reply})

    def dashboard_command(self, command: str) -> dict:
        """Escape hatch: send a raw Dashboard command. Audited; unvalidated."""
        args = {"command": command}
        if self.dry_run:
            return self._log("dashboard_command", args, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.command(command)
        return self._log("dashboard_command", args, ok=True, result={"reply": reply})
