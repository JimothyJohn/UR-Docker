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

from .audit import AuditLog
from .config import RobotConfig
from .dashboard import DashboardClient
from .primary import PrimaryClient
from .robotapi import RobotAPIClient
from .safety import SafetyEnvelope, SafetyVerdict

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
        self.safety = safety or SafetyEnvelope()
        self.audit = audit or AuditLog()
        self.dry_run = dry_run
        # The orchestration client (power/brake/load/play/state). On PolyScope X
        # this is the REST Robot-API; on e-Series it is the Dashboard server.
        # Both expose the same method surface, so the rest of this class is
        # platform-agnostic. The attribute keeps the name ``dashboard`` for
        # continuity even though it may hold a RobotAPIClient.
        self.dashboard = (
            RobotAPIClient(self.config)
            if self.config.is_polyscopex()
            else DashboardClient(self.config)
        )
        self.primary = PrimaryClient(self.config)
        # RTDE (port 30004) is created lazily on first use so tests and code
        # paths that never touch it pay no socket.
        self._rtde = None

    def _rtde_client(self):
        """Lazily build (and cache) the RTDE client. Dropped on read failure so
        the next call reconnects (see :meth:`rtde_state` / :meth:`get_state`)."""
        if self._rtde is None:
            from .rtde import RtdeClient

            self._rtde = RtdeClient(self.config)
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
            state = self.primary.read_state(collect_for=collect_for)
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

    def rtde_state(self) -> dict:
        """High-rate structured state via RTDE (port 30004).

        Unlike :meth:`get_state`, this is RTDE-only and works even when the
        robot is not RUNNING — RTDE reflects the controller directly, not a
        URScript ``textmsg`` we injected. Returns joints/velocities, TCP
        pose/speed/force, and the raw ``safety_status``/``runtime_state`` bit
        words. Returns ``ok=False`` if RTDE is disabled or unreachable.
        """
        if self.dry_run:
            return self._log("rtde_state", {}, ok=True, result={"reply": "(dry-run)"})
        if not self.config.rtde_enabled:
            return self._log(
                "rtde_state", {}, ok=False, result={"error": "RTDE disabled (UR_RTDE_DISABLE)"}
            )
        try:
            raw = self._rtde_client().read_outputs()
        except OSError as exc:
            self._rtde = None
            return self._log(
                "rtde_state", {}, ok=False, result={"error": f"RTDE unreachable: {exc}"}
            )
        except Exception as exc:  # RtdeError and friends
            self._rtde = None
            return self._log("rtde_state", {}, ok=False, result={"error": str(exc)})
        result = {
            "joints": raw.get("actual_q"),
            "joint_velocities": raw.get("actual_qd"),
            "tcp": raw.get("actual_TCP_pose"),
            "tcp_speed": raw.get("actual_TCP_speed"),
            "tcp_force": raw.get("actual_TCP_force"),
            "safety_status": raw.get("safety_status_bits"),
            "runtime_state": raw.get("runtime_state"),
            "rtde_robot_mode": raw.get("robot_mode"),
            "timestamp": raw.get("timestamp"),
        }
        return self._log("rtde_state", {}, ok=True, result=result)

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
            return self._log(
                "move_joints", args, ok=True, safety=verdict.as_dict(), result={"waited": False}
            )

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
        return self._log("move_joints", args, ok=ok, safety=verdict.as_dict(), result=result)

    def move_home(self, **kwargs) -> dict:
        return self.move_joints(HOME_JOINTS, **kwargs)

    def move_tcp(
        self,
        pose: list[float],
        *,
        relative: bool = False,
        velocity: float = DEFAULT_TCP_VELOCITY,
        acceleration: float = DEFAULT_TCP_ACCELERATION,
        wait: bool = True,
        timeout: float = 30.0,
    ) -> dict:
        """Linear Cartesian move (``movel``), after safety validation.

        ``pose`` is ``[x, y, z, rx, ry, rz]`` (metres + rotation-vector radians).
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
        verdict: SafetyVerdict = self.safety.validate_move_tcp(
            pose,
            velocity=velocity,
            acceleration=acceleration,
            relative=relative,
            robot_mode=None if self.dry_run else self.dashboard.robot_mode(),
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
        target_expr = (
            f"pose_add(get_actual_tcp_pose(), {pose_literal})" if relative else pose_literal
        )
        body = (
            f"movel({target_expr}, a={acceleration}, v={velocity})\n"
            "sync()\n"
            'textmsg("urctl/move/done=", get_actual_tcp_pose())\n'
        )
        if not wait:
            self.primary.run(body, fn_name="urctl_move_tcp")
            return self._log(
                "move_tcp", args, ok=True, safety=verdict.as_dict(), result={"waited": False}
            )

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
        return self._log("move_tcp", args, ok=ok, safety=verdict.as_dict(), result=result)

    def freedrive(self, enable: bool) -> dict:
        """Enable/disable freedrive (hand-guiding). All six axes in base frame."""
        args = {"enable": enable}
        if self.dry_run:
            return self._log("freedrive", args, ok=True, result={"reply": "(dry-run)"})
        body = "freedrive_mode()\n" if enable else "end_freedrive_mode()\n"
        self.primary.run(body, fn_name="urctl_freedrive")
        return self._log("freedrive", args, ok=True)

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
            return self._log(
                "set_digital_output", args, ok=False, result={"error": "pin must be an int 0-7"}
            )
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
        return self._log(
            "set_digital_output", args, ok=True, result={"pin": pin, "value": bool(value)}
        )

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
                captured = self.primary.run_and_capture(
                    script, marker=marker, collect_for=collect_for
                )
            else:
                captured = self.primary.send_and_capture(
                    script, marker=marker, collect_for=collect_for
                )
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

    # ----- programs ----------------------------------------------------------

    def load_program(self, name: str) -> dict:
        args = {"name": name}
        if self.dry_run:
            return self._log("load_program", args, ok=True, result={"reply": "(dry-run)"})
        reply = self.dashboard.load(name)
        state = self.dashboard.program_state()
        ok = "Loading program" in reply
        return self._log(
            "load_program", args, ok=ok, result={"reply": reply, "program_state": state}
        )

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
