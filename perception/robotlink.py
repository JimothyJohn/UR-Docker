"""The cockpit's bridge to a UR controller.

Every robot action goes through :func:`urctl.tools.call_tool` — the same
schema-validated, safety-enveloped, audited path the ``urctl`` CLI, the MCP
server and ``urctl gui`` use — so "send this point to the robot" from the
perception cockpit is one more tool caller, not a side door. Two calls:

* :meth:`RobotLink.locate` — read the flange pose (``ur_flange_pose``) and
  turn the segmented object's camera-frame point into a base-frame point and
  an approach pose (:func:`perception.handeye.locate`). No motion.
* :meth:`RobotLink.move` — ``ur_move_tcp`` to an absolute base-frame pose (the
  approach pose the operator just saw). Validated by the safety envelope;
  refused in a non-RUNNING state, over the speed caps, or outside reach.

Connection target is ``RobotConfig.from_env()`` (``UR_HOST`` etc.); nothing
connects until the first call. ``dry_run`` validates and audits without
sending, and ``ur_flange_pose`` then returns a stand-in pose so the whole
flow can be exercised on the synthetic camera without a controller.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence

from urctl.config import RobotConfig
from urctl.robot import Robot
from urctl.tools import ToolError, call_tool

from .calibrate import CalibrationSession
from .handeye import DEFAULT_STANDOFF_M, HandEye, default_handeye_path, locate

DEFAULT_APPROACH_VELOCITY = 0.1  # m/s — slow; this move follows a single click
DEFAULT_APPROACH_ACCELERATION = 0.3  # m/s^2
# A jog is one button press: cap the step so a mistyped unit can't send the arm
# across the cell (the safety envelope's own relative cap is 1.0 m).
MAX_JOG_STEP_M = 0.05
MAX_JOG_STEP_RAD = 0.35
DEFAULT_JOG_VELOCITY = 0.05


class RobotLink:
    def __init__(
        self,
        config: RobotConfig | None = None,
        *,
        dry_run: bool = False,
        handeye: HandEye | None = None,
        robot: Robot | None = None,
    ):
        self.config = config or RobotConfig.from_env()
        self.dry_run = dry_run
        self.robot = robot if robot is not None else Robot(self.config, dry_run=dry_run)
        self.handeye = handeye or HandEye.from_env()
        self.calibration = CalibrationSession(seed=self.handeye)
        self._extrinsics: Mapping | None = None

    def attach_camera(self, camera_description: Mapping) -> None:
        """Take the depth→colour extrinsics from ``camera.describe()``."""
        self._extrinsics = camera_description.get("extrinsics_depth_to_color")
        self.handeye = self.handeye.with_extrinsics(self._extrinsics)
        self.calibration.extrinsics = self._extrinsics
        self.calibration.seed = self.handeye

    def describe(self) -> dict:
        return {
            "host": self.config.host,
            "platform": self.config.platform,
            "dry_run": self.dry_run,
            "handeye": self.handeye.as_dict(),
            "approach": {
                "standoff_m": DEFAULT_STANDOFF_M,
                "velocity": DEFAULT_APPROACH_VELOCITY,
                "acceleration": DEFAULT_APPROACH_ACCELERATION,
            },
        }

    def state(self) -> dict:
        return self._tool("ur_get_state")

    def flange_pose(self) -> dict:
        return self._tool("ur_flange_pose")

    def locate(self, point_cam: Sequence[float], *, standoff_m: float = DEFAULT_STANDOFF_M) -> dict:
        """Camera point → base point + approach pose, using the live flange pose."""
        fp = self.flange_pose()
        if not fp.get("ok") or not fp.get("flange") or not fp.get("tcp"):
            return {"ok": False, "error": fp.get("error") or "could not read the flange pose", "robot": fp}
        result = locate(self.handeye, fp["flange"], point_cam, tcp_pose=fp["tcp"], standoff_m=standoff_m)
        result["ok"] = True
        result["robot"] = {
            k: fp.get(k)
            for k in ("host", "dry_run", "tcp_offset", "flange_reported", "host_controller_mismatch_m", "ts")
        }
        return result

    def move(
        self,
        pose: Sequence[float],
        *,
        velocity: float = DEFAULT_APPROACH_VELOCITY,
        acceleration: float = DEFAULT_APPROACH_ACCELERATION,
    ) -> dict:
        """``movel`` to an absolute base-frame pose (safety-validated, audited)."""
        vals = [float(v) for v in pose]
        if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
            raise ValueError("pose must be 6 finite numbers [x, y, z, rx, ry, rz]")
        if not (0.0 < velocity <= 1.0) or not (0.0 < acceleration <= 5.0):
            raise ValueError("velocity must be within (0, 1] m/s and acceleration within (0, 5] m/s^2")
        return self._tool(
            "ur_move_tcp",
            {
                "pose": vals,
                "relative": False,
                "velocity": float(velocity),
                "acceleration": float(acceleration),
            },
        )

    # -- hand-eye calibration (touch-and-click; perception.calibrate) --------------

    def cal_record_mark(self) -> dict:
        """The live TCP position is the mark in the base frame (the tip is on it)."""
        fp = self.flange_pose()
        if not fp.get("ok") or not fp.get("tcp"):
            return {"ok": False, "error": fp.get("error") or "could not read the TCP pose", "robot": fp}
        out = self.calibration.record_mark(fp["tcp"][:3])
        out["ok"] = True
        return out

    def cal_add_view(self, point_cam: Sequence[float], *, pixel=None, seq=None) -> dict:
        fp = self.flange_pose()
        if not fp.get("ok") or not fp.get("flange"):
            return {"ok": False, "error": fp.get("error") or "could not read the flange pose", "robot": fp}
        out = self.calibration.add_view(fp["flange"], point_cam, pixel=pixel, seq=seq)
        out["ok"] = True
        return out

    def cal_solve(self) -> dict:
        return self.calibration.solve()

    def cal_apply(self, *, save: bool = True, path: str | None = None, force: bool = False) -> dict:
        """Use the solved transform from now on (and write it so the next start
        picks it up: env > file > bracket seed). A solve that carries warnings
        (poor rotation diversity, high residual) is refused unless ``force``."""
        result = self.calibration.result
        if result and result.get("warnings") and not force:
            return {
                "ok": False,
                "error": "solve has warnings; fix them or pass force=true: " + "; ".join(result["warnings"]),
                "warnings": result["warnings"],
            }
        self.handeye = self.calibration.handeye()
        out: dict = {"ok": True, "handeye": self.handeye.as_dict()}
        if save:
            saved = self.calibration.save(path or default_handeye_path())
            out["saved"] = str(saved)
        return out

    def cal_status(self) -> dict:
        d = self.calibration.as_dict()
        d["ok"] = True
        d["active_handeye"] = self.handeye.as_dict()
        return d

    # -- pilot actions (each one tool call; all safety-enveloped + audited) ----

    def jog(self, delta: Sequence[float], *, velocity: float = DEFAULT_JOG_VELOCITY) -> dict:
        """One relative base-frame nudge: ``[dx, dy, dz, drx, dry, drz]``,
        each translation ≤ :data:`MAX_JOG_STEP_M`, each rotation ≤ :data:`MAX_JOG_STEP_RAD`."""
        vals = [float(v) for v in delta]
        if len(vals) != 6 or not all(math.isfinite(v) for v in vals):
            raise ValueError("delta must be 6 finite numbers [dx, dy, dz, drx, dry, drz]")
        if any(abs(v) > MAX_JOG_STEP_M for v in vals[:3]):
            raise ValueError(f"a jog moves at most {MAX_JOG_STEP_M} m per axis (got {vals[:3]})")
        if any(abs(v) > MAX_JOG_STEP_RAD for v in vals[3:]):
            raise ValueError(f"a jog rotates at most {MAX_JOG_STEP_RAD} rad per axis (got {vals[3:]})")
        if not any(vals):
            raise ValueError("zero jog")
        if not (0.0 < velocity <= 0.25):
            raise ValueError("jog velocity must be within (0, 0.25] m/s")
        return self._tool(
            "ur_move_tcp",
            {"pose": vals, "relative": True, "velocity": float(velocity), "acceleration": 0.3},
        )

    def bring_up(self) -> dict:
        return self._tool("ur_bring_up")

    def stop(self) -> dict:
        return self._tool("ur_stop")

    def freedrive(self, enable: bool) -> dict:
        return self._tool("ur_freedrive", {"enable": bool(enable)})

    def rtde_state(self, deep: bool = False) -> dict:
        return self._tool("ur_rtde_state", {"deep": bool(deep)})

    def _tool(self, name: str, params: dict | None = None) -> dict:
        try:
            return call_tool(self.robot, name, params)
        except ToolError as exc:
            return {"ok": False, "error": str(exc)}
        except OSError as exc:
            return {"ok": False, "error": f"robot unreachable at {self.config.host}: {exc}"}
