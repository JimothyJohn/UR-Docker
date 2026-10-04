"""The vendor-neutral surfaces the rest of the toolkit programs against.

Two structural interfaces (``typing.Protocol``, so nothing has to inherit):

* :class:`Controller` — what the tool registry, the cockpit's ``RobotLink``, the
  pick cycle and the calibration drive: state, bring-up, joint / Cartesian /
  path / trajectory motion, stop, freedrive, program load/play/pause, a native
  script. :class:`urctl.robot.Robot` is the UR implementation (e-Series over
  Dashboard + Primary, PolyScope X over the Robot-API + Primary — one class,
  selected by ``RobotConfig.platform``). :class:`urctl.ufactory.UFactoryArm` is
  the second vendor (UFACTORY 850 / xArm / Lite 6, ``platform="ufactory"``);
  :func:`make_controller` picks between them. ``isinstance(x, Controller)`` checks it.
* :class:`Gripper` — status / open / close / move / activate with one result
  shape. :class:`RobotiqUrcapGripper` adapts :meth:`Robot.gripper` (the Robotiq
  URCap daemon on the controller's loopback, driven from URScript).

The protocols are the contract; the result dicts stay the ones the tools and
the cockpit already read (``ok``, ``landed``, ``protective_stop``, ``status``,
``object_detected`` …), so this adds a seam, not a translation layer.
"""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING, Protocol, runtime_checkable

if TYPE_CHECKING:
    from .robot import Robot


@runtime_checkable
class Controller(Protocol):
    """A robot arm's controller, as the toolkit drives one."""

    # -- observation --------------------------------------------------------------------
    def get_state(self) -> dict: ...
    def get_flange_pose(self) -> dict: ...
    def get_tcp_offset(self) -> dict: ...
    def set_tcp_offset(self, offset: list[float], **kwargs) -> dict: ...

    # -- power / recovery -------------------------------------------------------------------
    def bring_up(self) -> dict: ...
    def power_off(self) -> dict: ...
    def stop(self) -> dict: ...

    # -- motion (validated by the implementation's safety envelope) -------------------------
    def move_joints(self, joints: list[float], **kwargs) -> dict: ...
    def move_tcp(self, pose: list[float], **kwargs) -> dict: ...
    def move_tcp_path(self, legs: Sequence[dict], **kwargs) -> dict: ...
    def move_trajectory(self, waypoints: Sequence[Sequence[float]], **kwargs) -> dict: ...
    def freedrive(self, enable: bool, **kwargs) -> dict: ...

    # -- programs / native code ----------------------------------------------------------------
    def load_program(self, name: str) -> dict: ...
    def play(self) -> dict: ...
    def pause(self) -> dict: ...
    def run_script(self, script: str, **kwargs) -> dict: ...

    def close(self) -> None: ...


@runtime_checkable
class Gripper(Protocol):
    """An end effector that opens, closes and reports what it holds.

    Every method returns a dict with ``ok`` and, where the hardware reports it,
    ``object_detected`` (True when the fingers stopped on something) and a
    ``status`` dict of raw device fields."""

    def status(self) -> dict: ...
    def open(self, **kwargs) -> dict: ...
    def close(self, **kwargs) -> dict: ...
    def move(self, position: int, **kwargs) -> dict: ...
    def activate(self) -> dict: ...


class RobotiqUrcapGripper:
    """:class:`Gripper` over :meth:`urctl.robot.Robot.gripper` — a Robotiq Hand-E /
    2F through its URCap daemon (``127.0.0.1:63352`` on the controller)."""

    def __init__(self, robot: Robot, *, speed: int = 255, force: int = 100):
        self.robot = robot
        self.speed = speed
        self.force = force

    def _run(self, action: str, **kwargs) -> dict:
        params = {"speed": self.speed, "force": self.force, **kwargs}
        return self.robot.gripper(action, **params)

    def status(self) -> dict:
        return self.robot.gripper("status")

    def open(self, **kwargs) -> dict:
        return self._run("open", **kwargs)

    def close(self, **kwargs) -> dict:
        return self._run("close", **kwargs)

    def move(self, position: int, **kwargs) -> dict:
        return self._run("move", position=int(position), **kwargs)

    def activate(self) -> dict:
        return self.robot.gripper("activate")


def make_controller(config=None, **kwargs) -> Controller:
    """The controller ``config.platform`` names: a :class:`urctl.ufactory.UFactoryArm`
    for ``ufactory``, a UR :class:`urctl.robot.Robot` otherwise. ``kwargs``
    (``dry_run``, ``safety``, ``audit``) go to the constructor."""
    from .config import RobotConfig

    config = config or RobotConfig.from_env()
    if config.is_ufactory():
        from .ufactory import UFactoryArm

        return UFactoryArm(config, **kwargs)
    from .robot import Robot

    return Robot(config, **kwargs)


__all__ = ["Controller", "Gripper", "RobotiqUrcapGripper", "make_controller"]
