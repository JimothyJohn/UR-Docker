"""urctl — a small, framework-neutral toolkit for controlling Universal Robots
e-Series controllers over the network.

It speaks the same backend protocols a real controller exposes (Dashboard on
29999, Primary Client on 30001) so the *same code* drives this repo's URSim
container and a physical robot on the factory network — the only thing that
changes is the connection target (see :class:`urctl.config.RobotConfig`).

Layers, lowest to highest:

  config      RobotConfig — where the robot is (host/ports), from env or args.
  dashboard   DashboardClient — the line-oriented "press buttons" surface.
  primary     PrimaryClient — stream URScript that runs immediately; read state.
  safety      SafetyEnvelope — pre-execution validation (joint/speed/state).
  audit       AuditLog — structured JSON record of every action taken.
  robot       Robot — the high-level facade humans and agents actually use.
  tools       A JSON-schema'd tool registry for agent frameworks (OpenClaw,
              ROSClaw, MCP, plain function-calling).

The `tools` layer is what makes this "agent friendly": capabilities are
discoverable as schemas, every call is validated against the safety envelope
before it reaches the robot, and every call is recorded in the audit log —
the contract ROSClaw-style executives expect from a robot backend.
"""

from __future__ import annotations

from .audit import AuditLog, AuditRecord
from .config import RobotConfig
from .guided import GuidedSession, LiveReloader, StepResult, docker_placer, local_dir_placer
from .robot import Robot
from .rtde import RtdeClient, RtdeError
from .safety import SafetyEnvelope, SafetyVerdict, SafetyViolation
from .urp_builder import UrpProgram, Waypoint

__version__ = "0.1.0"

__all__ = [
    "RobotConfig",
    "Robot",
    "SafetyEnvelope",
    "SafetyViolation",
    "SafetyVerdict",
    "AuditLog",
    "AuditRecord",
    "RtdeClient",
    "RtdeError",
    "UrpProgram",
    "Waypoint",
    "GuidedSession",
    "StepResult",
    "LiveReloader",
    "docker_placer",
    "local_dir_placer",
    "__version__",
]
