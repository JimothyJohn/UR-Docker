"""Connection configuration — *where* the robot is.

The whole point of this module: nothing else in the toolkit hardcodes a host
or port. In development the defaults point at this repo's URSim container on
``localhost``; in production you point the same code at a robot's IP address::

    # development (URSim container, ports published on localhost)
    cfg = RobotConfig.from_env()

    # a real robot on the plant network
    cfg = RobotConfig(host="10.0.0.5")
    cfg = RobotConfig.from_env(host="10.0.0.5")

    # or purely from the environment, no code change
    UR_HOST=10.0.0.5 python -m urctl state

Every field can be overridden by an environment variable so the same binary
works against the simulator and real hardware without recompilation — the
"swapping robot platforms is a configuration change" property OpenClaw/ROSClaw
rely on.
"""

from __future__ import annotations

import os
from dataclasses import dataclass

DEFAULT_HOST = "localhost"

# The backend ports a UR e-Series controller exposes. Documented in CLAUDE.md;
# duplicated here as defaults so callers never have to remember the numbers.
DEFAULT_DASHBOARD_PORT = 29999
DEFAULT_PRIMARY_PORT = 30001
DEFAULT_SECONDARY_PORT = 30002
DEFAULT_RT_PORT = 30003
DEFAULT_RTDE_PORT = 30004
DEFAULT_INTERPRETER_PORT = 30020

# Controller software platform — decides *which orchestration surface* the
# toolkit talks to. "e-series" (PolyScope 5) uses the line-oriented Dashboard
# server on 29999; "polyscopex" (PolyScope X / PolyScope 10) dropped Dashboard
# in favour of the REST "Robot-API" served over HTTP. Either way, URScript
# execution still rides the Primary client on 30001. See CLAUDE.md.
PLATFORM_E_SERIES = "e-series"
PLATFORM_POLYSCOPEX = "polyscopex"
DEFAULT_PLATFORM = PLATFORM_E_SERIES

# The PolyScope X Robot-API is reached over plain HTTP. On a real robot it is
# port 80; this repo's URSim PolyScope X container publishes it on host 8000.
# The path prefix every Robot-API resource lives under is fixed by the firmware.
DEFAULT_ROBOT_API_PORT = 80
DEFAULT_ROBOT_API_BASE_PATH = "/universal-robots/robot-api"

DEFAULT_TIMEOUT_S = 10.0


def _env_int(name: str, default: int) -> int:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return int(raw)
    except ValueError as exc:  # surface a clear message rather than a stack trace later
        raise ValueError(f"environment variable {name}={raw!r} is not an integer") from exc


def _env_float(name: str, default: float) -> float:
    raw = os.environ.get(name)
    if raw is None or raw == "":
        return default
    try:
        return float(raw)
    except ValueError as exc:
        raise ValueError(f"environment variable {name}={raw!r} is not a number") from exc


def _env_flag(name: str) -> bool:
    """True when ``name`` is set to a truthy value (1/true/yes/on)."""
    return os.environ.get(name, "").strip().lower() in ("1", "true", "yes", "on")


@dataclass(frozen=True)
class RobotConfig:
    """Immutable description of how to reach a controller.

    Construct directly for explicit control, or via :meth:`from_env` to pick up
    ``UR_HOST`` / ``UR_*_PORT`` / ``UR_TIMEOUT_S`` from the environment.
    """

    host: str = DEFAULT_HOST
    dashboard_port: int = DEFAULT_DASHBOARD_PORT
    primary_port: int = DEFAULT_PRIMARY_PORT
    secondary_port: int = DEFAULT_SECONDARY_PORT
    rt_port: int = DEFAULT_RT_PORT
    rtde_port: int = DEFAULT_RTDE_PORT
    interpreter_port: int = DEFAULT_INTERPRETER_PORT
    timeout: float = DEFAULT_TIMEOUT_S
    # When False (env ``UR_RTDE_DISABLE``), Robot.get_state() skips its RTDE
    # upgrade and uses the legacy Primary textmsg path. A kill-switch for
    # controllers where another client already owns the RTDE recipe.
    rtde_enabled: bool = True
    # Orchestration platform: PLATFORM_E_SERIES (Dashboard) or
    # PLATFORM_POLYSCOPEX (Robot-API REST). Picks the orchestration client the
    # Robot facade sits on; Primary URScript execution is the same either way.
    platform: str = DEFAULT_PLATFORM
    robot_api_port: int = DEFAULT_ROBOT_API_PORT
    robot_api_base_path: str = DEFAULT_ROBOT_API_BASE_PATH

    @classmethod
    def from_env(cls, host: str | None = None, **overrides) -> RobotConfig:
        """Build a config from environment variables, with optional overrides.

        Precedence (highest first): explicit ``host=`` / ``**overrides`` kwargs,
        then environment variables, then the module defaults. This lets a CLI
        flag win over ``UR_HOST`` while still falling back to it.
        """
        values: dict[str, object] = {
            "host": host or os.environ.get("UR_HOST", DEFAULT_HOST),
            "dashboard_port": _env_int("UR_DASH_PORT", DEFAULT_DASHBOARD_PORT),
            "primary_port": _env_int("UR_PRIMARY_PORT", DEFAULT_PRIMARY_PORT),
            "secondary_port": _env_int("UR_SECONDARY_PORT", DEFAULT_SECONDARY_PORT),
            "rt_port": _env_int("UR_RT_PORT", DEFAULT_RT_PORT),
            "rtde_port": _env_int("UR_RTDE_PORT", DEFAULT_RTDE_PORT),
            "interpreter_port": _env_int("UR_INTERPRETER_PORT", DEFAULT_INTERPRETER_PORT),
            "timeout": _env_float("UR_TIMEOUT_S", DEFAULT_TIMEOUT_S),
            "rtde_enabled": not _env_flag("UR_RTDE_DISABLE"),
            "platform": os.environ.get("UR_PLATFORM", DEFAULT_PLATFORM),
            "robot_api_port": _env_int("UR_ROBOT_API_PORT", DEFAULT_ROBOT_API_PORT),
            "robot_api_base_path": os.environ.get("UR_ROBOT_API_BASE_PATH", DEFAULT_ROBOT_API_BASE_PATH),
        }
        values.update(overrides)
        return cls(**values)  # type: ignore[arg-type]

    @property
    def robot_api_url(self) -> str:
        """Base URL for the PolyScope X Robot-API, e.g.
        ``http://localhost:8000/universal-robots/robot-api``. Resources hang off
        this (``/robotstate/v1/robotmode``, ``/program/v1/state``, …)."""
        return f"http://{self.host}:{self.robot_api_port}{self.robot_api_base_path}"

    def is_polyscopex(self) -> bool:
        """True when targeting a PolyScope X / PolyScope 10 controller."""
        return self.platform == PLATFORM_POLYSCOPEX

    def is_loopback(self) -> bool:
        """True when pointed at the local machine (URSim dev container)."""
        return self.host in ("localhost", "127.0.0.1", "::1")
