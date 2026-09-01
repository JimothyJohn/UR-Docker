"""System-level introspection — read *everything else* about a controller.

The network surfaces (Dashboard/Primary/RTDE) expose live state, but a UR
controller is also a small Debian machine whose filesystem holds the facts the
network APIs never mention: joint serial numbers and firmware generations,
whether the kinematic calibration still matches the installed joints, what
programs and installations exist, how full the eMMC is, what the controller
logged at boot. This module reads that layer.

Access is through a *runner* — the same pattern as :mod:`urctl.guided`'s
placers, in reverse:

  * :class:`SshRunner` — a real e-Series (sshd on :22 is the only file surface
    a real robot has; enroll a key or set ``SSHPASS``).
  * :class:`DockerRunner` — this repo's URSim container (no sshd; ``docker
    exec`` instead).

Every collector degrades gracefully: a file that doesn't exist on one platform
(URSim has no ``joint_info.txt`` — emulated joints) yields an ``"error"`` note
in that section rather than failing the snapshot.

    from urctl.sysinfo import SshRunner, SystemInspector
    insp = SystemInspector(SshRunner("192.168.1.50"))
    insp.snapshot()          # the whole cell model
    insp.joints()            # serials, firmware, calibration-mismatch flag
    insp.installation()      # parsed active default.installation
"""

from __future__ import annotations

import base64
import configparser
import re
import subprocess
from dataclasses import dataclass, field

from .installation import parse_installation

__all__ = [
    "RunnerError",
    "SshRunner",
    "DockerRunner",
    "SystemInspector",
    "parse_joint_info",
    "parse_df",
    "parse_program_listing",
]


class RunnerError(Exception):
    """A runner command failed (non-zero exit, timeout, or unreachable host)."""


# Filesystem layout differs between a real controller and the URSim container.
_REAL_PATHS = {
    "program_dir": "/programs",
    "urcontrol_dir": "/root/.urcontrol",
    "urcontrol_log": "/tmp/log/urcontrol/current",
    "polyscope_log": "/root/polyscope.log",
    "log_history": "/root/log_history.txt",
    "urcaps_dir": "/root/.urcaps",
    "flightreports_dir": "/root/flightreports",
}
_URSIM_PATHS = {
    "program_dir": "/ursim/programs",
    "urcontrol_dir": "/ursim/.urcontrol",
    "urcontrol_log": "/ursim/URControl.log",
    "polyscope_log": "/ursim/polyscope.log",
    "log_history": "/ursim/log_history.txt",
    "urcaps_dir": "/ursim/.urcaps",
    "flightreports_dir": "/ursim/flightreports",
}


@dataclass
class SshRunner:
    """Run commands on a real e-Series controller over SSH.

    Uses ``BatchMode`` so a missing key fails fast instead of hanging on a
    password prompt; if ``SSHPASS`` is set, ``sshpass -e`` is used instead
    (mirroring :func:`urctl.guided.scp_placer`).
    """

    host: str
    user: str = "root"
    port: int = 22
    timeout: float = 15.0
    paths: dict = field(default_factory=lambda: dict(_REAL_PATHS))

    @property
    def label(self) -> str:
        return f"ssh:{self.user}@{self.host}"

    def _base_cmd(self) -> list[str]:
        import os

        ssh = [
            "ssh",
            "-p",
            str(self.port),
            "-o",
            "StrictHostKeyChecking=accept-new",
            "-o",
            f"ConnectTimeout={max(1, int(self.timeout))}",
        ]
        if os.environ.get("SSHPASS"):
            return ["sshpass", "-e", *ssh]
        return [*ssh, "-o", "BatchMode=yes"]

    def run(self, command: str) -> str:
        proc = subprocess.run(
            [*self._base_cmd(), f"{self.user}@{self.host}", command],
            capture_output=True,
            text=True,
            timeout=self.timeout + 30,
        )
        if proc.returncode != 0:
            raise RunnerError(
                f"{self.label}: {command!r} exited {proc.returncode}: {proc.stderr.strip()[:200]}"
            )
        return proc.stdout


@dataclass
class DockerRunner:
    """Run commands inside this repo's URSim container via ``docker exec``."""

    container: str = "ur-docker-ursim-1"
    timeout: float = 15.0
    paths: dict = field(default_factory=lambda: dict(_URSIM_PATHS))

    @property
    def label(self) -> str:
        return f"docker:{self.container}"

    def run(self, command: str) -> str:
        proc = subprocess.run(
            ["docker", "exec", self.container, "sh", "-c", command],
            capture_output=True,
            text=True,
            timeout=self.timeout + 30,
        )
        if proc.returncode != 0:
            raise RunnerError(
                f"{self.label}: {command!r} exited {proc.returncode}: {proc.stderr.strip()[:200]}"
            )
        return proc.stdout


# ----- pure parsers (unit-testable without a robot) ---------------------------


def parse_joint_info(text: str) -> list[dict]:
    """Parse ``/root/.urcontrol/joint_info.txt`` (INI-ish, one section per
    joint) into ``[{joint, serial, firmware}, ...]``.

    The decisive signal: a joint whose serial prefix or selftest firmware
    differs from the majority is a *replacement part* — pair that with the
    calibration-mismatch flag to tell whether the arm needs recalibration.
    """
    cp = configparser.ConfigParser()
    cp.read_string(text)
    joints = []
    for section in cp.sections():
        m = re.match(r"joint (\d+)", section)
        if not m:
            continue
        joints.append(
            {
                "joint": int(m.group(1)),
                "serial": cp.get(section, "serial", fallback=None),
                "firmware": cp.get(section, "selftest_version_uA", fallback=None),
            }
        )
    joints.sort(key=lambda j: j["joint"])
    # Flag minority serial-prefix/firmware joints as likely replacements.
    if len(joints) >= 3:
        fw_counts: dict[str, int] = {}
        for j in joints:
            fw = j.get("firmware") or ""
            fw_counts[fw] = fw_counts.get(fw, 0) + 1
        majority_fw = max(fw_counts, key=lambda k: fw_counts[k])
        for j in joints:
            j["replacement_suspected"] = bool(j.get("firmware") and j["firmware"] != majority_fw)
    return joints


def parse_df(text: str) -> dict | None:
    """Parse ``df -k <mount>`` output (the root filesystem line)."""
    for line in text.splitlines()[1:]:
        parts = line.split()
        if len(parts) >= 6:
            try:
                total_kb, used_kb, avail_kb = int(parts[1]), int(parts[2]), int(parts[3])
            except ValueError:
                continue
            return {
                "filesystem": parts[0],
                "total_mb": round(total_kb / 1024),
                "used_mb": round(used_kb / 1024),
                "available_mb": round(avail_kb / 1024),
                # df's Use% formula: used / (used + available) — the difference
                # from used/total is the ext4 reserved blocks.
                "used_percent": round(100 * used_kb / (used_kb + avail_kb)) if (used_kb + avail_kb) else None,
                "mount": parts[5],
            }
    return None


def parse_program_listing(text: str) -> list[dict]:
    """Parse the ``find``-based program listing (``<epoch> <size> <path>``)."""
    out = []
    for line in text.splitlines():
        parts = line.split(maxsplit=2)
        if len(parts) != 3:
            continue
        try:
            mtime, size = int(float(parts[0])), int(parts[1])
        except ValueError:
            continue
        path = parts[2]
        name = path.rsplit("/", 1)[-1]
        kind = name.rsplit(".", 1)[-1] if "." in name else ""
        out.append({"name": name, "path": path, "kind": kind, "size": size, "mtime": mtime})
    out.sort(key=lambda p: p["name"].lower())
    return out


def runner_for(config=None, *, access: str | None = None, target: str | None = None):
    """Build the right runner for a :class:`~urctl.config.RobotConfig`.

    The inference rule mirrors how this repo actually reaches controllers:
    a loopback host is the URSim container (``docker exec``), anything else is
    a real robot (SSH as root). ``access`` ("ssh"/"docker") and ``target``
    (hostname / ``user@host`` / container name) override the inference.
    """
    from .config import RobotConfig

    config = config or RobotConfig.from_env()
    if access is None:
        access = "docker" if config.is_loopback() else "ssh"
    if access == "docker":
        return DockerRunner(target) if target else DockerRunner()
    if access == "ssh":
        user, _, host = (target or "").rpartition("@")
        return SshRunner(host or config.host, user=user or "root")
    raise ValueError(f"unknown access mode {access!r} (expected 'ssh' or 'docker')")


# ----- the inspector ----------------------------------------------------------


class SystemInspector:
    """Filesystem-level cell model of one controller, via a runner."""

    def __init__(self, runner):
        self.runner = runner
        self.paths = runner.paths

    # Each collector returns a dict; on failure it returns {"error": ...} so a
    # snapshot is always complete even when sections are unavailable.

    def _guard(self, fn) -> dict:
        try:
            return fn()
        except (RunnerError, subprocess.TimeoutExpired, OSError) as exc:
            return {"error": str(exc)}
        except Exception as exc:  # parse failures on unexpected content
            return {"error": f"{type(exc).__name__}: {exc}"}

    def read_file(self, path: str) -> bytes:
        """Fetch one file's raw bytes (base64 transport survives any shell)."""
        return base64.b64decode(self.runner.run(f"base64 < '{path}'"))

    # -- identity --------------------------------------------------------------

    def identity(self) -> dict:
        def collect():
            urc = self.paths["urcontrol_dir"]
            # Fields are joined with an explicit sentinel because several of
            # these files have no trailing newline (ur-serial) while others do
            # (readlink output) — line positions are not stable, delimiters are.
            sep = "printf '@@F@@';"
            raw = self.runner.run(
                "hostname 2>/dev/null;"
                + sep
                + "cat /root/ur-serial 2>/dev/null;"
                + sep
                + "cat /root/ur-robotarm-serial 2>/dev/null;"
                + sep
                + f"readlink {urc}/urcontrol.conf 2>/dev/null;"
                + sep
                + "grep -m1 -o 'URControl [0-9][^ ]* \\[[^]]*\\] [A-Z0-9]*' "
                + f"{self.paths['urcontrol_log']} 2>/dev/null;"
                + sep
                + "uptime 2>/dev/null"
            )
            fields = [f.strip() for f in raw.split("@@F@@")]

            def get(i: int) -> str:
                return fields[i] if i < len(fields) else ""

            model = get(3).replace("urcontrol.conf.", "") if get(3) else None
            return {
                "hostname": get(0) or None,
                "serial": get(1) or None,
                "arm_serial": get(2) or None,
                "model": model,
                "urcontrol_version": get(4) or None,
                "uptime": get(5) or None,
                "source": self.runner.label,
            }

        return self._guard(collect)

    # -- joints + calibration --------------------------------------------------

    def joints(self) -> dict:
        def collect():
            urc = self.paths["urcontrol_dir"]
            result: dict = {}
            try:
                info = self.runner.run(f"cat {urc}/joint_info.txt")
                result["joints"] = parse_joint_info(info)
            except RunnerError:
                result["joints"] = None
                result["note"] = "joint_info.txt unavailable (URSim emulates joints)"
            # The calibration-mismatch line URControl logs at every boot when a
            # replaced joint's checksum no longer matches the kinematic
            # calibration (see the UR Service Handbook §5.2.12: replacing a
            # joint invalidates the calibration).
            try:
                mism = self.runner.run(
                    "grep -c 'kinematic calibration checksum do not match' "
                    + f"{self.paths['urcontrol_log']} 2>/dev/null || true"
                )
                result["calibration_mismatch"] = int(mism.strip() or 0) > 0
                ids = self.runner.run(
                    "grep -o 'checksum of the joint, with id [0-9]*' "
                    + f"{self.paths['urcontrol_log']} 2>/dev/null "
                    + "| grep -o '[0-9]*$' | sort -u || true"
                )
                result["mismatched_joint_ids"] = [int(x) for x in ids.split()] if ids.strip() else []
            except RunnerError:
                result["calibration_mismatch"] = None
            try:
                cal = self.runner.run(f"date -r {urc}/calibration.conf '+%Y-%m-%d' 2>/dev/null || true")
                result["calibration_date"] = cal.strip() or None
            except RunnerError:
                result["calibration_date"] = None
            return result

        return self._guard(collect)

    # -- storage ---------------------------------------------------------------

    def storage(self) -> dict:
        def collect():
            disk = parse_df(self.runner.run("df -k /"))
            out: dict = {"root": disk}
            # The usual suspects when the eMMC fills up.
            heavy = self.runner.run(
                "du -sk /root/log_history.* /root/*.log "
                f"{self.paths['flightreports_dir']} 2>/dev/null | sort -rn || true"
            )
            hogs = []
            for line in heavy.splitlines():
                parts = line.split(maxsplit=1)
                if len(parts) == 2 and parts[0].isdigit():
                    hogs.append({"path": parts[1], "size_mb": round(int(parts[0]) / 1024, 1)})
            out["reclaimable"] = hogs
            return out

        return self._guard(collect)

    # -- programs --------------------------------------------------------------

    def programs(self) -> dict:
        def collect():
            pd = self.paths["program_dir"]
            listing = self.runner.run(
                f"find {pd} -maxdepth 2 \\( -name '*.urp' -o -name '*.script' -o -name '*.installation' \\) "
                "-exec stat -c '%Y %s %n' {} \\; 2>/dev/null || true"
            )
            entries = parse_program_listing(listing)
            return {
                "dir": pd,
                "programs": [e for e in entries if e["kind"] == "urp"],
                "scripts": [e for e in entries if e["kind"] == "script"],
                "installations": [e for e in entries if e["kind"] == "installation"],
            }

        return self._guard(collect)

    # -- installation ----------------------------------------------------------

    def installation(self, name: str = "default") -> dict:
        def collect():
            pd = self.paths["program_dir"]
            data = self.read_file(f"{pd}/{name}.installation")
            parsed = parse_installation(data)
            parsed["name"] = name
            return parsed

        return self._guard(collect)

    # -- URCaps ----------------------------------------------------------------

    def urcaps(self) -> dict:
        def collect():
            listing = self.runner.run(f"ls -1 {self.paths['urcaps_dir']} 2>/dev/null || true")
            return {"urcaps": [line for line in listing.splitlines() if line.strip()]}

        return self._guard(collect)

    # -- recent log noise ------------------------------------------------------

    def recent_errors(self, limit: int = 20) -> dict:
        def collect():
            raw = self.runner.run(
                f"grep -iE 'error|fault|violation|protective' {self.paths['urcontrol_log']} 2>/dev/null "
                f"| tail -n {int(limit)} || true"
            )
            return {"urcontrol_errors": [line for line in raw.splitlines() if line.strip()]}

        return self._guard(collect)

    # -- the whole cell model --------------------------------------------------

    def snapshot(self, *, installation_name: str = "default") -> dict:
        """One dict describing the controller as a machine: identity, joints
        and calibration state, storage, program inventory, active installation,
        URCaps, and recent controller-log errors. Sections fail independently.
        """
        return {
            "identity": self.identity(),
            "joints": self.joints(),
            "storage": self.storage(),
            "programs": self.programs(),
            "installation": self.installation(installation_name),
            "urcaps": self.urcaps(),
            "recent": self.recent_errors(),
        }
