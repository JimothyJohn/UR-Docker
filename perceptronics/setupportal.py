"""The setup portal's cockpit side: http://<pick PC>:7621/setup.

An operator connects a laptop to the pick PC at its default address (192.168.3.20) and, on
this page, moves it onto the plant's network and installs update bundles. The cockpit runs
unprivileged and sandboxed, so it changes nothing itself: it checks the request, writes it to
the queue (``PERCEPTRONICS_ADMIN_DIR``/queue) and the root helper
``deploy/pi/perceptronics-admin`` carries it out (started by perceptronics-admin.path).

Off unless ``PERCEPTRONICS_ADMIN_DIR`` is set — the pick PC's unit sets it; a cockpit on a
laptop has no portal. Login is HTTP Basic, ``admin`` / ``admin`` for now (Nick, 2026-10-03),
``PERCEPTRONICS_ADMIN_USER`` / ``PERCEPTRONICS_ADMIN_PASSWORD`` override it. Every POST also
needs the ``X-Perceptronics-Admin: 1`` header: a page on another site can't send that header
without a CORS preflight the cockpit refuses, so a browser that remembers the login can't be
made to post here by someone else's page.

The network rules are the helper's ``validate_network``, repeated here for an immediate,
friendly answer; the helper checks again, and ``tests/test_setup_portal.py`` holds both to the
same cases.
"""

from __future__ import annotations

import base64
import binascii
import datetime as _dt
import hmac
import ipaddress
import json
import os
import re
import shutil
from pathlib import Path
from typing import BinaryIO

ENV_ADMIN_DIR = "PERCEPTRONICS_ADMIN_DIR"
ENV_ADMIN_USER = "PERCEPTRONICS_ADMIN_USER"
ENV_ADMIN_PASSWORD = "PERCEPTRONICS_ADMIN_PASSWORD"
DEFAULT_USER = "admin"
DEFAULT_PASSWORD = "admin"
ADMIN_HEADER = "X-Perceptronics-Admin"
REALM = "Perceptronics setup"

STATUS_FILE = Path("/var/lib/perceptronics-admin/status.json")
NETWORK_ENV = Path("/etc/perceptronics/network.env")
IMAGE_RELEASE = Path("/etc/perceptronics/image-release")
CURRENT_RELEASE = Path("/opt/perceptronics/current")
DEFAULT_ADDRESS = "192.168.3.20/24"
RESCUE_ADDRESS = ipaddress.IPv4Interface("192.168.3.20/24")

MAX_BUNDLE_BYTES = 256 * 1024 * 1024  # the helper's MAX_BUNDLE_BYTES
FREE_MARGIN_BYTES = 64 * 1024 * 1024
_ADDR = re.compile(r"[0-9]{1,3}(\.[0-9]{1,3}){3}")


class PortalError(ValueError):
    """A request the portal refuses; the message is for the operator."""


class PortalBusy(PortalError):
    """Something is already queued or running."""


# ---- the network rules (deploy/pi/perceptronics-admin's validate_network) -------------------


def _ipv4(value, what: str) -> ipaddress.IPv4Address:
    if not isinstance(value, str) or not _ADDR.fullmatch(value):
        raise PortalError(f"{what}: expected an address like 192.168.1.20")
    try:
        return ipaddress.IPv4Address(value)
    except ValueError as exc:
        raise PortalError(f"{what}: {exc}") from None


def validate_network(req) -> dict:
    """{cidr, gateway, dns, robot_host, robot_dhcp} -> the same, checked and normalised."""
    if not isinstance(req, dict):
        raise PortalError("network request must be an object")
    unknown = set(req) - {"cidr", "gateway", "dns", "robot_host", "robot_dhcp"}
    if unknown:
        raise PortalError(f"unknown network fields: {', '.join(sorted(unknown))}")
    cidr = req.get("cidr")
    if not isinstance(cidr, str) or not re.fullmatch(r"[0-9]{1,3}(\.[0-9]{1,3}){3}/[0-9]{1,2}", cidr):
        raise PortalError("address: expected address/prefix like 192.168.1.20/24")
    try:
        iface = ipaddress.IPv4Interface(cidr)
    except ValueError as exc:
        raise PortalError(f"address: {exc}") from None
    if not 8 <= iface.network.prefixlen <= 30:
        raise PortalError("subnet mask: between 255.0.0.0 (/8) and 255.255.255.252 (/30)")
    ip = iface.ip
    if ip.is_loopback or ip.is_multicast or ip.is_link_local or ip.is_unspecified or ip.is_reserved:
        raise PortalError(f"address: {ip} can't be a computer's address")
    if ip in (iface.network.network_address, iface.network.broadcast_address):
        raise PortalError(f"address: {ip} is the network's own or broadcast address with that subnet mask")
    gateway = req.get("gateway") or ""
    if gateway:
        gw = _ipv4(gateway, "gateway")
        if gw not in iface.network:
            raise PortalError(f"gateway: {gw} is not on {iface.network}")
        if gw == ip:
            raise PortalError("gateway: it is this computer's own address")
        gateway = str(gw)
    dns = req.get("dns") or []
    if not isinstance(dns, list) or len(dns) > 3:
        raise PortalError("DNS: up to three addresses")
    dns = [str(_ipv4(d, "DNS")) for d in dns]
    robot = _ipv4(req.get("robot_host"), "robot address")
    if robot == ip:
        raise PortalError("robot address: it is this computer's own address")
    robot_dhcp = req.get("robot_dhcp", False)
    if not isinstance(robot_dhcp, bool):
        raise PortalError("robot_dhcp must be true or false")
    if robot_dhcp and robot not in iface.network:
        raise PortalError(f"giving the robot its address needs it on {iface.network}")
    return {
        "cidr": str(iface),
        "gateway": gateway,
        "dns": dns,
        "robot_host": str(robot),
        "robot_dhcp": robot_dhcp,
    }


def network_from_form(form) -> dict:
    """The setup page's fields -> a validated request.

    ``address`` + ``netmask`` (255.255.255.0, or /24, or 24), ``gateway`` and ``dns`` (blank,
    or addresses separated by commas/spaces), ``robot_host``, ``robot_dhcp``."""
    if not isinstance(form, dict):
        raise PortalError("expected the form's fields")
    address = str(form.get("address", "")).strip()
    mask = str(form.get("netmask", "")).strip().lstrip("/")
    if not address:
        raise PortalError("address: required")
    if not mask:
        raise PortalError("subnet mask: required (255.255.255.0 for most cell networks)")
    if _ADDR.fullmatch(mask):
        try:
            prefix = ipaddress.IPv4Network(f"0.0.0.0/{mask}").prefixlen
        except ValueError:
            raise PortalError(f"subnet mask: {mask} is not a subnet mask") from None
    elif mask.isdigit() and len(mask) <= 2:
        prefix = int(mask)
    else:
        raise PortalError("subnet mask: like 255.255.255.0")
    dns_text = str(form.get("dns", "") or "")
    dns = [d for d in re.split(r"[\s,;]+", dns_text.strip()) if d]
    return validate_network(
        {
            "cidr": f"{address}/{prefix}",
            "gateway": str(form.get("gateway", "") or "").strip(),
            "dns": dns,
            "robot_host": str(form.get("robot_host", "")).strip(),
            "robot_dhcp": form.get("robot_dhcp", False) is True,
        }
    )


def keeps_rescue_address(cidr: str) -> bool:
    return RESCUE_ADDRESS.ip not in ipaddress.IPv4Interface(cidr).network


# ---- the portal ----------------------------------------------------------------------------


def _read_env_file(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return out
    for line in text.splitlines():
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


class Portal:
    def __init__(
        self,
        admin_dir: Path,
        *,
        user: str = DEFAULT_USER,
        password: str = DEFAULT_PASSWORD,
        status_file: Path = STATUS_FILE,
        network_env: Path = NETWORK_ENV,
        image_release: Path = IMAGE_RELEASE,
        current_release: Path = CURRENT_RELEASE,
        robot_host: str = "",
    ):
        self.admin_dir = Path(admin_dir)
        self.queue = self.admin_dir / "queue"
        self.user = user
        self.password = password
        self.status_file = status_file
        self.network_env = network_env
        self.image_release = image_release
        self.current_release = current_release
        self.robot_host = robot_host

    @classmethod
    def from_env(cls, env=None) -> Portal | None:
        env = os.environ if env is None else env
        admin_dir = env.get(ENV_ADMIN_DIR, "").strip()
        if not admin_dir:
            return None
        return cls(
            Path(admin_dir),
            user=env.get(ENV_ADMIN_USER) or DEFAULT_USER,
            password=env.get(ENV_ADMIN_PASSWORD) or DEFAULT_PASSWORD,
            robot_host=env.get("UR_HOST", ""),
        )

    # -- login --

    def authorized(self, header: str | None) -> bool:
        if not header or not header.startswith("Basic "):
            return False
        try:
            raw = base64.b64decode(header[6:].strip(), validate=True).decode("utf-8")
        except (binascii.Error, UnicodeDecodeError, ValueError):
            return False
        user, sep, password = raw.partition(":")
        if not sep:
            return False
        # both compared, always, in constant time
        ok_user = hmac.compare_digest(user.encode(), self.user.encode())
        ok_pass = hmac.compare_digest(password.encode(), self.password.encode())
        return ok_user and ok_pass

    # -- what the page shows --

    def _job_status(self) -> dict:
        try:
            data = json.loads(self.status_file.read_text(encoding="utf-8"))
            return data if isinstance(data, dict) else {}
        except (OSError, ValueError):
            return {}

    def pending(self) -> list[str]:
        try:
            return sorted(p.name[: -len(".ready")] for p in self.queue.glob("*.ready"))
        except OSError:
            return []

    def status(self) -> dict:
        net = _read_env_file(self.network_env)
        cidr = net.get("CELL_ADDRESS") or DEFAULT_ADDRESS
        try:
            release = self.current_release.resolve(strict=True).name
        except OSError:
            release = None
        image = _read_env_file(self.image_release)
        jobs = self._job_status()
        job = jobs.get("job") if isinstance(jobs.get("job"), dict) else None
        return {
            "ok": True,
            "network": {
                "cell_if": net.get("CELL_IF") or "eth0",
                "cidr": cidr,
                "gateway": net.get("GATEWAY", ""),
                "dns": [d for d in net.get("DNS", "").split(",") if d],
                "robot_dhcp": (net.get("CELL_DHCP") or "auto") != "off",
                "robot_host": self.robot_host,
                "rescue_address": str(RESCUE_ADDRESS.ip) if keeps_rescue_address(cidr) else None,
            },
            "release": release,
            "image": image.get("IMAGE"),
            "pending": self.pending(),
            "busy": bool(self.pending()) or (job is not None and job.get("state") == "running"),
            "job": job,
            "history": jobs.get("history", []) if isinstance(jobs.get("history"), list) else [],
        }

    # -- the queue --

    def _new_id(self) -> str:
        stamp = _dt.datetime.now(_dt.timezone.utc).strftime("%Y%m%dT%H%M%S")
        return f"{stamp}-{os.urandom(4).hex()}"

    def _check_idle(self) -> None:
        if not self.queue.is_dir():
            raise PortalError(f"no queue at {self.queue}: this PC's setup portal isn't installed")
        st = self.status()
        if st["busy"]:
            raise PortalBusy("busy: the last change is still being carried out — wait for it to finish")

    def _write_ready(self, job_id: str, request: dict) -> None:
        tmp = self.queue / f".{job_id}.json"
        tmp.write_text(json.dumps(request), encoding="utf-8")
        tmp.replace(self.queue / f"{job_id}.json")
        (self.queue / f"{job_id}.ready").touch()

    def queue_network(self, form) -> dict:
        req = network_from_form(form)
        self._check_idle()
        job_id = self._new_id()
        self._write_ready(job_id, {"kind": "network", "network": req})
        new_ip = req["cidr"].split("/")[0]
        return {
            "ok": True,
            "job": job_id,
            "network": req,
            "new_url": f"http://{new_ip}:7621/setup",
            "rescue_address": str(RESCUE_ADDRESS.ip) if keeps_rescue_address(req["cidr"]) else None,
        }

    def queue_update(self, stream: BinaryIO, length: int) -> dict:
        if length <= 0:
            raise PortalError("no file: choose an update bundle (perceptronics-update-*.tar)")
        if length > MAX_BUNDLE_BYTES:
            raise PortalError(
                f"that file is {length >> 20} MiB; an update bundle is at most {MAX_BUNDLE_BYTES >> 20} MiB"
            )
        self._check_idle()
        free = shutil.disk_usage(self.queue).free
        if free < length + FREE_MARGIN_BYTES:
            raise PortalError(
                f"not enough free space on this PC ({free >> 20} MiB) for a {length >> 20} MiB update"
            )
        job_id = self._new_id()
        part = self.queue / f".{job_id}.tar"
        got = 0
        try:
            with open(part, "wb") as out:
                while got < length:
                    chunk = stream.read(min(1 << 20, length - got))
                    if not chunk:
                        break
                    out.write(chunk)
                    got += len(chunk)
            if got != length:
                raise PortalError(f"upload stopped after {got} of {length} bytes — try again")
            part.replace(self.queue / f"{job_id}.tar")
        except BaseException:
            part.unlink(missing_ok=True)
            raise
        self._write_ready(job_id, {"kind": "update", "bytes": length})
        return {"ok": True, "job": job_id, "bytes": length}
