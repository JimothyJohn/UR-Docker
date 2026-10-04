"""Write the cloud-init seed that makes a flashed pick PC image one particular board.

The image (deploy/pi/image/build.sh) is the same for every board; Raspberry Pi OS reads
``user-data``, ``network-config`` and ``meta-data`` from the boot partition on the first boot
(cloud-init NoCloud, ``seedfrom: file:///boot/firmware``). This writes those three files: the
host name, one login user with SSH keys only (no password, passwordless sudo, like Raspberry
Pi OS's default user), the cell address on eth0, optional Wi-Fi, and, when asked, the
installer re-run that points /etc/perceptronics/cell.env at a robot.

    python3 deploy/pi/image/seed.py /Volumes/bootfs --hostname pickpc2 --user nick \\
        --ssh-key ~/.ssh/id_ed25519.pub --address 192.168.3.21/24 --robot-host 192.168.3.3

Every value is checked, then written as JSON — which is YAML, so nothing typed here can
break out of its field. The Wi-Fi password is read from PERCEPTRONICS_WIFI_PSK or a prompt,
never from the command line. Stdlib only.
"""

from __future__ import annotations

import argparse
import getpass
import ipaddress
import json
import os
import re
import secrets
import shlex
import sys
from pathlib import Path

HOSTNAME_RE = re.compile(r"^(?!-)[a-z0-9-]{1,63}(?<!-)$")
USER_RE = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")
CELL_RE = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")
ROBOT_HOST_RE = re.compile(r"^[A-Za-z0-9.:-]{1,253}$")
TIMEZONE_RE = re.compile(r"^[A-Za-z0-9_+-]+(/[A-Za-z0-9_+-]+)*$")
COUNTRY_RE = re.compile(r"^[A-Z]{2}$")
KEY_TYPES = (
    "ssh-ed25519",
    "ssh-rsa",
    "ecdsa-sha2-nistp256",
    "ecdsa-sha2-nistp384",
    "ecdsa-sha2-nistp521",
    "sk-ssh-ed25519@openssh.com",
    "sk-ecdsa-sha2-nistp256@openssh.com",
)
RESERVED_USERS = {"root", "perceptronics", "daemon", "bin", "sys", "nobody", "sshd"}
INSTALL = "/opt/perceptronics/deploy/install.sh"
WHEELS = "/opt/perceptronics/wheels"
SEED_FILES = ("user-data", "network-config", "meta-data")


class SeedError(ValueError):
    """A value the seed refuses to write."""


def check_hostname(name: str) -> str:
    if not HOSTNAME_RE.match(name):
        raise SeedError(f"hostname {name!r}: 1-63 of a-z 0-9 -, not starting or ending with -")
    return name


def check_user(name: str) -> str:
    if not USER_RE.match(name) or name in RESERVED_USERS:
        raise SeedError(f"user {name!r}: a lowercase login name that isn't a system account")
    return name


def check_ssh_key(line: str) -> str:
    line = line.strip()
    if not line or "\n" in line or "\r" in line or "\x00" in line:
        raise SeedError("an SSH public key is one non-empty line")
    parts = line.split()
    if len(parts) < 2 or parts[0] not in KEY_TYPES:
        raise SeedError(f"not an SSH public key (type {parts[0]!r}); pass the .pub file")
    if not re.fullmatch(r"[A-Za-z0-9+/]+={0,3}", parts[1]):
        raise SeedError("SSH public key body is not base64")
    if "PRIVATE KEY" in line:
        raise SeedError("that is a private key; pass the .pub file")
    return line


def read_ssh_keys(paths: list[str]) -> list[str]:
    keys: list[str] = []
    for p in paths:
        text = Path(p).expanduser().read_text(encoding="utf-8")
        if "PRIVATE KEY" in text:
            raise SeedError(f"{p} is a private key; pass the .pub file")
        for ln in text.splitlines():
            if ln.strip() and not ln.lstrip().startswith("#"):
                keys.append(check_ssh_key(ln))
    if not keys:
        raise SeedError("no SSH public key: the image has no password login, pass --ssh-key")
    return keys


def check_interface(address: str | None) -> ipaddress.IPv4Interface | None:
    if address is None:
        return None
    try:
        iface = ipaddress.IPv4Interface(address)
    except ValueError as exc:
        raise SeedError(
            f"--address {address!r}: expected an IPv4 address with prefix, e.g. 192.168.3.21/24"
        ) from exc
    if "/" not in address:
        raise SeedError(f"--address {address!r}: give the prefix too, e.g. /24")
    net = iface.network
    if net.prefixlen < 31 and iface.ip in (net.network_address, net.broadcast_address):
        raise SeedError(f"--address {address}: that is the subnet's network or broadcast address")
    return iface


def check_ip(value: str, what: str) -> str:
    try:
        return str(ipaddress.IPv4Address(value))
    except ValueError as exc:
        raise SeedError(f"{what} {value!r}: expected an IPv4 address") from exc


def check_cidr(value: str) -> str:
    try:
        return str(ipaddress.IPv4Network(value, strict=True))
    except ValueError as exc:
        raise SeedError(f"--allow-from {value!r}: expected a subnet like 192.168.3.0/24") from exc


def check_ssid(ssid: str) -> str:
    raw = ssid.encode("utf-8")
    if not 1 <= len(raw) <= 32 or any(c < 0x20 or c == 0x7F for c in raw):
        raise SeedError("--wifi-ssid: 1-32 bytes, no control characters")
    return ssid


def check_psk(psk: str) -> str:
    if re.fullmatch(r"[0-9a-fA-F]{64}", psk):
        return psk
    if not 8 <= len(psk) <= 63 or any(not 0x20 <= ord(c) <= 0x7E for c in psk):
        raise SeedError("Wi-Fi password: 8-63 printable ASCII characters (or a 64-hex-digit PSK)")
    return psk


def build_user_data(
    *,
    hostname: str,
    user: str,
    keys: list[str],
    timezone: str,
    cell: str | None,
    robot_host: str | None,
    allow_from: str | None,
) -> dict:
    if not TIMEZONE_RE.match(timezone):
        raise SeedError(f"--timezone {timezone!r}: an IANA name like America/Chicago")
    data: dict = {
        "hostname": check_hostname(hostname),
        "manage_etc_hosts": True,
        "timezone": timezone,
        # Merged over Raspberry Pi OS's default_user (its groups: sudo, plugdev, video, ...).
        "user": {
            "name": check_user(user),
            "shell": "/bin/bash",
            "lock_passwd": True,
            "sudo": "ALL=(ALL) NOPASSWD:ALL",
            "ssh_authorized_keys": [check_ssh_key(k) for k in keys],
        },
        "ssh_pwauth": False,
        "runcmd": [["systemctl", "enable", "--now", "ssh"]],
    }
    if cell or robot_host or allow_from:
        args = [INSTALL, "--reconfigure"]
        if cell:
            if not CELL_RE.match(cell):
                raise SeedError(f"--cell {cell!r}: a cell name like ur3")
            args += ["--cell", cell]
        if robot_host:
            if not ROBOT_HOST_RE.match(robot_host):
                raise SeedError(f"--robot-host {robot_host!r}: an IP address or host name")
            args += ["--robot-host", robot_host]
        if allow_from:
            args += ["--allow-from", check_cidr(allow_from)]
        # The image carries exactly one wheel; install.sh with it installs nothing new and
        # only rewrites cell.env + the firewall, then restarts the service. Offline.
        cmd = (
            "set -e; w=$(ls "
            + WHEELS
            + "/*-py3-none-any.whl | tail -n 1); "
            + " ".join(shlex.quote(a) for a in args)
            + ' --wheel "$w" >/var/log/perceptronics-seed.log 2>&1'
        )
        data["runcmd"].append(["sh", "-c", cmd])
    return data


def build_network_config(
    *,
    address: ipaddress.IPv4Interface | None,
    gateway: str | None,
    dns: list[str],
    wifi_ssid: str | None,
    wifi_psk: str | None,
    wifi_country: str,
) -> dict:
    eth0: dict = {"optional": True}
    if address is None:
        if gateway or dns:
            raise SeedError("--gateway / --dns need --address (DHCP brings its own)")
        eth0["dhcp4"] = True
    else:
        eth0["dhcp4"] = False
        eth0["addresses"] = [str(address)]
        if gateway:
            gw = ipaddress.IPv4Address(check_ip(gateway, "--gateway"))
            if gw not in address.network:
                raise SeedError(f"--gateway {gw} is not on {address.network}")
            eth0["routes"] = [{"to": "default", "via": str(gw)}]
        if dns:
            eth0["nameservers"] = {"addresses": [check_ip(d, "--dns") for d in dns]}
    net: dict = {"version": 2, "ethernets": {"eth0": eth0}}
    if wifi_ssid is not None:
        if not COUNTRY_RE.match(wifi_country):
            raise SeedError(f"--wifi-country {wifi_country!r}: two capital letters, e.g. US")
        if wifi_psk is None:
            raise SeedError("Wi-Fi needs a password (PERCEPTRONICS_WIFI_PSK or the prompt)")
        net["wifis"] = {
            "wlan0": {
                "dhcp4": True,
                "optional": True,
                "regulatory-domain": wifi_country,
                "access-points": {check_ssid(wifi_ssid): {"password": check_psk(wifi_psk)}},
            }
        }
    return {"network": net}


def render(header: str, data: dict) -> str:
    # JSON is a subset of YAML; ensure_ascii keeps any odd character an escape sequence.
    return header + json.dumps(data, indent=2, ensure_ascii=True) + "\n"


def write_seed(boot: Path, user_data: dict, network: dict, instance_id: str) -> list[Path]:
    if not boot.is_dir():
        raise SeedError(
            f"{boot} is not a directory (the flashed card's boot partition, e.g. /Volumes/bootfs)"
        )
    if not (boot / "config.txt").is_file() or not (boot / "cmdline.txt").is_file():
        raise SeedError(f"{boot} has no config.txt + cmdline.txt: not a Raspberry Pi boot partition")
    files = {
        "user-data": render("#cloud-config\n", user_data),
        "network-config": render("", network),
        # A new instance id makes cloud-init treat this as a first boot, even on a re-seed.
        "meta-data": render("", {"instance-id": instance_id, "dsmode": "local"}),
    }
    written = []
    for name, text in files.items():
        path = boot / name
        tmp = boot / f".{name}.tmp"
        tmp.write_text(text, encoding="utf-8", newline="\n")
        os.replace(tmp, path)
        written.append(path)
    return written


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("boot", type=Path, help="the flashed card's boot partition (macOS: /Volumes/bootfs)")
    ap.add_argument("--hostname", required=True)
    ap.add_argument("--user", required=True, help="the login user (SSH key only, passwordless sudo)")
    ap.add_argument("--ssh-key", action="append", required=True, metavar="FILE.pub", help="repeatable")
    ap.add_argument("--address", help="eth0's static IPv4 with prefix, e.g. 192.168.3.21/24 (default: DHCP)")
    ap.add_argument("--gateway", help="eth0's default route; the cell network normally has none")
    ap.add_argument("--dns", action="append", default=[], metavar="IP")
    ap.add_argument("--wifi-ssid", help="optional Wi-Fi (password from PERCEPTRONICS_WIFI_PSK or a prompt)")
    ap.add_argument("--wifi-country", default="US")
    ap.add_argument("--timezone", default="America/Chicago")
    ap.add_argument("--cell", help="re-run the installer with this cell profile on the first boot")
    ap.add_argument("--robot-host", help="the controller's address, written into cell.env on the first boot")
    ap.add_argument("--allow-from", help="the subnet allowed to reach :7621/:7622 (default: the robot's /24)")
    args = ap.parse_args(argv)
    try:
        psk = None
        if args.wifi_ssid is not None:
            psk = os.environ.get("PERCEPTRONICS_WIFI_PSK") or getpass.getpass(
                f"Wi-Fi password for {args.wifi_ssid!r}: "
            )
        user_data = build_user_data(
            hostname=args.hostname,
            user=args.user,
            keys=read_ssh_keys(args.ssh_key),
            timezone=args.timezone,
            cell=args.cell,
            robot_host=args.robot_host,
            allow_from=args.allow_from,
        )
        network = build_network_config(
            address=check_interface(args.address),
            gateway=args.gateway,
            dns=args.dns,
            wifi_ssid=args.wifi_ssid,
            wifi_psk=psk,
            wifi_country=args.wifi_country,
        )
        instance_id = f"perceptronics-{args.hostname}-{secrets.token_hex(4)}"
        written = write_seed(args.boot, user_data, network, instance_id)
    except (SeedError, OSError) as exc:
        print(f"seed: {exc}", file=sys.stderr)
        return 2
    for p in written:
        print(f"wrote {p}")
    host = args.address.split("/")[0] if args.address else f"{args.hostname}.local"
    print(f"first boot: ssh {args.user}@{host}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
