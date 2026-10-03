"""The pick PC image's contract, checked without a Pi (deploy/pi/image/, scripts/pi-image.sh).

The builder must pin and verify its base image, never start anything or touch the build
host's firewall while it installs into the image (install.sh --image), and leave nothing
that identifies a board. The seed must turn whatever is typed into cloud-init files that
say exactly that and nothing more: every field validated, rendered as JSON (which is YAML),
so a newline, a quote or a shell metacharacter can't add a key or a command.
"""

from __future__ import annotations

import importlib.util
import json
import re
import shutil
import subprocess
from pathlib import Path

import pytest
from hypothesis import given
from hypothesis import strategies as st

ROOT = Path(__file__).resolve().parents[1]
IMAGE = ROOT / "deploy" / "pi" / "image"
BUILD = IMAGE / "build.sh"
SEED = IMAGE / "seed.py"
INSTALL = ROOT / "deploy" / "pi" / "install.sh"
WRAPPER = ROOT / "scripts" / "pi-image.sh"
SCRIPTS = [BUILD, WRAPPER]

_spec = importlib.util.spec_from_file_location("pi_image_seed", SEED)
seed = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(seed)

KEY = "ssh-ed25519 AAAAC3NzaC1lZDI1NTE5AAAAIOMqqnkVzrm0SdG6UOoqKLsabgH5C9okWi0dh2l9GKJl nick@studio"


def _code(text: str) -> str:
    return "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))


def _function(text: str, name: str) -> str:
    m = re.search(rf"^{name}\(\) \{{\n(.*?)^\}}", text, re.S | re.M)
    assert m, f"no function {name}"
    return m.group(1)


def _boot(tmp_path: Path) -> Path:
    boot = tmp_path / "bootfs"
    boot.mkdir()
    (boot / "config.txt").write_text("[all]\n")
    (boot / "cmdline.txt").write_text("console=tty1 root=PARTUUID=x-02 rootwait resize\n")
    return boot


def _user_data(**kw) -> dict:
    args = dict(
        hostname="pickpc2",
        user="nick",
        keys=[KEY],
        timezone="America/Chicago",
        cell=None,
        robot_host=None,
        allow_from=None,
    )
    args.update(kw)
    return seed.build_user_data(**args)


def _network(**kw) -> dict:
    args = dict(address=None, gateway=None, dns=[], wifi_ssid=None, wifi_psk=None, wifi_country="US")
    args.update(kw)
    return seed.build_network_config(**args)


# ----- scripts ------------------------------------------------------------------------------


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
def test_scripts_parse_and_are_strict(script):
    if shutil.which("bash"):
        subprocess.run(["bash", "-n", str(script)], check=True)
    assert "set -euo pipefail" in script.read_text()


@pytest.mark.parametrize("script", SCRIPTS, ids=lambda p: p.name)
def test_shellcheck_clean(script):
    if shutil.which("shellcheck") is None:
        pytest.skip("shellcheck not installed (CI's lint job installs it)")
    result = subprocess.run(["shellcheck", str(script)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("script", [*SCRIPTS, SEED], ids=lambda p: p.name)
def test_scripts_are_executable_or_run_by_python(script):
    if script.suffix == ".sh":
        assert script.stat().st_mode & 0o111


def test_base_image_is_pinned_and_verified():
    text = BUILD.read_text()
    name = re.search(r'^readonly BASE_NAME="([^"]+)"', text, re.M).group(1)
    url = re.search(r'^readonly BASE_URL="([^"]+)"', text, re.M).group(1)
    sha = re.search(r'^readonly BASE_SHA256="([^"]+)"', text, re.M).group(1)
    assert re.fullmatch(r"[0-9a-f]{64}", sha)
    assert url.startswith("https://downloads.raspberrypi.com/") and url.endswith("/${BASE_NAME}")
    assert "arm64-lite" in name
    code = _code(text)
    assert "sha256 mismatch, refusing it" in code
    # the check must come after the download and before the image is used
    assert code.index('!= "$BASE_SHA256"') < code.index('xz -dc "$base"')


def test_builder_never_pipes_a_download_into_a_shell():
    code = _code(BUILD.read_text())
    assert not re.search(r"curl[^\n|]*\|\s*(sudo\s+)?(ba)?sh", code)


def test_builder_starts_nothing_in_the_image():
    code = _code(BUILD.read_text())
    assert "exit 101" in code and "policy-rc.d" in code
    assert '"${root}/usr/sbin/policy-rc.d"' in code.split("rm -rf")[-2] + code.split("rm -rf")[-1]
    assert "--image" in code


def test_builder_generalises_the_image():
    code = _code(BUILD.read_text())
    assert 'rm -f "${root}"/etc/ssh/ssh_host_*' in code
    assert "uninitialized" in code
    assert "apt-get clean" in code
    # the host's resolv.conf is used for the build and the image's put back
    assert code.index("cp -L /etc/resolv.conf") < code.index('cp "${work}/resolv.conf.image"')


def test_builder_unmounts_on_any_exit():
    code = _code(BUILD.read_text())
    assert "trap cleanup EXIT" in code
    body = _function(code, "cleanup")
    assert "umount" in body and "losetup -d" in body


def test_install_image_mode_touches_no_live_system():
    text = INSTALL.read_text()
    assert "--image) IMAGE=1" in text
    units = _function(text, "install_units")
    image_branch = units.split('if [ "$IMAGE" = 1 ]; then', 1)[1].split("\n    fi\n", 1)[0]
    assert "systemctl enable" in image_branch and "return" in image_branch
    assert "restart" not in image_branch and "daemon-reload" not in image_branch
    fw = _function(text, "install_firewall")
    assert '[ "$IMAGE" = 1 ] || systemctl restart nftables.service' in fw
    lib = _function(text, "install_librealsense")
    assert re.search(r'if \[ "\$IMAGE" = 0 \]; then\s+udevadm control', lib)


def test_flash_refuses_internal_disks():
    body = _function(_code(WRAPPER.read_text()), "cmd_flash")
    assert "boot disk" in body
    assert "refusing anything but a removable or external disk" in body
    assert "Type the disk id" in body
    assert "/dev/r${disk}" in body


# ----- seed: user-data ------------------------------------------------------------------------


def test_user_data_is_key_only_with_sudo():
    data = _user_data()
    user = data["user"]
    assert user["name"] == "nick" and user["lock_passwd"] is True
    assert user["ssh_authorized_keys"] == [KEY]
    assert data["ssh_pwauth"] is False
    assert ["systemctl", "enable", "--now", "ssh"] in data["runcmd"]
    assert "passwd" not in user


def test_no_reconfigure_without_a_cell_change():
    assert len(_user_data()["runcmd"]) == 1


def test_reconfigure_runcmd_is_quoted_and_parses():
    data = _user_data(cell="ur3", robot_host="192.168.3.3", allow_from="192.168.3.0/24")
    cmd = data["runcmd"][-1]
    assert cmd[:2] == ["sh", "-c"]
    assert "--robot-host 192.168.3.3" in cmd[2] and "--cell ur3" in cmd[2]
    assert "--allow-from 192.168.3.0/24" in cmd[2] and "--reconfigure" in cmd[2]
    subprocess.run(["sh", "-n", "-c", cmd[2]], check=True)


@pytest.mark.parametrize(
    "kw",
    [
        {"robot_host": "1.2.3.4;reboot"},
        {"robot_host": "$(reboot)"},
        {"robot_host": "a b"},
        {"cell": "ur3; rm -rf /"},
        {"cell": "../../etc/passwd"},
        {"allow_from": "192.168.3.1/24"},  # host bits set
        {"allow_from": "0.0.0.0/0 ; x"},
        {"hostname": "Pickpc"},
        {"hostname": "-pick"},
        {"hostname": "pick\npc"},
        {"hostname": "a" * 64},
        {"hostname": ""},
        {"user": "root"},
        {"user": "perceptronics"},
        {"user": "Nick"},
        {"user": "nick\nroot"},
        {"timezone": "America/Chicago\nruncmd: [reboot]"},
        {"timezone": "../../etc"},
        {"keys": ["ssh-ed25519 AAAA\nssh-rsa BBBB"]},
        {"keys": ["ssh-dss AAAAB3NzaC1kc3M= old"]},
        {"keys": ["ssh-ed25519 not*base64"]},
        {"keys": [""]},
    ],
)
def test_user_data_refuses(kw):
    with pytest.raises(seed.SeedError):
        _user_data(**kw)


def test_private_key_file_is_refused(tmp_path):
    p = tmp_path / "id_ed25519"
    p.write_text(
        "-----BEGIN OPENSSH PRIVATE KEY-----\nb3BlbnNzaC1rZXktdjEAAAAA\n-----END OPENSSH PRIVATE KEY-----\n"
    )
    with pytest.raises(seed.SeedError, match="private key"):
        seed.read_ssh_keys([str(p)])


def test_key_file_with_comments_and_blank_lines(tmp_path):
    p = tmp_path / "keys.pub"
    p.write_text(f"# laptop\n\n{KEY}\n")
    assert seed.read_ssh_keys([str(p)]) == [KEY]


def test_no_keys_is_refused(tmp_path):
    p = tmp_path / "empty.pub"
    p.write_text("\n# nothing\n")
    with pytest.raises(seed.SeedError, match="no SSH public key"):
        seed.read_ssh_keys([str(p)])


# ----- seed: network-config -------------------------------------------------------------------


def test_static_cell_address_has_no_default_route():
    net = _network(address=seed.check_interface("192.168.3.21/24"))["network"]
    eth0 = net["ethernets"]["eth0"]
    assert eth0 == {"optional": True, "dhcp4": False, "addresses": ["192.168.3.21/24"]}
    assert "wifis" not in net


def test_dhcp_by_default():
    assert _network()["network"]["ethernets"]["eth0"]["dhcp4"] is True


def test_gateway_must_be_on_the_subnet():
    with pytest.raises(seed.SeedError, match="is not on"):
        _network(address=seed.check_interface("192.168.3.21/24"), gateway="10.0.0.1")
    eth0 = _network(address=seed.check_interface("10.0.0.56/24"), gateway="10.0.0.1")["network"]["ethernets"][
        "eth0"
    ]
    assert eth0["routes"] == [{"to": "default", "via": "10.0.0.1"}]


@pytest.mark.parametrize(
    "address", ["192.168.3.21", "192.168.3.0/24", "192.168.3.255/24", "300.1.1.1/24", "fe80::1/64", "x"]
)
def test_bad_addresses(address):
    with pytest.raises(seed.SeedError):
        seed.check_interface(address)


def test_gateway_without_address_is_refused():
    with pytest.raises(seed.SeedError):
        _network(gateway="10.0.0.1")


def test_wifi(tmp_path):
    net = _network(wifi_ssid='The "Cage"', wifi_psk="correct horse", wifi_country="US")["network"]
    ap = net["wifis"]["wlan0"]["access-points"]
    assert ap == {'The "Cage"': {"password": "correct horse"}}


@pytest.mark.parametrize("psk", ["short", "x" * 64, "tab\there!", "é" * 10])
def test_bad_psk(psk):
    with pytest.raises(seed.SeedError):
        _network(wifi_ssid="lab", wifi_psk=psk)


def test_hex_psk_is_accepted():
    _network(wifi_ssid="lab", wifi_psk="a" * 64)


@pytest.mark.parametrize("ssid", ["", "a" * 33, "lab\nnet", "lab\x00"])
def test_bad_ssid(ssid):
    with pytest.raises(seed.SeedError):
        _network(wifi_ssid=ssid, wifi_psk="password1")


# ----- seed: files ----------------------------------------------------------------------------


def test_write_seed_files(tmp_path):
    boot = _boot(tmp_path)
    written = seed.write_seed(boot, _user_data(), _network(), "perceptronics-pickpc2-0001")
    assert [p.name for p in written] == list(seed.SEED_FILES)
    ud = (boot / "user-data").read_text()
    assert ud.startswith("#cloud-config\n")
    assert json.loads(ud.split("\n", 1)[1])["hostname"] == "pickpc2"
    assert json.loads((boot / "meta-data").read_text())["instance-id"] == "perceptronics-pickpc2-0001"
    assert json.loads((boot / "network-config").read_text())["network"]["version"] == 2
    assert not list(boot.glob(".*.tmp"))


def test_write_seed_refuses_a_non_boot_directory(tmp_path):
    with pytest.raises(seed.SeedError, match="not a Raspberry Pi boot partition"):
        seed.write_seed(tmp_path, _user_data(), _network(), "x")


def test_cli_end_to_end(tmp_path, capsys):
    boot = _boot(tmp_path)
    key = tmp_path / "id.pub"
    key.write_text(KEY + "\n")
    rc = seed.main(
        [
            str(boot),
            "--hostname",
            "pickpc2",
            "--user",
            "nick",
            "--ssh-key",
            str(key),
            "--address",
            "192.168.3.21/24",
            "--robot-host",
            "192.168.3.3",
        ]
    )
    assert rc == 0
    assert "ssh nick@192.168.3.21" in capsys.readouterr().out
    ud = json.loads((boot / "user-data").read_text().split("\n", 1)[1])
    assert "--robot-host 192.168.3.3" in ud["runcmd"][-1][2]


def test_cli_error_is_a_message_not_a_traceback(tmp_path, capsys):
    boot = _boot(tmp_path)
    key = tmp_path / "id.pub"
    key.write_text(KEY)
    rc = seed.main([str(boot), "--hostname", "Bad_Name", "--user", "nick", "--ssh-key", str(key)])
    assert rc == 2
    assert "seed: hostname" in capsys.readouterr().err
    assert not (boot / "user-data").exists()


def test_reseed_gets_a_new_instance_id(tmp_path, capsys):
    boot = _boot(tmp_path)
    key = tmp_path / "id.pub"
    key.write_text(KEY)
    args = [str(boot), "--hostname", "pickpc2", "--user", "nick", "--ssh-key", str(key)]
    ids = set()
    for _ in range(2):
        assert seed.main(args) == 0
        ids.add(json.loads((boot / "meta-data").read_text())["instance-id"])
    assert len(ids) == 2


# ----- properties -----------------------------------------------------------------------------


@given(st.text(max_size=80))
def test_hostname_property(name):
    try:
        out = seed.check_hostname(name)
    except seed.SeedError:
        return
    assert re.fullmatch(r"[a-z0-9-]{1,63}", out) and not out.startswith("-") and not out.endswith("-")


@given(st.text(min_size=1, max_size=40))
def test_ssid_round_trips_or_is_refused(ssid):
    try:
        net = _network(wifi_ssid=ssid, wifi_psk="password1")
    except seed.SeedError:
        return
    text = seed.render("", net)
    assert text.isascii()
    assert list(json.loads(text)["network"]["wifis"]["wlan0"]["access-points"]) == [ssid]
    assert "\n" not in ssid and "\r" not in ssid


@given(st.text(max_size=120))
def test_any_robot_host_is_quoted_or_refused(host):
    try:
        data = _user_data(robot_host=host)
    except seed.SeedError:
        return
    if not host:
        return
    cmd = data["runcmd"][-1][2]
    assert re.fullmatch(r"[A-Za-z0-9.:-]+", host)
    assert f"--robot-host {host}" in cmd
