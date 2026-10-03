"""The pick PC deployment's contract, checked without a Pi (deploy/pi/, scripts/deploy-pi.sh).

The scripts must parse (and pass shellcheck where it is installed), the systemd unit must
run the cockpit as an unprivileged user with a restart policy and a command line the real
CLI accepts, the installer must pin librealsense to the release the ctypes binding was
written against and never pipe a download into a shell, every variable the cell.env
template sets must be one the code reads, the cell.env the installer writes must parse
the same under perceptronics's own parser, and the firewall must drop by default.
"""

from __future__ import annotations

import re
import shutil
import subprocess
import sys
from pathlib import Path

import pytest

from perceptronics import webapp
from perceptronics.cell import list_cells, parse_env_text
from perceptronics.picknode import DEFAULT_PICK_PORT

ROOT = Path(__file__).resolve().parents[1]
PI = ROOT / "deploy" / "pi"
INSTALL = PI / "install.sh"
UNIT = PI / "perceptronics-cockpit.service"
NFT = PI / "nftables.conf"
TEMPLATE = PI / "cell.env.template"
DOCTOR = PI / "perceptronics-doctor"
CELL_DHCP_CONF = PI / "cell-dhcp.conf"
CELL_DHCP_UNIT = PI / "perceptronics-cell-dhcp.service"
NM_HOOK = PI / "50-perceptronics-cell"
DEPLOY = ROOT / "scripts" / "deploy-pi.sh"
BASH_SCRIPTS = [INSTALL, DEPLOY]


def _text(path: Path) -> str:
    return path.read_text(encoding="utf-8")


def _code(text: str) -> str:
    """The script without its comment lines (so documentation can name what code must not do)."""
    return "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))


# ----- shell scripts ------------------------------------------------------------------------


def _real_bash() -> str | None:
    """A bash that runs (on a Windows runner ``bash`` is WSL's launcher with no distribution)."""
    bash = shutil.which("bash")
    if bash is None:
        return None
    try:
        ran = subprocess.run([bash, "-c", "echo ok"], capture_output=True, timeout=30)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return bash if ran.stdout.strip() == b"ok" else None


@pytest.mark.parametrize("script", BASH_SCRIPTS, ids=lambda p: p.name)
def test_bash_scripts_parse(script):
    bash = _real_bash()
    if bash is None:
        pytest.skip("no working bash on this machine")
    subprocess.run([bash, "-n", str(script)], check=True)


def test_doctor_wrapper_parses_as_posix_sh():
    subprocess.run(["sh", "-n", str(DOCTOR)], check=True)


@pytest.mark.parametrize("script", [*BASH_SCRIPTS, DOCTOR], ids=lambda p: p.name)
def test_shellcheck_clean(script):
    if shutil.which("shellcheck") is None:
        pytest.skip("shellcheck not installed (CI's lint job installs it)")
    result = subprocess.run(["shellcheck", str(script)], capture_output=True, text=True)
    assert result.returncode == 0, result.stdout + result.stderr


@pytest.mark.parametrize("script", BASH_SCRIPTS, ids=lambda p: p.name)
def test_strict_mode(script):
    assert re.search(r"^set -euo pipefail$", _text(script), re.M)


@pytest.mark.parametrize("script", [*BASH_SCRIPTS, DOCTOR], ids=lambda p: p.name)
def test_no_password_plumbing(script):
    code = _code(_text(script))
    assert "sshpass" not in code
    assert "SSHPASS" not in code
    assert not re.search(r"\bsudo\s+-S\b", code), "sudo -S reads a password from stdin"


@pytest.mark.parametrize("script", [*BASH_SCRIPTS, DOCTOR], ids=lambda p: p.name)
def test_never_pipes_a_download_into_a_shell(script):
    code = _code(_text(script))
    assert not re.search(r"\b(curl|wget)\b[^\n|]*\|\s*(sudo\s+)?(ba|z)?sh\b", code)
    assert not re.search(r"\b(ba)?sh\s+<\(\s*(curl|wget)", code)


def test_deploy_script_is_executable():
    # the mode git records is what a Linux checkout (the PC, CI) gets; Windows has no exec bit
    for path in (INSTALL, DEPLOY, DOCTOR):
        rel = path.relative_to(ROOT).as_posix()
        try:
            staged = subprocess.run(
                ["git", "ls-files", "-s", "--", rel], cwd=ROOT, capture_output=True, text=True, timeout=30
            ).stdout.split()
        except (OSError, subprocess.TimeoutExpired):
            staged = []
        if staged:
            assert staged[0] == "100755", f"{rel} is committed without its executable bit"
        elif sys.platform != "win32":
            assert path.stat().st_mode & 0o111, f"{path} is not executable"


# ----- librealsense pin -------------------------------------------------------------------


def _assignment(name: str, text: str) -> str:
    m = re.search(rf'^readonly {name}="([^"]+)"$', text, re.M)
    assert m, f"no readonly {name}=... in install.sh"
    return m.group(1)


def test_librealsense_is_pinned_to_the_binding_release():
    text = _text(INSTALL)
    tag = _assignment("LIBREALSENSE_TAG", text)
    commit = _assignment("LIBREALSENSE_COMMIT", text)
    assert re.fullmatch(r"v\d+\.\d+\.\d+", tag), f"not a release tag: {tag}"
    assert re.fullmatch(r"[0-9a-f]{40}", commit), "the tag's commit must be pinned too"
    # the ctypes binding checks enum ordinals written against this minor
    binding = _text(ROOT / "perceptronics" / "realsense.py")
    minor = re.search(r"written against librealsense (\d+\.\d+)", binding).group(1)
    assert tag.lstrip("v").startswith(minor + "."), f"{tag} is not librealsense {minor}.x"
    # and the container builds the same release
    docker_ref = re.search(r"LIBREALSENSE_REF=(\S+)", _text(ROOT / "Dockerfile.perceptronics")).group(1)
    assert docker_ref == tag
    assert 'rev-parse HEAD)"' in text and "$LIBREALSENSE_COMMIT" in text, "the clone's commit is checked"


@pytest.mark.parametrize(
    "option",
    [
        "-DFORCE_RSUSB_BACKEND=ON",
        "-DBUILD_EXAMPLES=OFF",
        "-DBUILD_GRAPHICAL_EXAMPLES=OFF",
        "-DBUILD_PYTHON_BINDINGS=OFF",
        "-DCHECK_FOR_UPDATES=OFF",
    ],
)
def test_librealsense_build_options(option):
    block = re.search(r"LIBREALSENSE_CMAKE_OPTS=\((.*?)\)", _text(INSTALL), re.S).group(1)
    assert option in block.split()


def test_installed_library_is_what_the_template_points_at():
    text = _text(INSTALL)
    link = _assignment("LIBREALSENSE_LINK", text)
    assert parse_env_text(_text(TEMPLATE))["REALSENSE_LIB"] == f"{link}/lib/librealsense2.so"


def test_installer_copies_only_files_that_exist():
    m = re.search(r"for f in ([^;]+); do", _text(INSTALL))
    names = [n for n in m.group(1).split() if n != "\\"]
    assert names, "copy_deploy_files names no files"
    for name in names:
        assert (PI / name).is_file(), f"install.sh copies {name}, which is not in deploy/pi/"


# ----- systemd unit -----------------------------------------------------------------------


def _unit(path: Path) -> dict[str, dict[str, list[str]]]:
    """systemd's INI dialect: repeated keys accumulate, ``#``/``;`` lines are comments."""
    sections: dict[str, dict[str, list[str]]] = {}
    current = None
    for raw in _text(path).splitlines():
        line = raw.strip()
        if not line or line[0] in "#;":
            continue
        m = re.fullmatch(r"\[([A-Za-z]+)\]", line)
        if m:
            current = sections.setdefault(m.group(1), {})
            continue
        assert current is not None, f"{path.name}: {raw!r} is outside a section"
        assert "=" in line, f"{path.name}: {raw!r} is not KEY=VALUE"
        key, value = line.split("=", 1)
        assert re.fullmatch(r"[A-Za-z]+", key.strip()), f"{path.name}: bad key {key!r}"
        current.setdefault(key.strip(), []).append(value.strip())
    return sections


def _one(section: dict[str, list[str]], key: str) -> str:
    assert key in section, f"missing {key}="
    return section[key][-1]


def test_unit_has_the_three_sections():
    assert set(_unit(UNIT)) == {"Unit", "Service", "Install"}
    assert _one(_unit(UNIT)["Install"], "WantedBy") == "multi-user.target"


def test_unit_runs_unprivileged_and_restarts():
    svc = _unit(UNIT)["Service"]
    user = _one(svc, "User")
    assert user and user not in ("root", "0")
    assert _one(svc, "Restart") in ("on-failure", "always")
    assert _one(svc, "RestartSec")
    assert _one(svc, "NoNewPrivileges") == "yes"
    assert _one(svc, "ProtectSystem") == "strict"
    assert _one(svc, "PrivateTmp") == "yes"
    assert _one(svc, "CapabilityBoundingSet") == ""
    assert "PrivateDevices" not in svc, "PrivateDevices hides /dev/bus/usb from librealsense"
    assert "char-usb_device rw" in svc["DeviceAllow"]


def test_unit_state_is_writable_where_the_cockpit_writes():
    svc = _unit(UNIT)["Service"]
    workdir = _one(svc, "WorkingDirectory")
    writable = " ".join(svc.get("ReadWritePaths", [])).split()
    assert workdir in writable, "captures/ (relative to the working directory) must be writable"
    template = parse_env_text(_text(TEMPLATE))
    for key in ("PERCEPTRONICS_HANDEYE_FILE", "UR_AUDIT_LOG"):
        assert Path(template[key]).is_relative_to(workdir), f"{key} is outside ReadWritePaths"


def test_unit_reads_the_cell_file_the_installer_writes():
    svc = _unit(UNIT)["Service"]
    cell_env = _assignment("CELL_ENV", _text(INSTALL).replace("${ETC_DIR}", "/etc/perceptronics"))
    assert _one(svc, "EnvironmentFile") == cell_env
    assert f"--cell {cell_env}" in _text(DOCTOR).replace('"$CELL"', cell_env)


def test_unit_execstart_parses_under_the_real_cli(monkeypatch):
    argv = _one(_unit(UNIT)["Service"], "ExecStart").split()
    assert argv[0] == "/opt/perceptronics/current/bin/perceptronics"
    monkeypatch.delenv("UR_CELL", raising=False)
    served = {}
    monkeypatch.setattr(webapp, "camera_from_args", lambda args, config: None)
    monkeypatch.setattr(webapp, "robot_from_args", lambda args: None)
    monkeypatch.setattr(webapp, "views_from_args", lambda args, config: [])
    monkeypatch.setattr(webapp, "serve", lambda *a, **kw: served.update(kw))
    from perceptronics import cli

    assert cli.main(argv[1:]) == 0  # argparse exits 2 on an unknown flag
    assert served["bind"] == "0.0.0.0"
    assert served["port"] == webapp.DEFAULT_PORT
    assert served["pick_port"] == DEFAULT_PICK_PORT
    assert served["open_browser"] is False


def test_unit_documents_ports_and_stop_command():
    text = _text(UNIT)
    assert str(webapp.DEFAULT_PORT) in text and str(DEFAULT_PICK_PORT) in text
    assert "systemctl stop perceptronics-cockpit" in text
    assert "pick-server" in text, "the :7622 clash with the sidecar is written where the next person looks"


# ----- cell.env ---------------------------------------------------------------------------


def _env_names_read_by_code() -> set[str]:
    names: set[str] = set()
    for pkg in ("perceptronics", "urctl"):
        for path in (ROOT / pkg).rglob("*.py"):
            names.update(re.findall(r"[\"']((?:UR|PERCEPTRONICS|REALSENSE)_[A-Z0-9_]+)[\"']", _text(path)))
    return names


def test_template_keys_are_read_by_the_code():
    keys = set(parse_env_text(_text(TEMPLATE)))
    assert keys, "empty template"
    unknown = keys - _env_names_read_by_code()
    assert not unknown, f"cell.env.template sets variables no code reads: {sorted(unknown)}"


def test_template_is_plain_for_both_parsers():
    for raw in _text(TEMPLATE).splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        key, value = line.split("=", 1)
        assert not set(value) & set("\"'\\$`#"), (
            f"{key}: systemd and the cell parser read {value!r} differently"
        )


def _cell_writer() -> str:
    m = re.search(r"<<'PY'\n(.*?)\nPY\n", _text(INSTALL), re.S)
    assert m, "install.sh has no cell.env writer heredoc"
    return m.group(1)


def _write_cell(tmp_path: Path, cell: str, robot_host: str) -> subprocess.CompletedProcess:
    out = tmp_path / "cell.env"
    return subprocess.run(
        [sys.executable, "-c", _cell_writer(), cell, robot_host, str(TEMPLATE), str(out)],
        capture_output=True,
        text=True,
        cwd=ROOT,
        env={"PYTHONPATH": str(ROOT), "PATH": "/usr/bin:/bin"},
    )


@pytest.mark.parametrize("cell", list_cells())
def test_written_cell_env_parses_and_is_self_contained(tmp_path, cell):
    result = _write_cell(tmp_path, cell, "192.168.3.3")
    assert result.returncode == 0, result.stderr
    values = parse_env_text((tmp_path / "cell.env").read_text(encoding="utf-8"))
    assert values["UR_HOST"] == "192.168.3.3"
    assert "PERCEPTRONICS_VIEWS" not in values, "the Mac's webcam names must not reach the pick PC"
    assert values["REALSENSE_LIB"] == parse_env_text(_text(TEMPLATE))["REALSENSE_LIB"]
    assert set(values) <= _env_names_read_by_code()


def test_cell_env_needs_a_robot_host(tmp_path):
    empty = [
        c
        for c in list_cells()
        if not parse_env_text(_text(ROOT / "perceptronics" / "cells" / f"{c}.env")).get("UR_HOST")
    ]
    if not empty:
        pytest.skip("every shipped cell has a UR_HOST")
    result = _write_cell(tmp_path, empty[0], "")
    assert result.returncode != 0
    assert "--robot-host" in result.stderr
    assert not (tmp_path / "cell.env").exists()


def test_cell_env_refuses_unknown_cell(tmp_path):
    result = _write_cell(tmp_path, "no-such-cell", "10.0.0.2")
    assert result.returncode != 0 and "--cell" in result.stderr


# ----- firewall ---------------------------------------------------------------------------


def _chain(name: str) -> str:
    m = re.search(rf"chain {name} \{{(.*?)\n    \}}", _text(NFT), re.S)
    assert m, f"no chain {name}"
    return m.group(1)


def test_firewall_drops_by_default():
    assert re.search(r"type filter hook input priority filter; policy drop;", _chain("input"))
    assert re.search(r"hook forward .*policy drop;", _chain("forward"))


def test_firewall_opens_only_ssh_and_the_cockpit_to_the_cell():
    rules = [
        ln.strip() for ln in _chain("input").splitlines() if ln.strip() and not ln.strip().startswith("#")
    ]
    accepts = [r for r in rules if r.endswith("accept")]
    port_rules = [r for r in accepts if "dport" in r and "udp sport 67" not in r]
    assert "tcp dport 22 accept" in port_rules
    cockpit = [r for r in port_rules if r != "tcp dport 22 accept"]
    assert cockpit == [
        "iifname $CELL_IF udp dport 67 accept",  # the robot's DHCP request, on the cell port only
        "ip saddr $CELL_NET tcp dport $COCKPIT_PORTS accept",
    ]
    assert re.search(
        rf"define COCKPIT_PORTS = \{{ {webapp.DEFAULT_PORT}, {DEFAULT_PICK_PORT} \}}", _text(NFT)
    )
    assert 'iif "lo" accept' in rules


def test_firewall_subnet_is_substituted_by_the_installer():
    assert "define CELL_NET = @CELL_NET@" in _text(NFT)
    assert "s#@CELL_NET@#" in _text(INSTALL)
    marker = re.search(r'NFT_MARKER="([^"]+)"', _text(INSTALL)).group(1)
    assert marker in _text(NFT), "install.sh recognises its own /etc/nftables.conf by this marker"
    assert "flush ruleset" not in _code(_text(NFT)), "only our own table is replaced"


# ----- the cell port: fixed address + one-lease DHCP for a robot nobody configured ----------


def _render(path: Path) -> str:
    """The file as install.sh writes it for the defaults (eth0, the ur3 cell's UR_HOST)."""
    from perceptronics import cellnet

    return (
        _text(path)
        .replace("@CELL_IF@", "eth0")
        .replace("@ROBOT_ADDRESS@", cellnet.ROBOT_ADDRESS)
        .replace("@CELL_MASK@", "255.255.255.0")
    )


def test_every_placeholder_in_the_cell_files_is_one_the_installer_fills():
    filled = set(re.findall(r'-e "s#(@[A-Z_]+@)#', _text(INSTALL)))
    for path in (CELL_DHCP_CONF, CELL_DHCP_UNIT, NM_HOOK, NFT):
        used = set(re.findall(r"@[A-Z_]+@", _text(path)))
        assert used <= filled | {"@CELL_NET@"}, f"{path.name}: {used - filled} never substituted"
        assert not re.findall(r"@[A-Z_]+@", _render(path).replace("@CELL_NET@", "x"))


def test_cell_dhcp_serves_one_address_and_nothing_else():
    from perceptronics import cellnet

    lines = [ln for ln in _render(CELL_DHCP_CONF).splitlines() if ln and not ln.startswith("#")]
    assert "port=0" in lines, "no DNS server"
    assert "interface=eth0" in lines and "except-interface=lo" in lines
    ranges = [ln for ln in lines if ln.startswith("dhcp-range=")]
    a = cellnet.ROBOT_ADDRESS
    assert ranges == [f"dhcp-range={a},{a},255.255.255.0,12h"], "one lease: the robot's address never moves"
    # an empty option is sent as "none": no default route and no DNS offered, the cell stays offline
    assert "dhcp-option=option:router" in lines and "dhcp-option=option:dns-server" in lines
    banned = ("dhcp-authoritative", "server=", "address=", "enable-tftp")
    assert not any(ln.startswith(banned) for ln in lines)


def test_cell_dhcp_only_starts_after_the_probe_heard_no_other_server():
    unit = _unit(CELL_DHCP_UNIT)
    assert set(unit) == {"Unit", "Service", "Install"}
    cond = _one(unit["Service"], "ExecCondition")
    assert cond == "/opt/perceptronics/current/bin/python -m perceptronics.cellnet probe @CELL_IF@"
    start = _one(unit["Service"], "ExecStart")
    assert start == "/usr/sbin/dnsmasq --keep-in-foreground --conf-file=/etc/perceptronics/cell-dhcp.conf"
    doc = _text(CELL_DHCP_UNIT)
    assert "stop:" in doc and "journalctl -u perceptronics-cell-dhcp" in doc


def test_link_up_re_probes(tmp_path):
    hook = _render(NM_HOOK)
    assert hook.startswith("#!/bin/sh")
    assert "systemctl restart --no-block perceptronics-cell-dhcp.service" in hook
    # NetworkManager only runs an executable hook: what ships is git's mode, not the checkout's
    # (a Windows checkout has no exec bit at all)
    staged = subprocess.run(
        ["git", "ls-files", "--stage", str(NM_HOOK.relative_to(ROOT))], cwd=ROOT, capture_output=True, text=True
    )
    if staged.returncode == 0 and staged.stdout:
        assert staged.stdout.startswith("100755 "), staged.stdout
    else:
        assert NM_HOOK.stat().st_mode & 0o111
    script = tmp_path / "hook"
    script.write_text(hook, encoding="utf-8")
    subprocess.run(["sh", "-n", str(script)], check=True)
    if shutil.which("shellcheck"):
        r = subprocess.run(["shellcheck", "-s", "sh", str(script)], capture_output=True, text=True)
        assert r.returncode == 0, r.stdout


def test_installer_defaults_and_its_guards():
    code = _code(_text(INSTALL))
    assert 'cell_if="eth0" cell_address="192.168.3.20/24"' in code
    assert "--cell-if)" in code and "--cell-address)" in code
    # a port already on another network (the bench's office LAN) is never re-addressed
    assert re.search(r'if \[ -n "\$have" \]; then\n\s+log "cell port: .* is on another network', code)
    # the lease must be on the pick PC's network, or nothing is served
    assert "is not on ${cidr} — not serving" in code
    assert "/usr/sbin/dnsmasq --test" in code
    assert "dnsmasq-base" in code
