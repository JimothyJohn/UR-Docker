"""The PolyScope 5 URCap on every PolyScope 5 from the floor up (``urcap/urcap5.py``'s
compatibility plan, the API jars read out of a URSim image, the committed jar's imports) and
the one place the Java feature-detects newer API: ``TeachPosition`` — run under a JDK against
stub URCap API classes shaped like PolyScope 5.4's (no ``RobotPositionCallback2``) and 5.8's.

No network and no UR jars: the synthetic bundles below mimic what ``urcap5.py sdk`` found in
the real images (2026-09-28; each case names the image it copies).
"""

from __future__ import annotations

import io
import json
import math
import random
import re
import shutil
import subprocess
import sys
import zipfile
from pathlib import Path

import pytest

from perceptronics import armfk

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "urcap"))
import urcap5  # noqa: E402

SRC = ROOT / "urcap" / "perceptronic-ps5"
JAVA = SRC / "src" / "io" / "advin" / "perceptronic"
PROPS = urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8"))
PLAN = urcap5.compat_plan(PROPS)
DIST = ROOT / "urcap" / "dist" / urcap5.dist_name(PROPS)
JAVAC = shutil.which("javac")


# -- versions and the compatibility plan ------------------------------------------------------


@pytest.mark.parametrize(
    "a, b",
    [("5.4", "5.8"), ("5.8", "5.9.4"), ("5.9.4", "5.10.2"), ("5.10.2", "5.26.1"), ("1.9.0", "1.19.0")],
)
def test_versions_order_numerically(a, b):
    assert urcap5.version_key(a) < urcap5.version_key(b)


@pytest.mark.parametrize("bad", ["", "5", "5.", "latest", "5.x", "5.26.1-rc", " 5.4", "5.4\n", "٥.٤"])
def test_version_key_refuses_what_is_not_a_version(bad):
    with pytest.raises(urcap5.Urcap5Error):
        urcap5.version_key(bad)


def test_the_committed_plan_builds_on_polyscope_5_4_and_guards_5_8_api():
    assert PLAN["floor"] == "5.4"
    assert PLAN["since"] == {"5.8": ["io/advin/perceptronic/TeachPosition2.java"]}
    assert PLAN["optional"] == ["com.ur.urcap.api.domain.value.robotposition"]
    # PolyScope 5.4 ships the API as polyscope-urcap/api-1.7.0.jar (com.ur.urcap:api:1.7.0);
    # 5.10+ refuse a URCap whose pom names a newer API than they carry.
    assert PROPS["urcap.api.version"] == "1.7.0"


def _props(**extra: str) -> dict[str, str]:
    base = {k: v for k, v in PROPS.items() if not k.startswith("compat.")}
    return {**base, **extra}


@pytest.mark.parametrize(
    "extra, why",
    [
        ({}, "not a version"),  # no floor at all
        ({"compat.floor": "five"}, "not a version"),
        ({"compat.floor": "5.8", "compat.since.5.8": "A.java"}, "not newer"),
        ({"compat.floor": "5.8", "compat.since.5.4": "A.java"}, "not newer"),
        ({"compat.floor": "5.4", "compat.since.5.8": " , "}, "names no source"),
        ({"compat.floor": "5.4", "compat.since.latest": "A.java"}, "not a version"),
    ],
)
def test_compat_plan_refuses_a_broken_plan(extra, why):
    with pytest.raises(urcap5.Urcap5Error, match=why):
        urcap5.compat_plan(_props(**extra))


def _names(paths):
    return {p.name for p in paths}


def test_the_floor_compiles_everything_but_the_guarded_code():
    every = _names(JAVA.glob("*.java"))
    assert _names(urcap5.sources_for(SRC, PLAN)) == every - {"TeachPosition2.java"}
    assert _names(urcap5.sources_for(SRC, PLAN, "5.7")) == every - {"TeachPosition2.java"}
    assert _names(urcap5.sources_for(SRC, PLAN, "5.8")) == every
    assert _names(urcap5.sources_for(SRC, PLAN, "5.26.1")) == every


def test_a_since_group_naming_a_missing_file_is_refused():
    plan = {**PLAN, "since": {"5.8": ["io/advin/perceptronic/Gone.java"]}}
    with pytest.raises(urcap5.Urcap5Error, match="Gone.java"):
        urcap5.sources_for(SRC, plan)


def test_nothing_but_the_guard_names_the_guarded_class():
    """TeachPosition2 is compiled after everything else, against newer jars: a static
    reference from the base code would not compile against the floor — and would load it
    on a PolyScope that can't link it. Only TeachPosition.MODERN names it, as a string."""
    for f in JAVA.glob("*.java"):
        if f.name in ("TeachPosition.java", "TeachPosition2.java"):
            continue
        assert "TeachPosition2" not in f.read_text(encoding="utf-8"), f.name
        assert "RobotPositionCallback2" not in f.read_text(encoding="utf-8"), f.name
        assert "robotposition" not in f.read_text(encoding="utf-8"), f.name
    text = (JAVA / "TeachPosition.java").read_text(encoding="utf-8")
    assert not re.search(r"new TeachPosition2|TeachPosition2\.class|TeachPosition2\s+\w+\s*[=;(]", text)
    assert '"io.advin.perceptronic.TeachPosition2"' in text


# -- the API jars out of an image --------------------------------------------------------------


def _jar(exports: str = "", classes: tuple[str, ...] = (), pom: dict[str, str] | None = None) -> bytes:
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as z:
        head = "Manifest-Version: 1.0\r\n"
        if exports:
            raw = f"Export-Package: {exports}".encode()
            head += raw[:72].decode() + "\r\n"
            rest = raw[72:]
            while rest:
                head += " " + rest[:71].decode() + "\r\n"
                rest = rest[71:]
        z.writestr("META-INF/MANIFEST.MF", head + "\r\n")
        for c in classes:
            z.writestr(c, b"\xca\xfe\xba\xbe")
        if pom:
            z.writestr("META-INF/maven/x/y/pom.properties", "".join(f"{k}={v}\n" for k, v in pom.items()))
    return buf.getvalue()


OSGI_4 = _jar('org.osgi.framework;version="1.4",org.osgi.service.url;version="1.0"')
OSGI_6 = _jar('org.osgi.framework;version="1.8",org.osgi.resource;version="1.0"')


def test_polyscope_5_4_the_api_is_one_maven_artifact():
    # ursim_e-series:5.4 — /ursim/GUI/bundle/polyscope-urcap/api-1.7.0.jar
    files = [
        (
            "polyscope-urcap/api-1.7.0.jar",
            _jar(
                'com.ur.urcap.api.domain;version="1.7.0"',
                pom={"groupId": "com.ur.urcap", "artifactId": "api", "version": "1.7.0"},
            ),
        ),
        ("org.osgi.core-6.0.0.jar", OSGI_6),
        ("polyscope-swing-5.19.11.jar", _jar("com.ur.polyscope.swing")),
        ("not-a-jar.jar", b"garbage"),
    ]
    got = urcap5.scan_bundles(files)
    assert sorted(got["jars"]) == ["api-1.7.0.jar", "org.osgi.core-6.0.0.jar"]
    assert got["api_version"] == "1.7.0"
    assert got["exports"]["api-1.7.0.jar"] == {"com.ur.urcap.api.domain": "1.7.0"}


def test_polyscope_5_10_plus_reports_its_newest_package_dependency_provider():
    # ursim_e-series:5.26.1 — polyscope-urcap-impl-13.31.2.jar carries
    # PackageDependencyProvider1_0_0Impl .. 1_19_0Impl (not 1_9 > 1_19: numeric)
    impl = _jar(
        "com.ur.polyscope.urcaps.impl",
        classes=tuple(
            f"com/ur/polyscope/urcaps/version/PackageDependencyProvider{v}Impl.class"
            for v in ("1_0_0", "1_2_56", "1_9_0", "1_19_0", "1_10_0")
        ),
    )
    files = [
        ("polyscope-urcap-impl-13.31.2.jar", impl),
        (
            "polyscope-urcap-urcap-api-13.31.2.jar",
            _jar(
                'com.ur.urcap.api.contribution;version="1.9.0";uses:="a,b",com.ur.urcap.api.domain;version="1.11.0"'
            ),
        ),
        ("org.osgi.core-4.1.0.jar", OSGI_4),
        ("osgi.core-6.0.0.jar", OSGI_6),
    ]
    got = urcap5.scan_bundles(files)
    assert got["api_version"] == "1.19.0"
    # 5.13+ ship both OSGi cores; the old one (no generics) breaks registerService's types
    assert sorted(got["jars"]) == ["osgi.core-6.0.0.jar", "polyscope-urcap-urcap-api-13.31.2.jar"]
    assert got["exports"]["polyscope-urcap-urcap-api-13.31.2.jar"] == {
        "com.ur.urcap.api.contribution": "1.9.0",
        "com.ur.urcap.api.domain": "1.11.0",
    }


def test_polyscope_5_5_to_5_9_report_no_api_version():
    files = [
        (
            "polyscope-urcap/polyscope-urcap-urcap-api-13.2.54.jar",
            _jar('com.ur.urcap.api.domain;version="1.8.0"'),
        ),
        ("org.osgi.core-6.0.0.jar", OSGI_6),
    ]
    assert urcap5.scan_bundles(files)["api_version"] is None


@pytest.mark.parametrize(
    "files, why",
    [
        ([("org.osgi.core-6.0.0.jar", OSGI_6)], "com.ur.urcap.api"),
        ([("api-1.7.0.jar", _jar('com.ur.urcap.api.domain;version="1.7.0"'))], "org.osgi.framework"),
        ([], "com.ur.urcap.api"),
    ],
)
def test_scan_refuses_a_directory_that_is_not_a_polyscope_bundle_dir(files, why):
    with pytest.raises(urcap5.Urcap5Error, match=why):
        urcap5.scan_bundles(files)


@pytest.mark.parametrize(
    "image, want",
    [
        ("5.4", ("universalrobots/ursim_e-series", "5.4")),
        ("5.26.1", ("universalrobots/ursim_e-series", "5.26.1")),
        ("ursim_e-series:5.8", ("universalrobots/ursim_e-series", "5.8")),
        ("universalrobots/ursim_e-series:5.10.2", ("universalrobots/ursim_e-series", "5.10.2")),
    ],
)
def test_split_image(image, want):
    assert urcap5.split_image(image) == want


@pytest.mark.parametrize("bad", ["", "ursim_e-series:", "x:y/z", "5.4 ; rm -rf /", "5.٤", ".5", "5.4\n"])
def test_split_image_refuses_what_is_not_a_tag(bad):
    with pytest.raises(urcap5.Urcap5Error):
        urcap5.split_image(bad)


def test_parse_exports_keeps_commas_inside_quotes():
    got = urcap5.parse_exports('a.b;version="1.2.0";uses:="x.y,z.w",c.d,e.f;version=3')
    assert got == {"a.b": "1.2.0", "c.d": None, "e.f": "3"}


# -- the committed jar's imports ------------------------------------------------------------


def test_import_package_marks_only_the_optional_packages():
    got = urcap5.import_package(
        ["com.ur.urcap.api.domain", "com.ur.urcap.api.domain.value.robotposition", "javax.swing"],
        ["com.ur.urcap.api.domain.value.robotposition"],
    )
    assert got == (
        'com.ur.urcap.api.domain;version="[1.0.0,2.0.0)",'
        'com.ur.urcap.api.domain.value.robotposition;version="[1.0.0,2.0.0)";resolution:=optional,'
        "javax.swing"
    )


def test_the_committed_jar_needs_only_what_polyscope_5_4_exports():
    """Every com.ur.urcap.api package PolyScope 5.4's api-1.7.0.jar exports (read from the
    ursim_e-series:5.4 image, 2026-09-28) against what the committed jar requires."""
    exports_5_4 = {
        "com.ur.urcap.api.contribution",
        "com.ur.urcap.api.contribution.installation",
        "com.ur.urcap.api.contribution.installation.swing",
        "com.ur.urcap.api.contribution.program",
        "com.ur.urcap.api.contribution.program.swing",
        "com.ur.urcap.api.contribution.toolbar",  # the toolbar API is in 1.7.0 (0.6.0 uses it)
        "com.ur.urcap.api.contribution.toolbar.swing",
        "com.ur.urcap.api.domain",
        "com.ur.urcap.api.domain.data",
        "com.ur.urcap.api.domain.program",
        "com.ur.urcap.api.domain.program.nodes",
        "com.ur.urcap.api.domain.program.nodes.builtin",
        "com.ur.urcap.api.domain.program.nodes.contributable",
        "com.ur.urcap.api.domain.program.structure",
        "com.ur.urcap.api.domain.robot",
        "com.ur.urcap.api.domain.script",
        "com.ur.urcap.api.domain.undoredo",
        "com.ur.urcap.api.domain.userinteraction",
        "com.ur.urcap.api.domain.userinteraction.keyboard",
        "com.ur.urcap.api.domain.userinteraction.robot.movement",
        "com.ur.urcap.api.domain.value",
        "com.ur.urcap.api.domain.value.jointposition",
        "com.ur.urcap.api.domain.value.simple",
        "com.ur.urcap.api.domain.variable",
        "org.osgi.framework",
    }
    imports = urcap5.imports_of(urcap5.read_bundle(DIST))
    assert urcap5.missing_imports(imports, exports_5_4) == []
    optional = sorted(p for p, a in imports.items() if a.get("resolution") == "optional")
    assert optional == PLAN["optional"]
    assert all(
        a.get("version") == "[1.0.0,2.0.0)" for p, a in imports.items() if p.startswith("com.ur.urcap.api")
    )


def test_missing_imports_names_each_required_package_a_polyscope_lacks():
    imports = {
        "com.ur.urcap.api.domain": {"version": "[1.0.0,2.0.0)"},
        "com.ur.urcap.api.domain.value.robotposition": {"resolution": "optional"},
        "com.ur.urcap.api.domain.userinteraction.robot.movement": {},
        "org.osgi.framework": {},
        "javax.swing": {},  # the system bundle's; never in an SDK's exports
    }
    assert urcap5.missing_imports(imports, {"com.ur.urcap.api.domain", "org.osgi.framework"}) == [
        "com.ur.urcap.api.domain.userinteraction.robot.movement"
    ]


def test_the_embedded_pom_names_the_floors_api():
    bundle = urcap5.read_bundle(DIST)
    assert "<artifactId>api</artifactId>" in bundle["pom"]
    assert f"<version>{PROPS['urcap.api.version']}</version>" in bundle["pom"]
    assert "io/advin/perceptronic/TeachPosition2.class" in bundle["names"]


@pytest.mark.parametrize(
    "info, ok",
    [
        ({"image": "x:5.4", "api_version": "1.7.0"}, True),
        ({"image": "x:5.6", "api_version": None}, True),  # 5.5-5.9 record none: nothing to hold it to
        ({"image": "x:5.4", "api_version": "1.9.0"}, False),
    ],
)
def test_the_api_version_must_be_the_floors(info, ok):
    if ok:
        urcap5.check_api_version(PROPS, info)
    else:
        with pytest.raises(urcap5.Urcap5Error, match="urcap.api.version"):
            urcap5.check_api_version(PROPS, info)


def test_check_refuses_a_polyscope_older_than_the_floor(tmp_path):
    (tmp_path / "x.jar").write_bytes(OSGI_6)
    (tmp_path / urcap5.SDK_INFO).write_text(json.dumps({"image": "u:5.3", "exports": {}}))
    with pytest.raises(urcap5.Urcap5Error, match="older than compat.floor"):
        urcap5.check(SRC, tmp_path, DIST)


def test_an_empty_sdk_dir_says_how_to_fill_it(tmp_path):
    with pytest.raises(urcap5.Urcap5Error, match=r"urcap5.py sdk --image 5.8"):
        urcap5.sdk_jars(tmp_path / "5.8")


# -- TeachPosition under a JDK, against stub URCap APIs ---------------------------------------

STUBS_5_4 = {
    "com/ur/urcap/api/domain/value/simple/Angle.java": (
        "package com.ur.urcap.api.domain.value.simple; public interface Angle { enum Unit { RAD, DEG } }"
    ),
    "com/ur/urcap/api/domain/value/simple/Length.java": (
        "package com.ur.urcap.api.domain.value.simple; public interface Length { enum Unit { M, MM } }"
    ),
    "com/ur/urcap/api/domain/value/Pose.java": (
        "package com.ur.urcap.api.domain.value; import com.ur.urcap.api.domain.value.simple.*;"
        " public interface Pose { double[] toArray(Length.Unit l, Angle.Unit a); }"
    ),
    "com/ur/urcap/api/domain/value/jointposition/JointPosition.java": (
        "package com.ur.urcap.api.domain.value.jointposition;"
        " import com.ur.urcap.api.domain.value.simple.Angle;"
        " public interface JointPosition { double getPosition(Angle.Unit u); }"
    ),
    "com/ur/urcap/api/domain/value/jointposition/JointPositions.java": (
        "package com.ur.urcap.api.domain.value.jointposition;"
        " public interface JointPositions { JointPosition[] getAllJointPositions(); }"
    ),
    "com/ur/urcap/api/domain/userinteraction/RobotPositionCallback.java": (
        "package com.ur.urcap.api.domain.userinteraction; import com.ur.urcap.api.domain.value.Pose;"
        " import com.ur.urcap.api.domain.value.jointposition.JointPositions;"
        " public abstract class RobotPositionCallback { public abstract void onOk(Pose p, JointPositions q);"
        " public void onCancel() {} }"
    ),
    "com/ur/urcap/api/domain/userinteraction/UserInteraction.java": (
        "package com.ur.urcap.api.domain.userinteraction;"
        " public interface UserInteraction { void getUserDefinedRobotPosition(RobotPositionCallback c); }"
    ),
}
STUBS_5_8 = {
    **STUBS_5_4,
    "com/ur/urcap/api/domain/value/robotposition/PositionParameters.java": (
        "package com.ur.urcap.api.domain.value.robotposition; import com.ur.urcap.api.domain.value.Pose;"
        " import com.ur.urcap.api.domain.value.jointposition.JointPositions;"
        " public interface PositionParameters { Pose getPose(); JointPositions getJointPositions();"
        " Pose getTCPOffset(); }"
    ),
    "com/ur/urcap/api/domain/userinteraction/RobotPositionCallback2.java": (
        "package com.ur.urcap.api.domain.userinteraction;"
        " import com.ur.urcap.api.domain.value.robotposition.PositionParameters;"
        " public abstract class RobotPositionCallback2 { public abstract void onOk(PositionParameters p);"
        " public void onCancel() {} }"
    ),
    "com/ur/urcap/api/domain/userinteraction/UserInteraction.java": (
        "package com.ur.urcap.api.domain.userinteraction;"
        " public interface UserInteraction { void getUserDefinedRobotPosition(RobotPositionCallback c);"
        " void getUserDefinedRobotPosition(RobotPositionCallback2 c); }"
    ),
}

# A pendant that answers the move screen at once: joints Q, the TCP at TCP under offset OFF.
DRIVER = r"""
package io.advin.perceptronic;

import com.ur.urcap.api.domain.userinteraction.*;
import com.ur.urcap.api.domain.value.Pose;
import com.ur.urcap.api.domain.value.jointposition.*;
import com.ur.urcap.api.domain.value.simple.*;
import java.util.*;

public class Driver {
    static double[] Q, TCP, OFF;

    static Pose pose(final double[] p) {
        return new Pose() { public double[] toArray(Length.Unit l, Angle.Unit a) { return p.clone(); } };
    }

    static JointPositions joints() {
        final JointPosition[] all = new JointPosition[6];
        for (int i = 0; i < 6; i++) {
            final double v = Q[i];
            all[i] = new JointPosition() { public double getPosition(Angle.Unit u) { return v; } };
        }
        return new JointPositions() { public JointPosition[] getAllJointPositions() { return all; } };
    }

    static double[] parse(String s) {
        String[] p = s.split(",");
        double[] r = new double[p.length];
        for (int i = 0; i < p.length; i++) r[i] = Double.parseDouble(p[i]);
        return r;
    }

    public static void main(String[] a) throws Exception {
        String type = a[0];
        Q = parse(a[1]); TCP = parse(a[2]); OFF = parse(a[3]);
        final List<String> calls = new ArrayList<String>();
        UserInteraction ui = (UserInteraction) java.lang.reflect.Proxy.newProxyInstance(
            Driver.class.getClassLoader(), new Class<?>[] {UserInteraction.class},
            new java.lang.reflect.InvocationHandler() {
                public Object invoke(Object proxy, java.lang.reflect.Method m, Object[] args)
                        throws Exception {
                    Object cb = args[0];
                    calls.add(m.getParameterTypes()[0].getSimpleName());
                    if (cb instanceof RobotPositionCallback) {
                        ((RobotPositionCallback) cb).onOk(pose(TCP), joints());
                    } else {
                        Class<?> pp = Class.forName(
                            "com.ur.urcap.api.domain.value.robotposition.PositionParameters");
                        Object params = java.lang.reflect.Proxy.newProxyInstance(
                            Driver.class.getClassLoader(),
                            new Class<?>[] {pp}, new java.lang.reflect.InvocationHandler() {
                                public Object invoke(Object p, java.lang.reflect.Method mm, Object[] aa) {
                                    if (mm.getName().equals("getPose")) return pose(TCP);
                                    if (mm.getName().equals("getTCPOffset")) return pose(OFF);
                                    return joints();
                                }
                            });
                        cb.getClass().getMethod("onOk", pp).invoke(cb, params);
                    }
                    return null;
                }
            });
        final Map<String, Object> out = new LinkedHashMap<String, Object>();
        TeachPosition.Teacher modern = TeachPosition.modern();
        out.put("modern", modern == null ? null : modern.getClass().getSimpleName());
        TeachPosition.teacher(type).teach(ui, new TeachPosition.Done() {
            public void taught(JointPositions q, double[] flange) {
                out.put("q0", q.getAllJointPositions()[0].getPosition(Angle.Unit.RAD));
                List<Object> f = null;
                if (flange != null) { f = new ArrayList<Object>(); for (double v : flange) f.add(v); }
                out.put("flange", f);
            }
        });
        out.put("calls", calls);
        // load(): a feature that isn't there, and an implementation that isn't a Teacher
        ClassLoader cl = Driver.class.getClassLoader();
        out.put("absent", TeachPosition.load("com.ur.NoSuchClass", TeachPosition.MODERN, cl) == null);
        out.put("notTeacher", TeachPosition.load("java.lang.String", "java.lang.Object", cl) == null);
        System.out.println(Json.write(out));
    }
}
"""


def _run_teach(tmp_path: Path, stubs: dict[str, str], extra: tuple[str, ...], *args: str) -> dict:
    src = tmp_path / "src"
    for rel, text in stubs.items():
        (src / rel).parent.mkdir(parents=True, exist_ok=True)
        (src / rel).write_text(text, encoding="utf-8")
    pkg = src / "io" / "advin" / "perceptronic"
    pkg.mkdir(parents=True, exist_ok=True)
    (pkg / "Driver.java").write_text(DRIVER, encoding="utf-8")
    for name in ("TeachPosition.java", "PoseMath.java", "Json.java", *extra):
        shutil.copy(JAVA / name, pkg / name)
    out = tmp_path / "classes"
    subprocess.run(
        [
            "javac",
            "--release",
            "8",
            "-Xlint:-options",
            "-nowarn",
            "-encoding",
            "UTF-8",
            "-d",
            str(out),
            *map(str, src.rglob("*.java")),
        ],
        check=True,
        capture_output=True,
    )
    r = subprocess.run(
        ["java", "-cp", str(out), "io.advin.perceptronic.Driver", *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return json.loads(r.stdout)


def _csv(xs) -> str:
    """Numbers as Java's Double.parseDouble reads them (NaN / Infinity, not nan / inf)."""
    java = {"nan": "NaN", "inf": "Infinity", "-inf": "-Infinity"}
    return ",".join(java.get(repr(float(x)), repr(float(x))) for x in xs)


Q = [0.3, -1.2, 1.4, -1.9, -1.5708, 0.7]
TCP = [0.21, -0.33, 0.12, 3.0, 0.4, -0.2]
OFF = [0.0, -0.035, 0.22, 0.0, 0.0, 0.3]


@pytest.mark.skipif(not JAVAC, reason="needs a JDK")
def test_polyscope_5_4_teaches_through_the_old_callback_and_the_arms_geometry(tmp_path):
    got = _run_teach(tmp_path, STUBS_5_4, (), "UR3", _csv(Q), _csv(TCP), _csv(OFF))
    assert got["modern"] is None
    assert got["calls"] == ["RobotPositionCallback"]
    assert got["q0"] == pytest.approx(Q[0])
    want = armfk.frames(Q, "UR3E")[-1]
    assert got["flange"][:3] == pytest.approx(want[:3], abs=1e-9)
    assert got["absent"] and got["notTeacher"]


@pytest.mark.skipif(not JAVAC, reason="needs a JDK")
def test_polyscope_5_4_with_an_arm_the_table_lacks_gives_no_flange(tmp_path):
    got = _run_teach(tmp_path, STUBS_5_4, (), "UNKNOWN", _csv(Q), _csv(TCP), _csv(OFF))
    assert got["flange"] is None and got["q0"] == pytest.approx(Q[0])


@pytest.mark.skipif(not JAVAC, reason="needs a JDK")
def test_polyscope_5_8_teaches_through_callback2_and_the_tcp_offset(tmp_path):
    got = _run_teach(tmp_path, STUBS_5_8, ("TeachPosition2.java",), "UR3", _csv(Q), _csv(TCP), _csv(OFF))
    assert got["modern"] == "TeachPosition2"
    assert got["calls"] == ["RobotPositionCallback2"]
    # flange = tcp ∘ offset⁻¹, whatever the joints say (the controller's calibrated answer)
    want = armfk.Transform.from_pose(TCP).compose(armfk.Transform.from_pose(OFF).inverse()).to_pose()
    assert got["flange"][:3] == pytest.approx(want[:3], abs=1e-9)


@pytest.mark.skipif(not JAVAC, reason="needs a JDK")
def test_callback2_on_the_classpath_without_teachposition2_falls_back(tmp_path):
    """A PolyScope with the new callback but a jar whose guarded class failed to link (or is
    missing) must still teach — the old way — not throw at the operator."""
    got = _run_teach(tmp_path, STUBS_5_8, (), "UR5", _csv(Q), _csv(TCP), _csv(OFF))
    assert got["modern"] is None and got["calls"] == ["RobotPositionCallback"]
    assert got["flange"][:3] == pytest.approx(armfk.frames(Q, "UR5E")[-1][:3], abs=1e-9)


# -- PoseMath.flange: the nominal DH table, the same as perceptronics/armfk.py -------------------

FK = r"""
package io.advin.perceptronic;
import java.util.*;
public class Fk {
    public static void main(String[] a) {
        List<Object> out = new ArrayList<Object>();
        for (int i = 1; i < a.length; i++) {
            String[] p = a[i].split(",");
            double[] q = new double[p.length];
            for (int k = 0; k < p.length; k++) q[k] = Double.parseDouble(p[k]);
            double[] f = PoseMath.flange(a[0], q);
            if (f == null) { out.add(null); continue; }
            List<Object> r = new ArrayList<Object>();
            for (double v : f) r.add(v);
            out.add(r);
        }
        System.out.println(Json.write(out));
    }
}
"""


@pytest.fixture(scope="module")
def fk(tmp_path_factory):
    if not JAVAC:
        pytest.skip("needs a JDK")
    root = tmp_path_factory.mktemp("fk")
    pkg = root / "src" / "io" / "advin" / "perceptronic"
    pkg.mkdir(parents=True)
    (pkg / "Fk.java").write_text(FK, encoding="utf-8")
    for name in ("PoseMath.java", "Json.java"):
        shutil.copy(JAVA / name, pkg / name)
    subprocess.run(
        [
            "javac",
            "--release",
            "8",
            "-Xlint:-options",
            "-encoding",
            "UTF-8",
            "-d",
            str(root / "c"),
            *map(str, pkg.glob("*.java")),
        ],
        check=True,
        capture_output=True,
    )

    def run(model: str, qs: list[list[float]]):
        r = subprocess.run(
            ["java", "-cp", str(root / "c"), "io.advin.perceptronic.Fk", model, *map(_csv, qs)],
            check=True,
            capture_output=True,
            text=True,
        )
        return json.loads(r.stdout)

    return run


@pytest.mark.parametrize(
    "model, key", [("UR3", "UR3E"), ("UR5", "UR5E"), ("UR10", "UR10E"), ("UR16", "UR16E")]
)
def test_java_forward_kinematics_is_armfks(fk, model, key):
    rng = random.Random(model)
    qs = [[rng.uniform(-2 * math.pi, 2 * math.pi) for _ in range(6)] for _ in range(40)]
    qs.append([0, -math.pi / 2, 0, -math.pi / 2, 0, 0])  # the elbow singularity: still a pose
    for q, got in zip(qs, fk(model, qs), strict=True):
        want = armfk.frames(q, key)[-1]
        assert got[:3] == pytest.approx(want[:3], abs=1e-9)
        # the same frame: its tool Z (where the fingertips go) agrees too
        g = (
            armfk.Transform.from_pose(got)
            .compose(armfk.Transform.from_pose([0, 0, 0.163, 0, 0, 0]))
            .to_pose()
        )
        w = (
            armfk.Transform.from_pose(want)
            .compose(armfk.Transform.from_pose([0, 0, 0.163, 0, 0, 0]))
            .to_pose()
        )
        assert g[:3] == pytest.approx(w[:3], abs=1e-9)


@pytest.mark.parametrize(
    "model, q",
    [
        ("UNKNOWN", [0, 0, 0, 0, 0, 0]),
        ("UR20", [0, 0, 0, 0, 0, 0]),  # not in the pre-5.8 table (no PolyScope before 5.8 drives one)
        ("UR3", [0, 0, 0, 0, 0]),
        ("UR3", [0, 0, 0, 0, 0, float("nan")]),
        ("UR3", [0, 0, 0, 0, 0, float("inf")]),
    ],
)
def test_java_forward_kinematics_refuses_what_it_cannot_answer(fk, model, q):
    assert fk(model, [q]) == [None]


def test_the_pre_5_8_table_is_armfks_rows():
    text = (JAVA / "PoseMath.java").read_text(encoding="utf-8")
    for key in ("UR3E", "UR5E", "UR10E", "UR16E"):
        d, a, _ = armfk.DH[key]
        for v in (*d, *a):
            if v:
                assert f"{v:g}" in text, (key, v)


# -- a rebuilt jar is the committed one whatever JDK built it (Nick, 2026-09-29) --------------------

JAVAC_OK = shutil.which("javac") is not None and shutil.which("javap") is not None


def _built_jar(tmp: Path, name: str, source: str, *flags: str, extra: dict[str, bytes] | None = None) -> Path:
    src = tmp / name / "src" / "p"
    src.mkdir(parents=True)
    (src / "A.java").write_text(source, encoding="utf-8")
    out = tmp / name / "classes"
    cmd = ["javac", "--release", "8", "-Xlint:-options", *flags, "-d", str(out), str(src / "A.java")]
    subprocess.run(cmd, check=True, capture_output=True)
    jar = tmp / f"{name}.jar"
    with zipfile.ZipFile(jar, "w") as z:
        z.writestr("META-INF/MANIFEST.MF", b"Manifest-Version: 1.0\r\n")
        for f in sorted(out.rglob("*.class")):
            z.writestr(f.relative_to(out).as_posix(), f.read_bytes())
        for k, v in (extra or {}).items():
            z.writestr(k, v)
    return jar


A_SRC = (
    "package p;\npublic class A {\n  static final int K = 7;\n  int x;\n"
    "  public int twice(int v) { return 2 * v + x; }\n}\n"
)


@pytest.mark.skipif(not JAVAC_OK, reason="needs a JDK")
def test_the_same_classes_built_differently_compare_equal(tmp_path):
    # -g vs -g:none: different class bytes (debug tables), the same members — what two JDK
    # majors building the same sources look like
    a = _built_jar(tmp_path, "a", A_SRC, "-g")
    b = _built_jar(tmp_path, "b", A_SRC, "-g:none")
    with zipfile.ZipFile(a) as za, zipfile.ZipFile(b) as zb:
        assert za.read("p/A.class") != zb.read("p/A.class")
    assert urcap5.compare_jars(a, b) == []


@pytest.mark.skipif(not JAVAC_OK, reason="needs a JDK")
@pytest.mark.parametrize(
    ("other", "extra", "why"),
    [
        (A_SRC.replace("public int twice", "public long twice"), None, "p.A: its members differ"),
        (A_SRC.replace("  int x;\n", "  int x;\n  int y;\n"), None, "p.A: its members differ"),
        (A_SRC.replace("K = 7", "K = 8"), None, "p.A: its members differ"),  # a constant is a member
        (A_SRC, {"META-INF/urcap5-sources.sha256": b"other"}, "in the rebuild, not in the committed jar"),
    ],
)
def test_a_real_difference_is_caught(tmp_path, other, extra, why):
    committed = _built_jar(tmp_path, "committed", A_SRC, "-g")
    rebuilt = _built_jar(tmp_path, "rebuilt", other, "-g:none", extra=extra)
    problems = urcap5.compare_jars(rebuilt, committed)
    assert any(why in p for p in problems), problems


@pytest.mark.skipif(not JAVAC_OK, reason="needs a JDK")
def test_a_changed_non_class_entry_is_caught_byte_for_byte(tmp_path):
    a = _built_jar(tmp_path, "a", A_SRC, extra={"META-INF/pom.xml": b"<version>1.7.0</version>"})
    b = _built_jar(tmp_path, "b", A_SRC, extra={"META-INF/pom.xml": b"<version>1.9.0</version>"})
    assert urcap5.compare_jars(a, b) == ["META-INF/pom.xml: differs"]


@pytest.mark.skipif(not JAVAC_OK, reason="needs a JDK")
def test_the_committed_jar_compares_equal_to_itself(tmp_path):
    dist = next((ROOT / "urcap" / "dist").glob("perceptronic-ps5-*.urcap"))
    copy = tmp_path / dist.name
    shutil.copy(dist, copy)
    assert urcap5.compare_jars(copy, dist) == []
    assert len(urcap5.class_signatures(dist)) > 10


# javac synthesises members whose names and shapes change between JDK majors (CI, 2026-09-29: the
# JDK-21 rebuild's lambdas, this$1 and anonymous-class constructors differed from JDK 25's) —
# they are not the sources' members, so the comparison leaves them out
_JAVAP = """\
class p.A$Feed$1 extends java.awt.event.MouseAdapter {
  final p.A$Feed this$1;
    descriptor: Lp/A$Feed;
  final int val$n;
    descriptor: I
  p.A$Feed$1(p.A$Feed, int);
    descriptor: (Lp/A$Feed;I)V
  public void mouseMoved(java.awt.event.MouseEvent);
    descriptor: (Ljava/awt/event/MouseEvent;)V
  private static void lambda$new$3(java.lang.Runnable);
    descriptor: (Ljava/lang/Runnable;)V
  static int access$000(p.A);
    descriptor: (Lp/A;)I
  static final int K = 7;
    descriptor: I
}"""


def test_the_comparison_ignores_what_javac_synthesises_and_keeps_what_the_sources_declare():
    kept = "\n".join(urcap5._source_members("p.A$Feed$1", _JAVAP.splitlines()))
    for gone in ("this$1", "val$n", "lambda$new$3", "access$000", "p.A$Feed$1(p.A$Feed, int)"):
        assert gone not in kept, gone
    for stays in ("mouseMoved", "static final int K = 7;", "(Ljava/awt/event/MouseEvent;)V"):
        assert stays in kept, stays
    # a named nested class's constructor is the sources': it stays, $ in its name or not
    named = [
        "class p.A$Stepper {",
        "  p.A$Stepper(java.lang.String);",
        "    descriptor: (Ljava/lang/String;)V",
        "}",
    ]
    assert "p.A$Stepper(java.lang.String);" in "\n".join(urcap5._source_members("p.A$Stepper", named))
