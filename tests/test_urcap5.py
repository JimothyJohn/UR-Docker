"""The PolyScope 5 (e-Series) Perceptronic URCap: the packager (``urcap/urcap5.py``),
the committed ``dist/`` jar, and the Java client's contract — URL rules, JSON, the
located-target text, and real HTTP against the cockpit (``perceptronics.webapp``).

The Java checks need only a JDK (the client classes import nothing from UR); the build
checks need the URCap API jars too (``python3 urcap/urcap5.py sdk``: the floor's and every
``compat.since`` version's) and skip without.
"""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

from perceptronics.config import PerceptionConfig
from perceptronics.realsense import SyntheticRgbdCamera
from perceptronics.webapp import ViewerApp, ViewerHandler

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "urcap"))
import urcap5  # noqa: E402

SRC = ROOT / "urcap" / "perceptronic-ps5"
JAVA = SRC / "src" / "io" / "advin" / "perceptronic"
DIST = ROOT / "urcap" / "dist" / "perceptronic-ps5-0.9.0.urcap"
JAVAC = shutil.which("javac")
# the screens (pure Swing): the harness lays them out off-screen
SCREEN_JAVA = ("PickScreen.java", "LiveView.java", "LocationsScreen.java")
# the pure-Java classes the harness compiles (no UR API)
PURE_JAVA = (
    "Json.java",
    "Cockpit.java",
    "PickScript.java",
    "PoseMath.java",
    "Diagrams.java",
    "Ui.java",
    "Scene.java",
    "Logo.java",
    "FeedPoller.java",
    "Log.java",
)
SVG = ROOT / "urcap" / "perceptronic.svg"
_PLAN = urcap5.compat_plan(urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8")))
HAS_SDK = all((urcap5.SDK_ROOT / v / urcap5.SDK_INFO).is_file() for v in (_PLAN["floor"], *_PLAN["since"]))


# -- bundle.properties + manifest ----------------------------------------------------------


def test_properties_require_identity_and_a_valid_compatibility():
    good = (SRC / "bundle.properties").read_text(encoding="utf-8")
    props = urcap5.read_properties(good)
    assert props["URCapCompatibility-eSeries"] == "true"
    with pytest.raises(urcap5.Urcap5Error, match="Bundle-Activator"):
        urcap5.read_properties(good.replace("Bundle-Activator=", "#"))
    with pytest.raises(urcap5.Urcap5Error, match="true or false"):
        urcap5.read_properties(
            good.replace("URCapCompatibility-eSeries=true", "URCapCompatibility-eSeries=yes")
        )
    with pytest.raises(urcap5.Urcap5Error, match="neither"):
        urcap5.read_properties(
            good.replace("URCapCompatibility-eSeries=true", "URCapCompatibility-eSeries=false")
        )
    with pytest.raises(urcap5.Urcap5Error, match="key=value"):
        urcap5.read_properties(good + "\nnot a pair\n")


@pytest.mark.parametrize("length", [1, 60, 70, 71, 72, 73, 140, 143, 144, 500])
def test_manifest_lines_wrap_at_72_bytes_and_unwrap_exactly(length):
    value = ("x" * (length - 1)) + "é"  # a multi-byte tail must not be split mid-character badly
    raw = urcap5.manifest_line("Import-Package", value)
    assert raw.endswith(b"\r\n")
    for line in raw.split(b"\r\n")[:-1]:
        assert len(line) <= 72
    # continuation lines start with one space; joining them restores the header
    lines = raw.split(b"\r\n")[:-1]
    joined = lines[0] + b"".join(line[1:] for line in lines[1:])
    assert joined.decode("utf-8", errors="strict") == f"Import-Package: {value}"


def test_import_package_never_imports_java_and_ranges_the_urcap_api():
    got = urcap5.import_package(
        [
            "java.lang",
            "java.util",
            "javax.swing",
            "com.ur.urcap.api.domain",
            "org.osgi.framework",
            "javax.swing",
        ]
    )
    assert got == 'com.ur.urcap.api.domain;version="[1.0.0,2.0.0)",javax.swing,org.osgi.framework'


def test_embedded_pom_names_the_api_version_the_way_polyscope_reads_it():
    # PolyScope 5's PomXMLAPIVersionParser: <dependencies><dependency> groupId com.ur.urcap,
    # artifactId api -> <version>. Parse it the same way.
    props = urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8"))
    path, body = urcap5.pom_xml(props)
    assert path == "META-INF/maven/io.advin/perceptronic/pom.xml"
    ns = {"m": "http://maven.apache.org/POM/4.0.0"}
    deps = ET.fromstring(body).findall("m:dependencies/m:dependency", ns)
    versions = [
        d.findtext("m:version", namespaces=ns)
        for d in deps
        if d.findtext("m:groupId", namespaces=ns) == "com.ur.urcap"
        and d.findtext("m:artifactId", namespaces=ns) == "api"
    ]
    assert versions == [props["urcap.api.version"]]


# -- the committed download ----------------------------------------------------------------


def test_the_downloadable_urcap_is_built_from_the_current_sources():
    assert DIST.is_file(), f"{DIST.relative_to(ROOT)} missing — run `make urcap5-package` and commit it"
    bundle = urcap5.read_bundle(DIST)
    assert bundle["sources_sha256"] == urcap5.sources_digest(SRC), (
        f"{DIST.relative_to(ROOT)} is stale — run `make urcap5-package` and commit it"
    )
    props = urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8"))
    h = bundle["headers"]
    for key in ("Bundle-SymbolicName", "Bundle-Version", "Bundle-Activator"):
        assert h[key] == props[key], key
    assert h["URCapCompatibility-eSeries"] == "true" and h["URCapCompatibility-CB3"] == "false"
    imports = h["Import-Package"].split(",")
    assert not [p for p in imports if p.startswith("java.")]
    assert any(p.startswith("com.ur.urcap.api.domain.userinteraction.robot.movement;") for p in imports)
    # the toolbar button (0.6.0): in the API since 1.7.0 (PolyScope 5.4), so a required import
    assert any(p.startswith("com.ur.urcap.api.contribution.toolbar.swing;") for p in imports)
    assert not any("toolbar" in p and "optional" in p for p in imports)
    for cls in ("ToolbarService", "ToolbarContribution", "Logo", "FeedPoller"):
        assert f"io/advin/perceptronic/{cls}.class" in bundle["names"], cls
    assert bundle["names"][:2] == ["META-INF/", "META-INF/MANIFEST.MF"]
    activator = props["Bundle-Activator"].replace(".", "/") + ".class"
    assert activator in bundle["names"]
    assert "<artifactId>api</artifactId>" in bundle["pom"]


def test_the_downloadable_urcap_passes_polyscope_5s_install_checks():
    # PolyScope 5.26's installer (Settings → System → URCaps → +) runs
    # URCapsServiceImpl.isValidFile before it will touch OSGi, in this order: a manifest,
    # Bundle-Category equal to "urcap" ignoring case (else "the BundleCategory attribute in
    # the manifest file must be present and have the value URCap"), the manifest among the
    # first two zip entries, a non-empty Bundle-SymbolicName and Bundle-Version. Read off
    # polyscope-internal-urcap-10.30.21.jar in the ursim_e-series:5.26.0 image. Dropping the
    # jar into /urcaps (the VM's install-urcap) skips all of it — that is how a bundle with
    # no category loaded in the VM and was refused on the UR3e (2026-09-27).
    bundle = urcap5.read_bundle(DIST)
    h = bundle["headers"]
    assert h.get("Bundle-Category", "").lower() == "urcap"
    assert "META-INF/MANIFEST.MF" in bundle["names"][:2]
    assert h.get("Bundle-SymbolicName") and h.get("Bundle-Version")


@pytest.mark.skipif(not (JAVAC and HAS_SDK), reason="needs a JDK and `urcap5.py sdk`")
def test_package_is_reproducible(tmp_path):
    a = urcap5.package(SRC, tmp_path / "a")
    b = urcap5.package(SRC, tmp_path / "b")
    assert a.read_bytes() == b.read_bytes()


def test_package_reports_a_missing_sdk_clearly(tmp_path):
    with pytest.raises(urcap5.Urcap5Error, match="urcap5.py sdk"):
        urcap5.package(SRC, tmp_path, sdk_root=tmp_path / "no-sdk")


# -- the Java client, under a JDK -----------------------------------------------------------

HARNESS = r"""
package io.advin.perceptronic;

import java.util.*;

@SuppressWarnings("unchecked")
public class Harness {
    public static void main(String[] a) throws Exception {
        Object out;
        switch (a[0]) {
            case "base": {
                List<Object> r = new ArrayList<Object>();
                for (int i = 1; i < a.length; i++) r.add(Cockpit.base(a[i]));
                out = r;
                break;
            }
            case "json": {
                try { out = Json.parse(a[1]); } catch (IllegalArgumentException e) { out = "ERR"; }
                break;
            }
            case "write": out = Json.write(Json.parse(a[1])); break;
            case "target": out = Cockpit.targetText(Json.parseObject(a[1])); break;
            case "six": out = Cockpit.six(Json.parse(a[1])) == null ? null : "ok"; break;
            case "pick": {
                Map<String, Object> o = Json.parseObject(a[1]);
                PickScript s = new PickScript();
                if (o.containsKey("host")) s.host = (String) o.get("host");
                if (o.containsKey("port")) s.port = ((Number) o.get("port")).intValue();
                if (o.containsKey("node")) s.nodeId = (String) o.get("node");
                if (o.containsKey("points")) {
                    for (Object pt : (List<?>) o.get("points")) {
                        Map<?, ?> m = (Map<?, ?>) pt;
                        double[] plane = m.get("plane") == null ? null : Cockpit.six(m.get("plane"));
                        List<?> area = (List<?>) m.get("area");
                        s.points.add(new PickScript.Point(Cockpit.six(m.get("q")), plane,
                                area == null ? 0 : ((Number) area.get(0)).doubleValue(),
                                area == null ? 0 : ((Number) area.get(1)).doubleValue()));
                    }
                }
                if (o.containsKey("values")) {
                    for (Map.Entry<String, Object> e : ((Map<String, Object>) o.get("values")).entrySet()) {
                        s.values.put(e.getKey(), ((Number) e.getValue()).doubleValue());
                    }
                }
                if (o.containsKey("order")) {
                    s.orderFirst = (String) ((List<?>) o.get("order")).get(0);
                    s.orderRows = (String) ((List<?>) o.get("order")).get(1);
                }
                if (o.containsKey("shape")) s.shape = (String) o.get("shape");
                if (o.containsKey("gripCheck")) s.gripCheck = (Boolean) o.get("gripCheck");
                if (o.containsKey("gripLong")) s.gripLongSide = (Boolean) o.get("gripLong");
                if (o.containsKey("closeLook")) s.closeLook = (Boolean) o.get("closeLook");
                if (o.containsKey("arm")) s.arm = (String) o.get("arm");
                if (o.containsKey("popup")) s.popupOnFail = (Boolean) o.get("popup");
                if (o.containsKey("polyscope")) {
                    List<?> v = (List<?>) o.get("polyscope");
                    s.polyscope = new int[v.size()];
                    for (int i = 0; i < v.size(); i++) s.polyscope[i] = ((Number) v.get(i)).intValue();
                }
                if (o.containsKey("var")) s.foundVariable = (String) o.get("var");
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("problem", s.problem());
                m.put("script", s.problem() == null ? s.render() : null);
                m.put("words", PickScript.class.getDeclaredMethod("partText").invoke(s));
                List<Object> toks = new ArrayList<Object>();
                for (int i = -1; i < s.points.size(); i++) toks.add(s.tokens(i));
                m.put("tokens", toks);
                List<Object> teach = new ArrayList<Object>();
                for (int i = -1; i < s.points.size(); i++) teach.add(s.teachTokens(i));
                m.put("teach_tokens", teach);
                out = m;
                break;
            }
            case "at": {
                // the Part step's tap: what Scene.at finds under picture pixel (u, v)
                Scene sc = Scene.parse(Json.parseObject(a[1]));
                Scene.Part p = sc.at(Integer.parseInt(a[2]), Integer.parseInt(a[3]), Integer.parseInt(a[4]));
                out = p == null ? null : java.util.Arrays.asList(p.lengthMm, p.widthMm, p.heightMm);
                break;
            }
            case "set": {
                PickScript s = new PickScript();
                out = s.set(a[1], Double.parseDouble(a[2]));
                break;
            }
            case "numbers": {
                List<Object> r = new ArrayList<Object>();
                for (PickScript.Num n : PickScript.NUMBERS) {
                    Map<String, Object> m = new LinkedHashMap<String, Object>();
                    m.put("key", n.key); m.put("def", n.def); m.put("min", n.min); m.put("max", n.max);
                    m.put("section", n.section);
                    r.add(m);
                }
                out = r;
                break;
            }
            case "reasons": {
                List<Object> r = new ArrayList<Object>();
                for (String[] x : PickScript.REASONS) r.add(Integer.parseInt(x[0]));
                out = r;
                break;
            }
            case "grid": {
                int[][] g = Diagrams.orderGrid(a[1], a[2], Integer.parseInt(a[3]), Integer.parseInt(a[4]));
                List<Object> r = new ArrayList<Object>();
                for (int[] row : g) {
                    List<Object> rr = new ArrayList<Object>();
                    for (int v : row) rr.add(v);
                    r.add(rr);
                }
                out = r;
                break;
            }
            case "plane": {
                List<?> q = (List<?>) Json.parse(a[1]);
                double[][] p = new double[3][];
                for (int i = 0; i < 3; i++) {
                    List<?> xyz = (List<?>) q.get(i);
                    p[i] = new double[3];
                    for (int k = 0; k < 3; k++) p[i][k] = ((Number) xyz.get(k)).doubleValue();
                }
                double[] r = PoseMath.plane(p[0], p[1], p[2]);
                if (r == null) { out = null; break; }
                List<Object> rr = new ArrayList<Object>();
                for (double v : r) rr.add(v);
                rr.add(PoseMath.tiltDeg(r));
                out = rr;
                break;
            }
            case "fingertip": {
                double[] tcp = Cockpit.six(Json.parse(a[1]));
                double[] off = Cockpit.six(Json.parse(a[2]));
                double[] r = PoseMath.fingertip(tcp, off, Double.parseDouble(a[3]));
                out = Arrays.asList(r[0], r[1], r[2]);
                break;
            }
            case "trans": {
                double[] r = PoseMath.trans(Cockpit.six(Json.parse(a[1])), Cockpit.six(Json.parse(a[2])));
                List<Object> rr = new ArrayList<Object>();
                for (double v : r) rr.add(v);
                out = rr;
                break;
            }
            case "scene": {
                Scene sc = Scene.parse(Json.parseObject(a[1]));
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                List<Object> orders = new ArrayList<Object>();
                for (Scene.Part p : sc.parts) orders.add(p.order);
                List<Object> whys = new ArrayList<Object>();
                for (Scene.Part p : sc.rejected) whys.add(p.why);
                List<Object> drawn = new ArrayList<Object>();
                for (Scene.Part p : sc.nearMisses()) drawn.add(p.why);
                m.put("drawn", drawn); m.put("summary", sc.summary());
                m.put("orders", orders); m.put("whys", whys); m.put("surface", sc.surface);
                m.put("width", sc.width); m.put("base", sc.baseFrame);
                out = m;
                break;
            }
            case "reach": {
                double[] r = PickScript.modelReach(a[1]);
                out = r == null ? null : Arrays.asList(r[0], r[1]);
                break;
            }
            case "hostof": out = PickScript.hostOf(a[1]); break;
            case "explain": {
                Exception e = a[1].equals("refused") ? new java.net.ConnectException("Connection refused")
                        : a[1].equals("dns") ? new java.net.UnknownHostException("jetson")
                        : a[1].equals("timeout") ? new java.net.SocketTimeoutException("connect timed out")
                        : new java.net.MalformedURLException("no protocol");
                Cockpit.Advice adv = Cockpit.advise(e, a[2]);
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("summary", adv.summary);
                m.put("checks", Arrays.asList((Object[]) adv.checks));
                m.put("detail", adv.detail);
                m.put("text", Cockpit.explain(e, a[2]));
                m.put("logged", new ArrayList<Object>(Log.recent()));
                out = m;
                break;
            }
            case "nopicture": {
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("text", Cockpit.noPicture(a[1], "Frame didn't arrive within 5000"));
                m.put("logged", new ArrayList<Object>(Log.recent()));
                out = m;
                break;
            }
            case "toggle": {
                // the Picture / Depth toggle in a w-wide frame: where it is, and what a tap there chooses
                int w = Integer.parseInt(a[1]);
                java.awt.Rectangle r = Ui.ViewToggle.bounds(w, 0);
                out = Arrays.asList(r.x, r.y, r.width, r.height);
                break;
            }
            case "liveview": {
                // LiveView painted off-screen: [toggle tapped on Depth?, pixels the overlay changed]
                final LiveView v = new LiveView();
                v.setSize(640, 400);
                java.awt.image.BufferedImage img =
                        new java.awt.image.BufferedImage(848, 480, java.awt.image.BufferedImage.TYPE_INT_RGB);
                v.setFrame(img);
                v.setLive(true);
                final boolean[] heard = new boolean[2];
                v.setViewListener(new LiveView.ViewListener() {
                    public void depthView(boolean on) { heard[0] = true; heard[1] = on; }
                });
                java.awt.Rectangle r = Ui.ViewToggle.bounds(640, 0);
                int tapX = r.x + (a[1].equals("depth") ? r.width * 3 / 4 : r.width / 4);
                int tapY = r.y + r.height / 2;
                java.awt.event.MouseEvent tap = new java.awt.event.MouseEvent(
                        v, java.awt.event.MouseEvent.MOUSE_RELEASED, 0L, 0, tapX, tapY, 1, false);
                for (java.awt.event.MouseListener l : v.getMouseListeners()) l.mouseReleased(tap);
                java.awt.image.BufferedImage plain = paint(v);
                v.setScene(Scene.parse(Json.parseObject(a[2])));
                java.awt.image.BufferedImage drawn = paint(v);
                int changed = 0;
                for (int y = 60; y < 400; y++) {
                    for (int x = 0; x < 640; x++) if (plain.getRGB(x, y) != drawn.getRGB(x, y)) changed++;
                }
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("heard", heard[0]); m.put("depth", heard[1]); m.put("view", v.depthView());
                m.put("changed", changed);
                int green = 0, yellow = 0;
                for (int y = 60; y < 400; y++) {
                    for (int x = 0; x < 640; x++) {
                        int px = drawn.getRGB(x, y);
                        if (px == plain.getRGB(x, y)) continue;
                        int cr = (px >> 16) & 255, cg = (px >> 8) & 255, cb = px & 255;
                        if (cg > 180 && cr < 120 && cb < 180) green++;       // Ui.PART 0x3ddc84
                        if (cr > 230 && cg > 170 && cg < 215 && cb < 110) yellow++; // Ui.JAW 0xffc53d
                    }
                }
                m.put("green", green); m.put("yellow", yellow);
                out = m;
                break;
            }
            case "screen": {
                // the node's whole screen, laid out at the pendant's panel size: does everything fit?
                final Map<String, Object> m = new LinkedHashMap<String, Object>();
                final String[] args = a;
                javax.swing.SwingUtilities.invokeAndWait(new Runnable() {
                    public void run() {
                        PickScreen screen = new PickScreen(null);
                        PickScript ps = new PickScript();
                        ps.host = "192.168.3.10"; ps.nodeId = "ab12";
                        if (args[3].equals("cyl")) ps.shape = "cyl";
                        List<PickScreen.PointRow> rows = new ArrayList<PickScreen.PointRow>();
                        int n = Integer.parseInt(args[4]);
                        for (int i = 0; i < n; i++) {
                            ps.points.add(new PickScript.Point(new double[6], null, 0, 0));
                            rows.add(new PickScreen.PointRow("live table", false));
                        }
                        screen.show(ps, rows, Math.max(0, n - 1));
                        if (args[5].equals("part")) screen.showOptions(0);
                        if (args[5].equals("approach")) screen.showOptions(1);
                        if (args[5].equals("look")) screen.showStep(PickScreen.LOOK);
                        if (args[5].equals("teach")) screen.showStep(PickScreen.PART);
                        if (args[5].equals("grip")) screen.showStep(PickScreen.GRIP);
                        screen.setSize(Integer.parseInt(args[1]), Integer.parseInt(args[2]));
                        layout(screen);
                        List<Object> clipped = new ArrayList<Object>();
                        List<Object> scrollers = new ArrayList<Object>();
                        List<Object> texts = new ArrayList<Object>();
                        walk(screen, screen, clipped, scrollers, texts);
                        m.put("clipped", clipped); m.put("scrollers", scrollers); m.put("texts", texts);
                    }
                });
                out = m;
                break;
            }
            case "svg": out = Logo.SVG; break;
            case "logo": {
                // the mark at the toolbar's size: badge pixels, white glyph pixels, clear corners
                java.awt.image.BufferedImage img =
                        Logo.image(Integer.parseInt(a[1]), java.awt.Color.WHITE, Ui.ACCENT);
                int white = 0, badge = 0, clear = 0;
                for (int y = 0; y < img.getHeight(); y++) {
                    for (int x = 0; x < img.getWidth(); x++) {
                        int p = img.getRGB(x, y);
                        if ((p >>> 24) < 16) clear++;
                        else if ((p & 0xffffff) == 0xffffff) white++;
                        else badge++;
                    }
                }
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("white", white); m.put("badge", badge); m.put("clear", clear);
                m.put("corner_clear", (img.getRGB(0, 0) >>> 24) < 16);
                m.put("size", img.getWidth());
                // a plain glyph on nothing: only its ink is opaque
                java.awt.image.BufferedImage glyph = Logo.image(64, java.awt.Color.BLACK, null);
                int ink = 0;
                for (int y = 0; y < 64; y++) {
                    for (int x = 0; x < 64; x++) if ((glyph.getRGB(x, y) >>> 24) > 200) ink++;
                }
                m.put("glyph_ink", ink);
                m.put("lens", (glyph.getRGB(36, 24) >>> 24) > 200);      // the dot in the bowl
                m.put("bowl_hole", (glyph.getRGB(36, 16) >>> 24) < 16);   // between dot and bowl
                m.put("stem", (glyph.getRGB(22, 46) >>> 24) > 200);
                out = m;
                break;
            }
            case "poll": {
                // FeedPoller against a cockpit: until `want` frames or 6 s; every callback in order
                final List<Object> events = new ArrayList<Object>();
                final int want = Integer.parseInt(a[2]);
                final Object lock = new Object();
                FeedPoller p = new FeedPoller(new Cockpit(a[1]), new FeedPoller.Listener() {
                    public void frame(java.awt.image.BufferedImage image, String fps) {
                        synchronized (lock) {
                            events.add("frame " + image.getWidth() + "x" + image.getHeight() + " fps=" + fps);
                            lock.notifyAll();
                        }
                    }
                    public void live(String base) { synchronized (lock) { events.add("live " + base); } }
                    public void waiting(String why) {
                        synchronized (lock) { events.add("waiting"); lock.notifyAll(); }
                    }
                    public void failed(String why) {
                        synchronized (lock) { events.add("failed " + why.split("\\n")[0]); lock.notifyAll(); }
                    }
                });
                p.start("poll-test");
                long until = System.currentTimeMillis() + 6000;
                synchronized (lock) {
                    while (System.currentTimeMillis() < until) {
                        int frames = 0;
                        for (Object e : events) if (e.toString().startsWith("frame")) frames++;
                        if (want > 0 && frames >= want) break;   // want 0: the first event of any kind
                        if (want == 0 && !events.isEmpty()) break;
                        lock.wait(200);
                    }
                }
                boolean wasRunning = p.running();
                p.stop();
                Thread.sleep(300);
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                synchronized (lock) { m.put("events", new ArrayList<Object>(events)); }
                m.put("was_running", wasRunning);
                m.put("stopped", !p.running());
                out = m;
                break;
            }
            case "color": {
                Cockpit.Frame f = new Cockpit(a[1]).colorPng(Long.parseLong(a[2]), 500);
                Map<String, Object> m = new LinkedHashMap<String, Object>();
                m.put("status", f.status);
                m.put("seq", f.seq);
                m.put("w", f.image == null ? null : f.image.getWidth());
                m.put("h", f.image == null ? null : f.image.getHeight());
                out = m;
                break;
            }
            case "get": out = new Cockpit(a[1]).get(a[2], 3000); break;
            case "post": out = new Cockpit(a[1]).post(a[2], Json.parseObject(a[3]), 5000); break;
            case "error": {
                try { new Cockpit(a[1]).get("/api/info", 1000); out = "no error"; }
                catch (Exception e) { out = Cockpit.explain(e, new Cockpit(a[1]).base); }
                break;
            }
            default: throw new IllegalArgumentException(a[0]);
        }
        // bytes, not print(): the JVM's default charset is cp1252 on Windows
        System.out.write(Json.write(out).getBytes("UTF-8"));
        System.out.flush();
        System.exit(0); // the screens start AWT's event thread, which would keep the JVM alive
    }

    static java.awt.image.BufferedImage paint(javax.swing.JComponent c) {
        java.awt.image.BufferedImage out = new java.awt.image.BufferedImage(c.getWidth(), c.getHeight(),
                java.awt.image.BufferedImage.TYPE_INT_RGB);
        java.awt.Graphics2D g = out.createGraphics();
        c.paint(g);
        g.dispose();
        return out;
    }

    /** Lay out without a display: AWT skips re-validating what isn't on screen, so invalidate first. */
    static void layout(java.awt.Component c) {
        invalidateAll(c);
        doLayouts(c);
    }

    static void invalidateAll(java.awt.Component c) {
        c.invalidate();
        if (c instanceof java.awt.Container) {
            for (java.awt.Component k : ((java.awt.Container) c).getComponents()) invalidateAll(k);
        }
    }

    static void doLayouts(java.awt.Component c) {
        c.doLayout();
        if (c instanceof java.awt.Container) {
            for (java.awt.Component k : ((java.awt.Container) c).getComponents()) doLayouts(k);
        }
    }

    /**
     * Every showing component under {@code c}: one that sticks out of the screen (or was laid
     * out with no room) is "clipped" — content the operator could only reach by scrolling,
     * which nothing here does; any scroll pane or scroll bar is named; every label's text.
     */
    static void walk(java.awt.Component root, java.awt.Component c, List<Object> clipped,
            List<Object> scrollers, List<Object> texts) {
        if (!c.isVisible()) return;
        if (c instanceof javax.swing.JScrollPane || c instanceof javax.swing.JScrollBar
                || c instanceof javax.swing.JViewport) {
            scrollers.add(c.getClass().getSimpleName());
        }
        java.awt.Rectangle r = javax.swing.SwingUtilities.convertRectangle(
                c.getParent() == null ? c : c.getParent(), c.getBounds(), root);
        boolean leaf = !(c instanceof java.awt.Container)
                || ((java.awt.Container) c).getComponentCount() == 0;
        if (c instanceof javax.swing.JLabel) texts.add(((javax.swing.JLabel) c).getText());
        if (c instanceof javax.swing.AbstractButton) texts.add(((javax.swing.AbstractButton) c).getText());
        if (leaf && c != root && !(c instanceof javax.swing.Box.Filler)) {
            boolean inside = r.x >= 0 && r.y >= 0 && r.x + r.width <= root.getWidth()
                    && r.y + r.height <= root.getHeight();
            boolean squashed = r.width <= 0 || r.height <= 0;
            if (!inside || squashed) {
                String what = c instanceof javax.swing.JLabel ? ((javax.swing.JLabel) c).getText()
                        : c instanceof javax.swing.AbstractButton ? ((javax.swing.AbstractButton) c).getText()
                        : c.getClass().getName();
                clipped.add(what + " @ " + r.x + "," + r.y + " " + r.width + "x" + r.height);
            }
        }
        if (c instanceof java.awt.Container) {
            for (java.awt.Component k : ((java.awt.Container) c).getComponents()) {
                // only the card that is showing: CardLayout hides the others
                walk(root, k, clipped, scrollers, texts);
            }
        }
    }
}
"""


@pytest.fixture(scope="module")
def java_client(tmp_path_factory):
    if not JAVAC:
        pytest.skip("javac is not installed")
    root = tmp_path_factory.mktemp("java")
    pkg = root / "src" / "io" / "advin" / "perceptronic"
    pkg.mkdir(parents=True)
    (pkg / "Harness.java").write_text(HARNESS, encoding="utf-8")
    for name in (*PURE_JAVA, *SCREEN_JAVA):
        shutil.copy(JAVA / name, pkg / name)
    classes = root / "classes"
    subprocess.run(
        [
            JAVAC,
            "--release",
            "8",
            "-Xlint:-options",
            "-encoding",
            "UTF-8",
            "-d",
            str(classes),
            *map(str, pkg.glob("*.java")),
        ],
        check=True,
        capture_output=True,
        timeout=120,
    )

    def run(*args: str):
        proc = subprocess.run(
            [
                "java",
                "-Djava.awt.headless=true",
                "-cp",
                str(classes),
                "io.advin.perceptronic.Harness",
                *args,
            ],
            capture_output=True,
            timeout=60,
        )
        assert proc.returncode == 0, proc.stderr.decode("utf-8", "replace")
        return json.loads(proc.stdout.decode("utf-8"))

    return run


def test_cockpit_url_rules_match_the_polyscope_x_node(java_client):
    # The PolyScope X node's cockpitBase table, with "this host" = the controller; empty is
    # the pick PC's factory address, so a robot nobody configured needs nothing typed.
    cases = {
        "": "http://192.168.3.20:7621",
        ":7621": "http://127.0.0.1:7621",
        "7621": "http://127.0.0.1:7621",
        " :7622/ ": "http://127.0.0.1:7622",
        "127.0.0.1:7621": "http://127.0.0.1:7621",
        "192.168.3.10": "http://192.168.3.10:7621",
        "jetson.local/": "http://jetson.local:7621",
        "http://127.0.0.1:7621//": "http://127.0.0.1:7621",
        "https://cam.example:8443": "https://cam.example:8443",
    }
    got = java_client("base", *cases)
    assert dict(zip(cases, got, strict=True)) == cases


@pytest.mark.parametrize(
    "text",
    [
        '{"ok": true, "point_m": [0.1, -2e-3, 3.5E1], "s": "a\\"b\\\\c\\u00e9\\n", '
        '"n": null, "e": {}, "l": []}',
        '[1, [2, [3, {"k": [false]}]]]',
        '  {"deep": {"a": {"b": {"c": [1.5, -0.25]}}}}  ',
    ],
)
def test_json_parses_what_python_parses(java_client, text):
    assert java_client("json", text) == json.loads(text)
    # and what it writes, Python reads back to the same value
    assert json.loads(java_client("write", text)) == json.loads(text)


@pytest.mark.parametrize(
    "bad", ['{"a": 1', '{"a" 1}', "[1,,2]", '{"a": tru}', '"\\x"', "{} {}", "", '{"a": "\\u12"}', "-"]
)
def test_json_rejects_malformed_input(java_client, bad):
    assert java_client("json", bad) == "ERR"


def test_six_accepts_only_six_finite_numbers(java_client):
    assert java_client("six", "[1,2,3,4,5,6]") == "ok"
    for bad in ("[1,2,3,4,5]", "[1,2,3,4,5,6,7]", '[1,2,3,4,5,"6"]', "null", "{}"):
        assert java_client("six", bad) is None, bad


def test_target_text_says_which_check_judged_reach(java_client):
    loc = {
        "point_base_m": [-0.224, 0.353, -0.277],
        "point_distance_m": 0.5,
        "reference": "flange",
        "flange_target_pose": [-0.232, 0.31, -0.216, -2.583, 0.566, 0.001],
        "standoff_m": 0.075,
        "reachable": True,
        "reach_check": "controller_ik",
    }
    tips = dict(loc, reference="fingertip", tip_m=0.163, approach_pose=[-0.2, 0.3, -0.2, 0, 3.14, 0])
    assert java_client("target", json.dumps(tips)).splitlines()[1] == (
        "fingertips -0.200, 0.300, -0.200, 0.000, 3.140, 0.000  0.075 m above the object (tool 0.163 m)"
    )
    text = java_client("target", json.dumps(loc))
    assert text.splitlines()[1].startswith("flange   -0.232, 0.310, -0.216")
    assert text.endswith("reachable  (the controller's inverse kinematics)")
    sphere = dict(
        loc,
        reachable=False,
        reach_check="sphere",
        max_reach_m=0.5,
        model="UR3E",
        reference="tcp",
        approach_pose=[0.1, 0.2, 0.3, 0, 3.14, 0],
    )
    text = java_client("target", json.dumps(sphere))
    assert "approach 0.100, 0.200, 0.300" in text
    assert text.endswith("OUT OF REACH  (0.50 m datasheet radius, UR3E — no IK answer)")


def test_a_lost_camera_computer_says_what_to_check_and_the_long_story_goes_to_the_log(java_client):
    """Nick, 2026-09-30: "more helpful suggestions ... like check your firewall, IP address,
    cables, etc. and keep the verbose errors in the logs"."""
    silent = java_client("explain", "timeout", "http://10.0.0.9:7621")
    assert silent["summary"] == "No answer from the camera computer at 10.0.0.9."
    cables, address, firewall = silent["checks"]
    assert cables.startswith("Cables:") and "network cable" in cables
    assert address.startswith("IP address:") and "10.0.0.9" in address
    assert firewall.startswith("Firewall:") and "7621" in firewall and "7622" in firewall
    assert silent["text"] == silent["summary"] + "".join(f"\n• {c}" for c in silent["checks"])
    # nothing a technician needs is lost, and none of it is on the screen
    assert "SocketTimeoutException" in silent["detail"] and "--bind 0.0.0.0" in silent["detail"]
    assert silent["logged"] == [silent["detail"]]
    for noise in ("Exception", "--bind", "perceptronics", "http://"):
        assert noise not in silent["text"]

    itself = java_client("explain", "refused", "http://127.0.0.1:7621")
    assert "robot controller itself" in itself["summary"]
    assert itself["checks"][0].startswith("IP address: 127.0.0.1 is this robot")
    remote = java_client("explain", "refused", "http://192.168.3.10:7621")
    assert "not the camera program" in remote["summary"]
    assert [c.split(":")[0] for c in remote["checks"]] == ["IP address", "Camera program", "Firewall"]
    assert "loopback" in remote["detail"]
    # the pick PC at its factory address and silent: the robot is probably not on its network
    factory = java_client("explain", "timeout", "http://192.168.3.20:7621")
    robot_net = [c for c in factory["checks"] if c.startswith("Robot network:")]
    assert len(robot_net) == 1 and "DHCP" in robot_net[0] and "192.168.3.3" in robot_net[0]
    assert not any(c.startswith("IP address:") for c in factory["checks"])
    dns = java_client("explain", "dns", "http://jetson:7621")
    assert "jetson" in dns["summary"] and dns["checks"][0].startswith("IP address:")
    assert "is not an address" in java_client("explain", "url", "ht!tp://x")["summary"]
    # a real refused connection (nothing listens on port 1) takes the same path
    assert "robot controller itself" in java_client("error", "127.0.0.1:1")


def test_a_camera_computer_with_no_picture_points_at_the_camera_cable_not_the_network(java_client):
    out = java_client("nopicture", "http://192.168.3.10:7621")
    assert out["text"].startswith("The camera computer is on, but its camera gives no picture.")
    assert "USB" in out["text"] and "Firewall" not in out["text"]
    assert "Frame didn't arrive within 5000" in out["logged"][-1] and "5000" not in out["text"]


# -- the Java client against the real cockpit HTTP server ----------------------------------


@pytest.fixture
def cockpit():
    app = ViewerApp(SyntheticRgbdCamera(width=64, height=48, fps=0), config=PerceptionConfig())
    srv = ThreadingHTTPServer(("127.0.0.1", 0), ViewerHandler)
    srv.daemon_threads = True
    srv.app = app  # type: ignore[attr-defined]
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    app.start()
    deadline = time.monotonic() + 5
    while app.latest()[1] is None and time.monotonic() < deadline:
        time.sleep(0.01)
    yield f"127.0.0.1:{srv.server_address[1]}"
    srv.shutdown()
    srv.server_close()
    app.stop()


def test_colour_feed_decodes_and_advances_the_sequence(java_client, cockpit):
    first = java_client("color", cockpit, "0")
    assert first["status"] == 200 and (first["w"], first["h"]) == (64, 48) and first["seq"] > 0
    nxt = java_client("color", cockpit, str(first["seq"]))
    assert nxt["status"] == 200 and nxt["seq"] >= first["seq"]


def test_segment_and_errors_round_trip_as_json(java_client, cockpit):
    seg = java_client("post", cockpit, "/api/segment", json.dumps({"x": 32, "y": 24}))
    assert seg["ok"] is True and "features" in seg
    bad = java_client("post", cockpit, "/api/segment", json.dumps({"x": 9999, "y": 24}))
    assert bad["ok"] is False and bad.get("error")
    # no robot link on this cockpit: locate answers a JSON error, never a crash
    loc = java_client("post", cockpit, "/api/robot/locate", json.dumps({"point_m": [0, 0, 0.4]}))
    assert loc["ok"] is False
    point = java_client("get", cockpit, "/api/point?x=10&y=10")
    assert "ok" in point


# -- the urcap5-v<version> release ---------------------------------------------------------


def test_release_check_publishes_exactly_the_committed_jar_for_its_version():
    version = urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8"))[
        "Bundle-Version"
    ]
    out = urcap5.release_check(f"urcap5-v{version}", SRC, DIST.parent)
    assert out["version"] == version and Path(out["path"]) == DIST
    assert out["sha256"] == hashlib.sha256(DIST.read_bytes()).hexdigest()
    # ... and the stick's auto-install file for that jar, which the release attaches beside it
    assert Path(out["stick"]) == DIST.parent / urcap5.MAGIC_NAME
    assert out["sha256"] in Path(out["stick"]).read_text(encoding="utf-8")


@pytest.mark.parametrize(
    ("tag", "why"),
    [
        ("v0.3.0", "not urcap5-v"),  # the Python package's tags are not this release's
        ("urcap5-v0.3", "not urcap5-v"),
        ("urcap5-v0.3.0-rc1", "not urcap5-v"),
        ("urcap5-v../../etc", "not urcap5-v"),
        ("urcap5-v9.9.9", "Bundle-Version="),
    ],
)
def test_release_check_refuses_a_tag_that_is_not_this_version(tag, why):
    with pytest.raises(urcap5.Urcap5Error, match=why):
        urcap5.release_check(tag, SRC, DIST.parent)


def test_release_check_refuses_a_stale_or_missing_jar(tmp_path):
    version = urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8"))[
        "Bundle-Version"
    ]
    src = tmp_path / "src"
    shutil.copytree(SRC, src)
    with pytest.raises(urcap5.Urcap5Error, match="not committed"):
        urcap5.release_check(f"urcap5-v{version}", src, tmp_path / "empty")
    dist = tmp_path / "dist"
    dist.mkdir()
    shutil.copy(DIST, dist / DIST.name)
    with pytest.raises(urcap5.Urcap5Error, match="urmagic_perceptronic.sh is not committed"):
        urcap5.release_check(f"urcap5-v{version}", src, dist)  # a release without its stick file
    stick = dist / urcap5.MAGIC_NAME
    good = (DIST.parent / urcap5.MAGIC_NAME).read_text(encoding="utf-8")
    stick.write_text(good.replace(hashlib.sha256(DIST.read_bytes()).hexdigest(), "0" * 64), encoding="utf-8")
    with pytest.raises(urcap5.Urcap5Error, match="is not the one for"):
        urcap5.release_check(f"urcap5-v{version}", src, dist)  # ... or one written for another jar
    stick.write_text(good, encoding="utf-8")
    urcap5.release_check(f"urcap5-v{version}", src, dist)  # a faithful copy passes
    java = next((src / "src").rglob("PickScript.java"))
    java.write_text(java.read_text(encoding="utf-8") + "\n// edited after the build\n", encoding="utf-8")
    with pytest.raises(urcap5.Urcap5Error, match="current sources"):
        urcap5.release_check(f"urcap5-v{version}", src, dist)


def test_release_check_cli_prints_json_or_fails_with_the_reason(capsys):
    version = urcap5.read_properties((SRC / "bundle.properties").read_text(encoding="utf-8"))[
        "Bundle-Version"
    ]
    assert (
        urcap5.main(["release-check", f"urcap5-v{version}", "--src", str(SRC), "--dist", str(DIST.parent)])
        == 0
    )
    assert json.loads(capsys.readouterr().out)["version"] == version
    assert urcap5.main(["release-check", "urcap5-v9.9.9", "--src", str(SRC), "--dist", str(DIST.parent)]) == 1
    assert "Bundle-Version=" in capsys.readouterr().err


# -- the mark and the feed poller (0.6.0) ---------------------------------------------------


def test_the_logo_java_carries_the_svg_the_docs_and_polyscope_x_use(java_client):
    """One glyph everywhere: ``urcap/perceptronic.svg`` (the README's mark), the PolyScope X
    node's icon, and the Java constant next to the Java2D drawing of the same path."""
    svg = SVG.read_text(encoding="utf-8")
    assert java_client("svg") == svg
    px_icon = (
        ROOT / "urcap" / "perceptronic" / "perceptronic-frontend" / "assets" / "icons" / "perceptronic.svg"
    )
    assert px_icon.read_text(encoding="utf-8") == svg
    assert 'd="M22 54V10h14a14 14 0 0 1 0 28H22"' in svg and 'cx="36" cy="24" r="5"' in svg


@pytest.mark.parametrize("size", [24, 30, 64])
def test_the_logo_renders_a_p_with_a_lens_on_a_badge(java_client, size):
    got = java_client("logo", str(size))
    assert got["size"] == size
    # a rounded badge: transparent corners, mostly badge, a readable share of white glyph
    assert got["corner_clear"] is True and got["clear"] > 0
    assert got["badge"] > got["white"] > 0.08 * size * size
    # the plain glyph: stem where the SVG's stem is, the lens dot filled, a hole between dot and bowl
    assert got["stem"] and got["lens"] and got["bowl_hole"]
    assert 0.15 * 64 * 64 < got["glyph_ink"] < 0.45 * 64 * 64


def test_feed_poller_streams_frames_and_stops(java_client, cockpit):
    """Against the real cockpit (a synthetic camera): frames arrive in sequence order with
    the fps header, `live` is announced once on the first frame, and stop() ends the thread."""
    got = java_client("poll", cockpit, "3")
    frames = [e for e in got["events"] if e.startswith("frame ")]
    assert len(frames) >= 3, got
    assert re.fullmatch(r"frame \d+x\d+ fps=\d+(\.\d+)?", frames[0]), frames[0]  # its size + X-Fps
    live = [e for e in got["events"] if e.startswith("live ")]
    assert live == [f"live http://{cockpit}"], got["events"]  # once, as Cockpit.base spells it
    assert got["events"].index(live[0]) == 1  # right after the first frame
    assert got["was_running"] and got["stopped"]


def test_feed_poller_explains_a_dead_cockpit_and_keeps_going(java_client):
    got = java_client("poll", "http://127.0.0.1:9", "0")  # port 9: nothing listens
    assert got["events"], got
    assert got["events"][0].startswith("failed Nothing on the robot controller itself answers at"), got[
        "events"
    ]
    assert got["was_running"] and got["stopped"]
