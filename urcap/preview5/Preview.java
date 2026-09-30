package com.nickarmenta.perceptronic;

import java.awt.BorderLayout;
import java.awt.Dimension;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.io.File;
import java.net.URLEncoder;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import javax.imageio.ImageIO;
import javax.swing.BorderFactory;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * The Perceptronic Pick node's and the Installation node's screens in a desktop window, driven by
 * a live camera computer — the pendant's own Swing classes ({@link PickScreen},
 * {@link LocationsScreen}), with desktop stand-ins for what only PolyScope provides: typed
 * values come from a dialog instead of PolyScope's keypad, a picture point is added where the
 * robot "is" (no arm here: its joints are placeholders), and a pick area's three touches are
 * sample points on the demo table. Everything drawn on the picture is the camera computer's
 * real answer ({@code GET /api/pick/scene}) to exactly the options the program would send.
 *
 * <p>Not part of the URCap (it is outside {@code perceptronic-ps5/src}). Run it with
 * {@code uv run python urcap/preview5.py}.
 *
 * <pre>java … Preview http://127.0.0.1:7650 [--snapshot out.png]</pre>
 */
public final class Preview {
    private static final double[] PLACEHOLDER_Q = {0, -1.0, 1.2, -1.8, -1.5708, 0};
    // the demo table's sample touches: three rectangles under the fake camera (z = -0.27 m)
    private static final double[][][] SAMPLE_AREAS = {
        {{0.20, -0.11, -0.27}, {0.37, -0.11, -0.27}, {0.20, 0.03, -0.27}},
        {{0.20, -0.11, -0.27}, {0.27, -0.11, -0.27}, {0.20, 0.11, -0.27}},
        {{0.28, -0.11, -0.27}, {0.38, -0.11, -0.27}, {0.28, 0.11, -0.27}},
    };

    final Cockpit cockpit;
    final PickScript s = new PickScript();
    final List<Integer> pointArea = new ArrayList<Integer>(); // per picture point: area index or -1
    final List<String> areaNames = new ArrayList<String>();
    final List<double[][]> areaTouches = new ArrayList<double[][]>(); // 3 points each, null until taught
    int selected;
    int selectedArea;
    boolean perPoint;
    double innerMm = 150;
    double outerMm = 150;
    double tipMm = 163;
    PickScreen pick;
    LocationsScreen areas;
    volatile long seq;

    Preview(String base) {
        cockpit = new Cockpit(base);
        s.host = PickScript.hostOf(cockpit.base);
        s.nodeId = "preview";
    }

    public static void main(String[] a) throws Exception {
        final Preview p = new Preview(a.length > 0 ? a[0] : "http://127.0.0.1:7650");
        final String snapshot = a.length > 2 && "--snapshot".equals(a[1]) ? a[2] : null;
        final JPanel[] frame = new JPanel[1];
        SwingUtilities.invokeAndWait(() -> frame[0] = p.window(snapshot == null));
        Thread poll = new Thread(p::poll, "preview-feed");
        poll.setDaemon(true);
        poll.start();
        if (snapshot != null) {
            Thread.sleep(4000);
            final BufferedImage[] out = new BufferedImage[1];
            SwingUtilities.invokeAndWait(() -> {
                JPanel root = frame[0];
                layout(root);
                out[0] = new BufferedImage(root.getWidth(), root.getHeight(), BufferedImage.TYPE_INT_RGB);
                Graphics2D g = out[0].createGraphics();
                root.paint(g);
                g.dispose();
            });
            ImageIO.write(out[0], "png", new File(snapshot));
            System.exit(0);
        }
    }

    /** The screens in one panel; in a 1280 × 800 window when {@code show} (headless otherwise). */
    private JPanel window(boolean show) {
        pick = new PickScreen(new PickActions());
        areas = new LocationsScreen(new AreaActions());
        final JPanel deck = new JPanel(new java.awt.CardLayout());
        JPanel areasHost = new JPanel(new BorderLayout());
        areasHost.setBackground(Ui.BG);
        areasHost.setBorder(BorderFactory.createEmptyBorder(8, 12, 8, 12));
        areasHost.add(areas);
        deck.add(pick, "pick");
        deck.add(areasHost, "areas");
        JPanel top = new JPanel(new BorderLayout());
        top.setBackground(Ui.BG);
        top.setBorder(BorderFactory.createEmptyBorder(8, 12, 0, 12));
        top.add(Ui.label("Preview — " + cockpit.base + "   (desktop stand-ins for the arm and PolyScope's keypad)",
                12f, false, Ui.MUTED), BorderLayout.WEST);
        Ui.Segmented tabs = new Ui.Segmented(new String[] {"Program: Perceptronic Pick", "Installation: pick areas"}, 0,
                i -> ((java.awt.CardLayout) deck.getLayout()).show(deck, i == 0 ? "pick" : "areas"));
        tabs.setPreferredSize(new Dimension(460, 40));
        top.add(tabs, BorderLayout.EAST);
        JPanel root = new JPanel(new BorderLayout());
        root.setBackground(Ui.BG);
        root.add(top, BorderLayout.NORTH);
        root.add(deck, BorderLayout.CENTER);
        if (show) {
            JFrame f = new JFrame("Perceptronic Pick " + PickScript.VERSION + " — preview");
            f.setContentPane(root);
            f.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            f.setSize(1280, 800); // the e-Series pendant's screen
            f.setLocationRelativeTo(null);
            f.setVisible(true);
        } else {
            root.setSize(1280, 772);
            layout(root);
        }
        pickDefaults();
        refresh();
        pick.setStatus("preview: tap + Add picture point, try the order tiles and Options — the numbers are the"
                + " camera computer's answer", Ui.Kind.INFO);
        return root;
    }

    private void pickDefaults() {
        // start with one picture point on the live table and the first sample area taught
        s.points.add(new PickScript.Point(PLACEHOLDER_Q, null, 0, 0));
        pointArea.add(-1);
        areaNames.add("Infeed tray");
        areaTouches.add(SAMPLE_AREAS[0]);
    }

    /** Lay out without a display: AWT skips re-validating what isn't on screen, so invalidate first. */
    private static void layout(java.awt.Component c) {
        invalidateAll(c);
        doLayouts(c);
    }

    private static void invalidateAll(java.awt.Component c) {
        c.invalidate();
        if (c instanceof java.awt.Container) {
            for (java.awt.Component k : ((java.awt.Container) c).getComponents()) invalidateAll(k);
        }
    }

    private static void doLayouts(java.awt.Component c) {
        c.doLayout();
        if (c instanceof java.awt.Container) {
            for (java.awt.Component k : ((java.awt.Container) c).getComponents()) doLayouts(k);
        }
    }

    // -- the picture -------------------------------------------------------------------------

    private void poll() {
        long sceneAt = 0;
        while (true) {
            try {
                Cockpit.Frame fr = cockpit.colorPng(seq, 1500);
                if (fr.status != 200) {
                    pick.setLive(false, null);
                    Thread.sleep(800);
                    continue;
                }
                seq = fr.seq;
                pick.setFrame(fr.image);
                pick.setLive(true, fr.fps);
                if (System.currentTimeMillis() - sceneAt > 500) {
                    sceneAt = System.currentTimeMillis();
                    rebuild();
                    Map<String, Object> res = cockpit.get("/api/pick/scene?opts="
                            + URLEncoder.encode(s.tokens(s.points.isEmpty() ? -1 : selected), "UTF-8"), 5000);
                    pick.setScene(Scene.parse(res));
                    if (!Boolean.TRUE.equals(res.get("ok"))) {
                        pick.setStatus("camera computer: " + res.get("error"), Ui.Kind.WARN);
                    }
                }
            } catch (Exception e) {
                pick.setLive(false, null);
                pick.setStatus(Cockpit.explain(e, cockpit.base), Ui.Kind.ERR);
                try {
                    Thread.sleep(1500);
                } catch (InterruptedException ie) {
                    return;
                }
            }
        }
    }

    // -- the model -------------------------------------------------------------------------------

    double[] plane(int area) {
        if (area < 0 || area >= areaTouches.size() || areaTouches.get(area) == null) return null;
        double[][] t = areaTouches.get(area);
        return PoseMath.plane(t[0], t[1], t[2]);
    }

    /** The script's picture points and reach from the preview's own state. */
    synchronized void rebuild() {
        List<PickScript.Point> pts = new ArrayList<PickScript.Point>();
        for (int i = 0; i < pointArea.size(); i++) {
            double[] pl = plane(pointArea.get(i));
            pts.add(new PickScript.Point(PLACEHOLDER_Q, pl == null ? null : java.util.Arrays.copyOf(pl, 6),
                    pl == null ? 0 : pl[6] * 1000, pl == null ? 0 : pl[7] * 1000));
        }
        s.points.clear();
        s.points.addAll(pts);
        double[] m = PickScript.modelReach("UR3");
        s.reachMinM = m[0] + innerMm / 1000;
        s.reachMaxM = Math.max(0, m[1] - outerMm / 1000);
    }

    synchronized void refresh() {
        rebuild();
        List<PickScreen.PointRow> rows = new ArrayList<PickScreen.PointRow>();
        for (int a : pointArea) {
            rows.add(plane(a) != null ? new PickScreen.PointRow(areaNames.get(a), true)
                    : new PickScreen.PointRow("live table", false));
        }
        selected = Math.max(0, Math.min(selected, pointArea.size() - 1));
        pick.show(s, rows, selected, perPoint);
        List<LocationsScreen.Area> list = new ArrayList<LocationsScreen.Area>();
        for (int i = 0; i < areaNames.size(); i++) {
            boolean done = areaTouches.get(i) != null;
            list.add(new LocationsScreen.Area(areaNames.get(i), new boolean[] {done, done, done}, plane(i)));
        }
        double[] m = PickScript.modelReach("UR3");
        areas.show(list, selectedArea, "UR3e", m[0], m[1], innerMm, outerMm, tipMm);
    }

    private static Double ask(String what, double now) {
        String v = JOptionPane.showInputDialog(null, what, PickScript.num(now));
        if (v == null) return null;
        try {
            return Double.parseDouble(v.trim());
        } catch (NumberFormatException e) {
            return null;
        }
    }

    // -- the Pick node's taps ----------------------------------------------------------------

    private final class PickActions implements PickScreen.Actions {
        @Override
        public void addPoint() {
            synchronized (Preview.this) {
                if (pointArea.size() >= PickScript.MAX_POINTS) return;
                pointArea.add(pointArea.isEmpty() ? -1 : pointArea.get(pointArea.size() - 1));
                selected = pointArea.size() - 1;
            }
            refresh();
            pick.setStatus("picture " + (selected + 1) + " added (on the robot: where the arm is, via PolyScope's"
                    + " move screen)", Ui.Kind.OK);
        }

        @Override
        public void goTo(int i) {
            pick.setStatus("on the robot this opens PolyScope's move screen: hold Move to go to picture " + (i + 1),
                    Ui.Kind.INFO);
        }

        @Override
        public void retake(int i) {
            pick.setStatus("on the robot: picture " + (i + 1) + " retaken where the arm is now", Ui.Kind.INFO);
        }

        @Override
        public void remove(int i) {
            synchronized (Preview.this) {
                if (i >= 0 && i < pointArea.size()) pointArea.remove(i);
            }
            refresh();
        }

        @Override
        public void select(int i) {
            selected = i;
            refresh();
        }

        @Override
        public void cycleArea(int i) {
            synchronized (Preview.this) {
                int next = pointArea.get(i);
                for (int step = 0; step <= areaNames.size(); step++) {
                    next = next + 1 >= areaNames.size() ? -1 : next + 1;
                    if (next == -1 || plane(next) != null) break;
                }
                pointArea.set(i, next);
                selected = i;
            }
            refresh();
            int a = pointArea.get(i);
            pick.setStatus(a < 0 ? "picture " + (i + 1) + " finds the table live" : "picture " + (i + 1)
                    + " looks at " + areaNames.get(a) + " — parts outside it are dashed: outside the pick area",
                    Ui.Kind.OK);
        }

        @Override
        public void setOrder(String first, String rows) {
            s.orderFirst = first;
            s.orderRows = rows;
            refresh();
            pick.setStatus("pick order: " + PickScript.orderText(first, rows), Ui.Kind.OK);
        }

        @Override
        public void setNumber(String key, double value) {
            s.set(key, value);
            refresh();
        }

        @Override
        public void askNumber(String key, JLabel anchor) {
            PickScript.Num n = PickScript.BY_KEY.get(key);
            Double v = ask(n.label + " (" + n.unit + ", " + PickScript.num(n.min) + ".." + PickScript.num(n.max) + ")",
                    s.n(key));
            if (v != null) setNumber(key, v);
        }

        @Override
        public void setGripper(String mode) {
            s.gripper = mode;
            refresh();
        }

        @Override
        public void setFlag(String key, boolean on) {
            if (PickScreen.FLAG_POPUP.equals(key)) s.popupOnFail = on;
            else perPoint = on;
            refresh();
        }

        @Override
        public void checkApproach() {
            new Thread(() -> {
                try {
                    Map<String, Object> res = cockpit.get("/api/pick/scene?opts=" + URLEncoder.encode(s.tokens(selected),
                            "UTF-8") + "&approach_mm=" + PickScript.num(s.n("approachMm")), 5000);
                    Object parts = res.get("parts");
                    Object first = parts instanceof List && !((List<?>) parts).isEmpty() ? ((List<?>) parts).get(0) : null;
                    double[] p = first instanceof Map ? Cockpit.six(((Map<?, ?>) first).get("polyscope_approach_pose"))
                            : null;
                    pick.setStatus(p == null ? "no part #1 to approach" : String.format(Locale.ROOT,
                            "on the robot PolyScope's move screen would take the TCP to p[%.3f, %.3f, %.3f, %.3f, %.3f,"
                            + " %.3f] — fingertips %s mm over part #1, fingers open", p[0], p[1], p[2], p[3], p[4], p[5],
                            PickScript.num(s.n("approachMm"))), p == null ? Ui.Kind.WARN : Ui.Kind.OK);
                } catch (Exception e) {
                    pick.setStatus(Cockpit.explain(e, cockpit.base), Ui.Kind.ERR);
                }
            }, "preview-check").start();
        }

        @Override
        public void resetDefaults() {
            for (PickScript.Num n : PickScript.NUMBERS) s.values.put(n.key, n.def);
            s.popupOnFail = true;
            refresh();
        }
    }

    // -- the Installation's taps ----------------------------------------------------------------

    private final class AreaActions implements LocationsScreen.Actions {
        @Override
        public void addArea() {
            synchronized (Preview.this) {
                if (areaNames.size() >= 8) return;
                areaNames.add("Area " + (areaNames.size() + 1));
                areaTouches.add(null);
                selectedArea = areaNames.size() - 1;
            }
            refresh();
        }

        @Override
        public void teach(int area, int point) {
            // no arm to touch the table with: the three touches of a sample rectangle
            synchronized (Preview.this) {
                areaTouches.set(area, SAMPLE_AREAS[area % SAMPLE_AREAS.length]);
                selectedArea = area;
            }
            refresh();
        }

        @Override
        public void rename(int area, JLabel anchor) {
            String v = JOptionPane.showInputDialog(null, "Name", areaNames.get(area));
            if (v != null && !v.trim().isEmpty()) {
                areaNames.set(area, v.trim());
                refresh();
            }
        }

        @Override
        public void remove(int area) {
            synchronized (Preview.this) {
                areaNames.remove(area);
                areaTouches.remove(area);
                for (int i = 0; i < pointArea.size(); i++) {
                    int a = pointArea.get(i);
                    pointArea.set(i, a == area ? -1 : a > area ? a - 1 : a);
                }
                selectedArea = 0;
            }
            refresh();
        }

        @Override
        public void select(int area) {
            selectedArea = area;
            refresh();
        }

        @Override
        public void setReach(String key, double delta) {
            if ("reachInnerMm".equals(key)) innerMm = Math.max(0, innerMm + delta);
            else if ("reachOuterMm".equals(key)) outerMm = Math.max(0, outerMm + delta);
            else tipMm = Math.max(0, tipMm + delta);
            refresh();
        }

        @Override
        public void askReach(String key, JLabel anchor) {
            double now = "reachInnerMm".equals(key) ? innerMm : "reachOuterMm".equals(key) ? outerMm : tipMm;
            Double v = ask(key, now);
            if (v != null) setReach(key, v - now);
        }
    }
}
