package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.InstallationNodeContribution;
import com.ur.urcap.api.contribution.installation.InstallationAPIProvider;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.userinteraction.robot.movement.MovementCompleteEvent;
import com.ur.urcap.api.domain.userinteraction.robot.movement.MovementErrorEvent;
import com.ur.urcap.api.domain.userinteraction.robot.movement.RobotMovement;
import com.ur.urcap.api.domain.userinteraction.robot.movement.RobotMovementCallback;
import com.ur.urcap.api.domain.value.Pose;
import com.ur.urcap.api.domain.value.ValueFactoryProvider;
import com.ur.urcap.api.domain.value.jointposition.JointPositions;
import com.ur.urcap.api.domain.value.simple.Angle;
import com.ur.urcap.api.domain.value.simple.Length;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardNumberInput;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;
import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import javax.swing.JLabel;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import javax.swing.SwingUtilities;

/**
 * The node's behaviour — the PolyScope X presenter's, on PolyScope 5: poll the cockpit's
 * colour feed while the node is open, hover → depth, tap → segment → locate, and the same
 * five buttons. Network calls run off the Swing thread; results come back through
 * {@link PilotView}'s setters, which marshal onto it.
 */
public class PilotContribution implements InstallationNodeContribution, LocationsScreen.Actions {
    static final String KEY_COCKPIT_URL = "cockpitUrl";
    private static final int HOVER_MS = 150;

    private final InstallationAPIProvider api;
    private final PilotView view;
    private final DataModel model;
    private final ExecutorService actions = Executors.newSingleThreadExecutor(daemon("perceptronic-actions"));

    private volatile Cockpit cockpit;
    private volatile boolean open;
    private final FeedPoller poller;
    private volatile Map<String, Object> located;
    private volatile int[] hoverPending;
    private volatile Thread hoverThread;

    PilotContribution(InstallationAPIProvider api, PilotView view, DataModel model) {
        this.api = api;
        this.view = view;
        this.model = model;
        this.cockpit = new Cockpit(savedUrl());
        this.poller = new FeedPoller(cockpit, feedListener);
    }

    String savedUrl() {
        return model.get(KEY_COCKPIT_URL, "");
    }

    String cockpitBase() {
        return cockpit.base;
    }

    // -- lifecycle --------------------------------------------------------------------------

    @Override
    public void openView() {
        open = true;
        view.showUrl(savedUrl());
        showAreas();
        view.setStatus("connecting to " + cockpit.base + "…", PilotView.Kind.INFO);
        startPolling();
    }

    @Override
    public void closeView() {
        open = false;
        poller.stop();
        Thread h = hoverThread;
        if (h != null) h.interrupt();
    }

    @Override
    public void generateScript(ScriptWriter writer) {
        // the node adds nothing to programs: it is an operator tool, not a program step
    }

    // -- cockpit URL --------------------------------------------------------------------------

    void saveUrl(String value) {
        String v = value == null ? "" : value.trim();
        model.set(KEY_COCKPIT_URL, v);
        cockpit = new Cockpit(v);
        poller.setCockpit(cockpit);
        view.setStatus("cockpit: " + cockpit.base, PilotView.Kind.INFO);
    }

    // -- the feed ----------------------------------------------------------------------------

    private void startPolling() {
        poller.start("perceptronic-feed");
    }

    /** {@link FeedPoller}'s callbacks onto the view (whose setters marshal onto Swing). */
    private final FeedPoller.Listener feedListener = new FeedPoller.Listener() {
        @Override
        public void frame(java.awt.image.BufferedImage image, String fps) {
            view.setFrame(image);
            view.setLive(true, fps);
        }

        @Override
        public void live(String base) {
            view.setStatus("live from " + base, PilotView.Kind.OK);
        }

        @Override
        public void waiting(String why) {
            view.setLive(false, null);
            view.setStatus(why, PilotView.Kind.WARN);
        }

        @Override
        public void failed(String why) {
            view.setLive(false, null);
            view.setStatus(why, PilotView.Kind.ERR);
        }
    };

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    // -- pixels ------------------------------------------------------------------------------

    void onHover(int x, int y) {
        hoverPending = new int[] {x, y};
        Thread h = hoverThread;
        if (h != null && h.isAlive()) return;
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                sleep(HOVER_MS);
                int[] p = hoverPending;
                if (p == null || !open) return;
                try {
                    Map<String, Object> res = cockpit.get("/api/point?x=" + p[0] + "&y=" + p[1], 3000);
                    java.util.List<?> pt = res.get("point_m") instanceof java.util.List ? (java.util.List<?>) res.get("point_m") : null;
                    if (Boolean.TRUE.equals(res.get("ok")) && pt != null && pt.size() == 3) {
                        view.setHover("(" + p[0] + ", " + p[1] + ") → " + Cockpit.fmtVec(pt) + " m in the camera · "
                                + Cockpit.fmt(pt.get(2), 2) + " m away");
                    } else {
                        view.setHover("(" + p[0] + ", " + p[1] + ") → no depth");
                    }
                } catch (IOException e) {
                    // the feed loop reports connectivity
                }
            }
        }, "perceptronic-hover");
        t.setDaemon(true);
        hoverThread = t;
        t.start();
    }

    void onClick(final int x, final int y) {
        located = null;
        view.setMoveEnabled(false, false);
        view.setTarget("");
        view.setStatus("segmenting at (" + x + ", " + y + ")…", PilotView.Kind.INFO);
        actions.submit(new Runnable() {
            @Override
            public void run() {
                try {
                    Map<String, Object> body = new LinkedHashMap<String, Object>();
                    body.put("x", x);
                    body.put("y", y);
                    Map<String, Object> seg = cockpit.post("/api/segment", body, 15000);
                    if (!Boolean.TRUE.equals(seg.get("ok"))) throw new IllegalStateException(errorOf(seg, "segment failed"));
                    Object features = seg.get("features");
                    Object point = features instanceof Map ? ((Map<?, ?>) features).get("point_m") : null;
                    if (point == null) {
                        throw new IllegalStateException("the segment has no depth point — tap the object, not the void");
                    }
                    Map<String, Object> req = new LinkedHashMap<String, Object>();
                    req.put("point_m", point);
                    Map<String, Object> loc = cockpit.post("/api/robot/locate", req, 15000);
                    if (!Boolean.TRUE.equals(loc.get("ok"))) {
                        throw new IllegalStateException(errorOf(loc, "locate failed (is the cockpit's robot link up?)"));
                    }
                    located = loc;
                    view.setTarget(Cockpit.targetText(loc));
                    boolean offer = !Boolean.FALSE.equals(loc.get("reachable"));
                    view.setMoveEnabled(offer && polyscopeCanTake(loc), offer);
                    view.setStatus(offer ? "target located — choose how to move"
                            : "target located but out of reach — pick a nearer point",
                            offer ? PilotView.Kind.OK : PilotView.Kind.WARN);
                } catch (IOException e) {
                    view.setStatus(Cockpit.explain(e, cockpit.base), PilotView.Kind.ERR);
                } catch (RuntimeException e) {
                    view.setStatus(String.valueOf(e.getMessage()), PilotView.Kind.ERR);
                }
            }
        });
    }

    private static String errorOf(Map<String, Object> res, String fallback) {
        Object e = res.get("error");
        return e == null ? fallback : e.toString();
    }

    // -- moving ------------------------------------------------------------------------------

    /**
     * PolyScope's own hold-to-move screen — works in Local mode. The target is the
     * controller's joint solution for the flange target when the cockpit got one
     * (``joint_target``: get_inverse_kin, a script, so Remote only); otherwise
     * ``polyscope_pose``, the flange target expressed in PolyScope's active TCP (from the
     * state broadcast, no script), which PolyScope solves itself. Never the approach pose:
     * that is the fingertip TCP's pose, and PolyScope would put its active TCP there.
     */
    void movePolyScope() {
        final Map<String, Object> loc = located;
        if (loc == null) return;
        final RobotMovement movement = api.getUserInterfaceAPI().getUserInteraction().getRobotMovement();
        final ValueFactoryProvider values = api.getInstallationAPI().getValueFactoryProvider();
        final double[] q = Cockpit.six(loc.get("joint_target"));
        final double[] p = Cockpit.six(loc.get("polyscope_pose"));
        if (q == null && p == null) {
            Object note = loc.get("polyscope_pose_note");
            view.setStatus("PolyScope can't be given this target: " + (note != null ? note
                    : "no joint solution and no pose in PolyScope's TCP (update the cockpit)"), PilotView.Kind.ERR);
            return;
        }
        final RobotMovementCallback done = new RobotMovementCallback() {
            @Override
            public void onComplete(MovementCompleteEvent event) {
                view.setStatus("PolyScope move finished", PilotView.Kind.OK);
            }

            @Override
            public void onCancel(com.ur.urcap.api.domain.userinteraction.robot.movement.MovementCancelEvent event) {
                view.setStatus("PolyScope move cancelled", PilotView.Kind.WARN);
            }

            @Override
            public void onError(MovementErrorEvent event) {
                view.setStatus("PolyScope move: " + event.getErrorType(), PilotView.Kind.ERR);
            }
        };
        SwingUtilities.invokeLater(new Runnable() {
            @Override
            public void run() {
                try {
                    if (q != null) {
                        JointPositions joints = values.getJointPositionFactory()
                                .createJointPositions(q[0], q[1], q[2], q[3], q[4], q[5], Angle.Unit.RAD);
                        view.setStatus("PolyScope's move screen is open — hold Move to go to the joint solution",
                                PilotView.Kind.OK);
                        movement.requestUserToMoveRobot(joints, done);
                    } else {
                        Pose pose = values.getPoseFactory()
                                .createPose(p[0], p[1], p[2], p[3], p[4], p[5], Length.Unit.M, Angle.Unit.RAD);
                        view.setStatus("PolyScope's move screen is open — hold Move (the target in the active TCP)",
                                PilotView.Kind.OK);
                        movement.requestUserToMoveRobot(pose, done);
                    }
                } catch (RuntimeException e) {
                    view.setStatus("PolyScope move: " + e.getMessage(), PilotView.Kind.ERR);
                }
            }
        });
    }

    void moveCockpit() {
        final Map<String, Object> loc = located;
        if (loc == null) return;
        view.setMoveEnabled(true, false);
        view.setStatus("moving through the cockpit (Primary, safety-enveloped)…", PilotView.Kind.INFO);
        actions.submit(new Runnable() {
            @Override
            public void run() {
                try {
                    Map<String, Object> body = new LinkedHashMap<String, Object>();
                    body.put("pose", loc.get("approach_pose"));
                    body.put("velocity", 0.1);
                    Map<String, Object> res = cockpit.post("/api/robot/move", body, 60000);
                    if (Boolean.TRUE.equals(res.get("ok"))) {
                        view.setStatus("landed at " + Cockpit.fmtVec(res.get("landed")), PilotView.Kind.OK);
                    } else {
                        view.setStatus("move refused: " + refusal(res), PilotView.Kind.ERR);
                    }
                } catch (IOException e) {
                    view.setStatus("move: " + Cockpit.explain(e, cockpit.base), PilotView.Kind.ERR);
                } finally {
                    Map<String, Object> now = located;
                    view.setMoveEnabled(now != null && polyscopeCanTake(now), now != null);
                }
            }
        });
    }

    /** A joint solution, or a pose in PolyScope's own active TCP — what its move screen can take. */
    static boolean polyscopeCanTake(Map<String, Object> loc) {
        return Cockpit.six(loc.get("joint_target")) != null || Cockpit.six(loc.get("polyscope_pose")) != null;
    }

    private static String refusal(Map<String, Object> res) {
        if (res.get("error") != null) return res.get("error").toString();
        Object safety = res.get("safety");
        if (safety instanceof Map && ((Map<?, ?>) safety).get("violations") != null) {
            return ((Map<?, ?>) safety).get("violations").toString();
        }
        if (Boolean.TRUE.equals(res.get("protective_stop"))) return "protective stop — Bring up, then retry";
        return "the move did not confirm";
    }

    void bringUp() {
        view.setStatus("bringing the robot up…", PilotView.Kind.INFO);
        actions.submit(new Runnable() {
            @Override
            public void run() {
                try {
                    Map<String, Object> res = cockpit.post("/api/robot/bring_up", new LinkedHashMap<String, Object>(), 180000);
                    boolean ok = Boolean.TRUE.equals(res.get("ok"));
                    view.setStatus(ok ? "robot " + orElse(res.get("robot_mode"), "up") + " / " + orElse(res.get("safety_mode"), "")
                            : "bring-up failed: " + orElse(res.get("error"), "?"), ok ? PilotView.Kind.OK : PilotView.Kind.ERR);
                } catch (IOException e) {
                    view.setStatus("bring-up: " + Cockpit.explain(e, cockpit.base), PilotView.Kind.ERR);
                }
            }
        });
    }

    /** STOP skips the action queue: it must not wait behind a move that is still running. */
    void stop() {
        Thread t = new Thread(new Runnable() {
            @Override
            public void run() {
                try {
                    Map<String, Object> res = cockpit.post("/api/robot/stop", new LinkedHashMap<String, Object>(), 10000);
                    boolean ok = Boolean.TRUE.equals(res.get("ok"));
                    view.setStatus(ok ? "stopped" : "stop failed: " + orElse(res.get("error"), "?"),
                            ok ? PilotView.Kind.WARN : PilotView.Kind.ERR);
                } catch (IOException e) {
                    view.setStatus("stop: " + Cockpit.explain(e, cockpit.base), PilotView.Kind.ERR);
                }
            }
        }, "perceptronic-stop");
        t.setDaemon(true);
        t.start();
    }

    void clear() {
        located = null;
        view.clearMark();
        view.setTarget("");
        view.setMoveEnabled(false, false);
        view.setStatus("cleared", PilotView.Kind.INFO);
    }

    // -- pick areas: patches of the table taught with the fingertips ---------------------------

    static final String KEY_AREAS = "areas";
    static final String KEY_AREA_SELECTED = "areaSelected";
    static final String KEY_TIP_MM = "tipMm";
    static final String KEY_REACH_INNER = "reachInnerMm";
    static final String KEY_REACH_OUTER = "reachOuterMm";
    static final int MAX_AREAS = 8;
    static final double DEFAULT_TIP_MM = 163; // Hand-E 157 mm + the 6 mm adapter (the UR3e cell)

    int areaCount() {
        return Math.max(0, Math.min(MAX_AREAS, model.get(KEY_AREAS, 0)));
    }

    String areaName(int i) {
        return i < 0 || i >= areaCount() ? null : model.get("area." + i + ".name", "Area " + (i + 1));
    }

    /** Area {@code i} as {@code [pose(6), sizeX, sizeY]} (m) once its three points are taught, else null. */
    double[] areaPlane(int i) {
        if (i < 0 || i >= areaCount()) return null;
        double[][] p = new double[3][];
        for (int k = 0; k < 3; k++) {
            p[k] = model.get("area." + i + ".p" + k, (double[]) null);
            if (p[k] == null || p[k].length != 3) return null;
        }
        return PoseMath.plane(p[0], p[1], p[2]);
    }

    int firstTaughtArea() {
        for (int i = 0; i < areaCount(); i++) {
            if (areaPlane(i) != null) return i;
        }
        return -1;
    }

    /** {base radius, rated reach} (m) of this robot, or null when the model isn't in the table. */
    double[] robotReach() {
        try {
            return PickScript.modelReach(api.getSystemAPI().getRobotModel().getRobotType().name());
        } catch (RuntimeException e) {
            return null;
        }
    }

    String modelName() {
        try {
            return api.getSystemAPI().getRobotModel().getRobotType().name();
        } catch (RuntimeException e) {
            return "?";
        }
    }

    /** {min, max} radial reach (m) for the program, or null when the robot model is unknown. */
    double[] reachLimits() {
        double[] m = robotReach();
        if (m == null) return null;
        double min = m[0] + model.get(KEY_REACH_INNER, 150.0) / 1000.0;
        double max = Math.max(0, m[1] - model.get(KEY_REACH_OUTER, 150.0) / 1000.0);
        return max > min ? new double[] {min, max} : new double[] {min, 0};
    }

    private void showAreas() {
        List<LocationsScreen.Area> out = new ArrayList<LocationsScreen.Area>();
        for (int i = 0; i < areaCount(); i++) {
            boolean[] t = new boolean[3];
            for (int k = 0; k < 3; k++) t[k] = model.get("area." + i + ".p" + k, (double[]) null) != null;
            out.add(new LocationsScreen.Area(areaName(i), t, areaPlane(i)));
        }
        double[] m = robotReach();
        view.areas().show(out, model.get(KEY_AREA_SELECTED, 0), modelName(), m == null ? 0.064 : m[0],
                m == null ? 0 : m[1], model.get(KEY_REACH_INNER, 150.0), model.get(KEY_REACH_OUTER, 150.0),
                model.get(KEY_TIP_MM, DEFAULT_TIP_MM));
    }

    @Override
    public void addArea() {
        int n = areaCount();
        if (n >= MAX_AREAS) return;
        model.set("area." + n + ".name", "Area " + (n + 1));
        model.set(KEY_AREAS, n + 1);
        model.set(KEY_AREA_SELECTED, n);
        showAreas();
        view.setStatus("new pick area: teach its corner, a point along one edge and one on the far side",
                PilotView.Kind.INFO);
    }

    /** PolyScope's move screen: touch the table with the fingertips, then OK. */
    @Override
    public void teach(final int area, final int point) {
        TeachPosition.teacher(modelName()).teach(api.getUserInterfaceAPI().getUserInteraction(), new TeachPosition.Done() {
            @Override
            public void taught(JointPositions joints, double[] flange) {
                if (flange == null) {
                    view.setStatus("this PolyScope (before 5.8) can't say where the " + modelName() + "'s flange is:"
                            + " leave the area untaught (the table is found live) or teach it on PolyScope 5.8+",
                            PilotView.Kind.ERR);
                    return;
                }
                double[] tip = PoseMath.tipOf(flange, model.get(KEY_TIP_MM, DEFAULT_TIP_MM) / 1000.0);
                model.set("area." + area + ".p" + point, tip);
                model.set(KEY_AREA_SELECTED, area);
                showAreas();
                double[] plane = areaPlane(area);
                view.setStatus(plane != null ? areaName(area) + " is taught" : LocationsScreen.POINTS[point]
                        + " taught — " + (point < 2 ? LocationsScreen.POINT_HELP[point + 1] : "check the other points"),
                        PilotView.Kind.OK);
            }
        });
    }

    @Override
    public void rename(final int area, JLabel anchor) {
        KeyboardTextInput kb = api.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory()
                .createStringKeyboardInput();
        kb.setInitialValue(areaName(area));
        kb.show(anchor, new KeyboardInputCallback<String>() {
            @Override
            public void onOk(String value) {
                String v = value == null ? "" : value.replaceAll("[^A-Za-z0-9 ._-]", "").trim();
                if (v.isEmpty()) return;
                model.set("area." + area + ".name", v.length() > 24 ? v.substring(0, 24) : v);
                showAreas();
            }
        });
    }

    @Override
    public void remove(int area) {
        int n = areaCount();
        if (area < 0 || area >= n) return;
        for (int i = area; i < n - 1; i++) {
            model.set("area." + i + ".name", model.get("area." + (i + 1) + ".name", "Area " + (i + 1)));
            for (int k = 0; k < 3; k++) {
                double[] p = model.get("area." + (i + 1) + ".p" + k, (double[]) null);
                if (p != null) model.set("area." + i + ".p" + k, p);
                else model.remove("area." + i + ".p" + k);
            }
        }
        model.remove("area." + (n - 1) + ".name");
        for (int k = 0; k < 3; k++) model.remove("area." + (n - 1) + ".p" + k);
        model.set(KEY_AREAS, n - 1);
        model.set(KEY_AREA_SELECTED, Math.max(0, area - 1));
        showAreas();
        view.setStatus("pick area removed — picture points that looked at it now find the table live, or the next"
                + " area; check them", PilotView.Kind.WARN);
    }

    @Override
    public void select(int area) {
        model.set(KEY_AREA_SELECTED, area);
        showAreas();
    }

    @Override
    public void setReach(String key, double delta) {
        double def = KEY_TIP_MM.equals(key) ? DEFAULT_TIP_MM : 150.0;
        double v = clampReach(key, model.get(key, def) + delta);
        model.set(key, v);
        showAreas();
    }

    @Override
    public void askReach(final String key, JLabel anchor) {
        KeyboardNumberInput<Double> kb = api.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory()
                .createPositiveDoubleKeypadInput();
        kb.setInitialValue(model.get(key, KEY_TIP_MM.equals(key) ? DEFAULT_TIP_MM : 150.0));
        kb.show(anchor, new KeyboardInputCallback<Double>() {
            @Override
            public void onOk(Double value) {
                if (value == null) return;
                model.set(key, clampReach(key, value));
                showAreas();
            }
        });
    }

    static double clampReach(String key, double v) {
        if (KEY_TIP_MM.equals(key)) return Math.max(0, Math.min(500, Math.round(v)));
        return Math.max(0, Math.min(1000, Math.round(v)));
    }

    private static String orElse(Object v, String fallback) {
        return v == null ? fallback : v.toString();
    }

    private static ThreadFactory daemon(final String name) {
        return new ThreadFactory() {
            @Override
            public Thread newThread(Runnable r) {
                Thread t = new Thread(r, name);
                t.setDaemon(true);
                return t;
            }
        };
    }
}
