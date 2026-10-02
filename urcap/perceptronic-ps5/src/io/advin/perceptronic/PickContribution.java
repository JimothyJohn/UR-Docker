package io.advin.perceptronic;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.program.CreationContext;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.undoredo.UndoableChanges;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardNumberInput;
import com.ur.urcap.api.domain.userinteraction.robot.movement.MovementCancelEvent;
import com.ur.urcap.api.domain.userinteraction.robot.movement.MovementCompleteEvent;
import com.ur.urcap.api.domain.userinteraction.robot.movement.MovementErrorEvent;
import com.ur.urcap.api.domain.userinteraction.robot.movement.RobotMovementCallback;
import com.ur.urcap.api.domain.value.Pose;
import com.ur.urcap.api.domain.value.jointposition.JointPosition;
import com.ur.urcap.api.domain.value.jointposition.JointPositions;
import com.ur.urcap.api.domain.value.simple.Angle;
import com.ur.urcap.api.domain.value.simple.Length;
import com.ur.urcap.api.domain.variable.Variable;
import java.io.IOException;
import java.net.URLEncoder;
import java.security.SecureRandom;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import javax.swing.JLabel;
import javax.swing.SwingUtilities;

/**
 * One 3D Pick node in a program (0.7.0): a single move sequence with no children. Teach time —
 * the node's screen open — the live picture (or the depth), what is found in it
 * ({@code GET /api/pick/scene}, asked with exactly the options the program will send), the
 * picture points, the pick order, and the Options view. Run time: {@link PickScript}.
 * Everything is in the node's data model; the camera computer's address and the pick areas
 * come from the Installation node, one per robot.
 */
public class PickContribution implements ProgramNodeContribution, PickScreen.Actions {
    static final String KEY_NODE_ID = "nodeId";
    static final String KEY_TEMPLATED = "templated";
    static final String KEY_POINTS = "points";
    static final String KEY_SELECTED = "selectedPoint";
    static final String KEY_ORDER_FIRST = "orderFirst";
    static final String KEY_ORDER_ROWS = "orderRows";
    static final String KEY_SHAPE = "partShape";
    static final String KEY_GRIP_CHECK = "gripCheck";
    static final String KEY_GRIP_LONG = "gripLongSide";
    static final String KEY_CLOSE_LOOK = "closeLook";
    static final String KEY_POPUP = "popupOnFail";
    static final String KEY_PORT = "pickPort";
    static final String KEY_VARIABLE = "foundVariable";
    static final String KEY_LOC_VARIABLE = "locVariable";
    static final String KEY_SURVEY_040 = "surveyJoints"; // 0.4.0's single survey position
    static final String FOUND_NAME = "rs_pick_found";
    static final String LOC_NAME = "rs_pick_loc";
    private static final int POLL_TIMEOUT_MS = 1500;
    private static final long SCENE_EVERY_MS = 700;

    private final ProgramAPIProvider api;
    private final PickView view;
    private final DataModel model;
    private final ExecutorService actions = Executors.newSingleThreadExecutor(r -> {
        Thread t = new Thread(r, "realsense-pick-actions");
        t.setDaemon(true);
        return t;
    });

    private volatile boolean open;
    private volatile Thread poller;
    private volatile long seq;
    private volatile long sceneAt;
    private volatile boolean depthView;
    private volatile Scene scene = Scene.empty();

    PickContribution(ProgramAPIProvider api, PickView view, DataModel model, CreationContext context) {
        this.api = api;
        this.view = view;
        this.model = model;
    }

    // -- where things come from ---------------------------------------------------------------

    PilotContribution installation() {
        return api.getProgramAPI().getInstallationNode(PilotContribution.class);
    }

    /** This PolyScope's {major, minor, bugfix}, or null when it won't say (the script then assumes the newest). */
    int[] polyscopeVersion() {
        try {
            com.ur.urcap.api.domain.SoftwareVersion v = api.getSystemAPI().getSoftwareVersion();
            return new int[] {v.getMajorVersion(), v.getMinorVersion(), v.getBugfixVersion()};
        } catch (RuntimeException e) {
            return null;
        }
    }

    /** PolyScope's name for this arm ("UR3", ...), or "?" when it won't say. */
    String robotType() {
        try {
            return api.getSystemAPI().getRobotModel().getRobotType().name();
        } catch (RuntimeException e) {
            return "?";
        }
    }

    Cockpit cockpit() {
        PilotContribution i = installation();
        return new Cockpit(i == null ? "" : i.savedUrl());
    }

    int pointCount() {
        return Math.max(0, Math.min(PickScript.MAX_POINTS, model.get(KEY_POINTS, 0)));
    }

    double[] joints(int i) {
        JointPositions q = model.get("point." + i + ".q", (JointPositions) null);
        if (q == null) return null;
        JointPosition[] all = q.getAllJointPositions();
        double[] out = new double[all.length];
        for (int k = 0; k < all.length; k++) out[k] = all[k].getPosition(Angle.Unit.RAD);
        return out;
    }

    /** The installation's pick area picture point {@code i} looks at: its index, or -1 (the table found live). */
    int areaOf(int i) {
        return model.get("point." + i + ".area", -1);
    }

    int selected() {
        return Math.max(0, Math.min(pointCount() - 1, model.get(KEY_SELECTED, 0)));
    }

    /** Everything the script needs, from this node's data model and the installation. */
    PickScript script() {
        PickScript s = new PickScript();
        PilotContribution inst = installation();
        s.host = PickScript.hostOf(cockpit().base);
        s.port = model.get(KEY_PORT, PickScript.DEFAULT_PICK_PORT);
        s.nodeId = model.get(KEY_NODE_ID, "");
        for (PickScript.Num n : PickScript.NUMBERS) s.values.put(n.key, model.get(n.key, n.def));
        s.orderFirst = model.get(KEY_ORDER_FIRST, "LR");
        s.orderRows = model.get(KEY_ORDER_ROWS, "FB");
        s.shape = model.get(KEY_SHAPE, "box");
        s.gripCheck = model.get(KEY_GRIP_CHECK, true);
        s.gripLongSide = model.get(KEY_GRIP_LONG, false);
        s.closeLook = model.get(KEY_CLOSE_LOOK, true);
        s.popupOnFail = model.get(KEY_POPUP, true);
        s.polyscope = polyscopeVersion();
        // the arm by name: the pick server asks its kinematics which parts are in reach
        String arm = robotType();
        s.arm = arm.matches("[A-Za-z0-9]{1,8}") ? arm : "";
        for (int i = 0; i < pointCount(); i++) {
            double[] q = joints(i);
            double[] plane = inst == null ? null : inst.areaPlane(areaOf(i));
            s.points.add(new PickScript.Point(q, plane == null ? null : java.util.Arrays.copyOf(plane, 6),
                    plane == null ? 0 : plane[6] * 1000, plane == null ? 0 : plane[7] * 1000));
        }
        return s;
    }

    private List<PickScreen.PointRow> rows() {
        PilotContribution inst = installation();
        List<PickScreen.PointRow> out = new ArrayList<PickScreen.PointRow>();
        for (int i = 0; i < pointCount(); i++) {
            int a = areaOf(i);
            String name = inst == null ? null : inst.areaName(a);
            boolean taught = inst != null && inst.areaPlane(a) != null;
            out.add(new PickScreen.PointRow(taught ? name : "live table", taught));
        }
        return out;
    }

    // -- program node --------------------------------------------------------------------------

    @Override
    public void openView() {
        open = true;
        ensureTemplate();
        refresh();
        view.screen().setStatus("connecting to " + cockpit().base + "…", Ui.Kind.INFO);
        startPolling();
    }

    @Override
    public void closeView() {
        open = false;
        Thread t = poller;
        if (t != null) t.interrupt();
    }

    @Override
    public String getTitle() {
        PickScript s = script();
        int n = pointCount();
        return "3D Pick (" + (s.round() ? "Ø" + PickScript.num(s.longSide())
                : PickScript.num(s.longSide()) + "×" + PickScript.num(s.shortSide())) + "×"
                + PickScript.num(s.n("partHeightMm")) + " mm, " + n + " picture" + (n == 1 ? "" : "s") + ")";
    }

    @Override
    public boolean isDefined() {
        return problem() == null;
    }

    @Override
    public void generateScript(ScriptWriter writer) {
        PickScript s = script();
        Variable v = model.get(KEY_VARIABLE, (Variable) null);
        if (v != null) s.foundVariable = writer.getResolvedVariableName(v);
        Variable l = model.get(KEY_LOC_VARIABLE, (Variable) null);
        if (l != null) s.locVariable = writer.getResolvedVariableName(l);
        for (String line : s.lines()) writer.appendLine(line);
    }

    /** What still stops the node from running, for the screen; null when ready. */
    String problem() {
        return script().problem();
    }

    // -- first open: identity, variables, 0.4.0's survey position --------------------------------

    private void ensureTemplate() {
        if (!model.get(KEY_NODE_ID, "").isEmpty() && model.get(KEY_TEMPLATED, false)) return;
        change(() -> {
            if (model.get(KEY_NODE_ID, "").isEmpty()) model.set(KEY_NODE_ID, newId());
            JointPositions survey = model.get(KEY_SURVEY_040, (JointPositions) null);
            if (survey != null && pointCount() == 0) {
                model.set("point.0.q", survey);
                model.set(KEY_POINTS, 1);
                model.remove(KEY_SURVEY_040);
            }
            model.set(KEY_TEMPLATED, true);
            variable(KEY_VARIABLE, FOUND_NAME);
            variable(KEY_LOC_VARIABLE, LOC_NAME);
        });
    }

    private void variable(String key, String name) {
        try {
            if (model.get(key, (Variable) null) == null) {
                model.set(key, api.getProgramAPI().getVariableModel().getVariableFactory().createGlobalVariable(name));
            }
        } catch (Exception e) {
            // the script falls back to the plain name
        }
    }

    private static String newId() {
        byte[] b = new byte[3];
        new SecureRandom().nextBytes(b);
        return String.format("%02x%02x%02x", b[0] & 0xff, b[1] & 0xff, b[2] & 0xff);
    }

    private void change(final Runnable r) {
        api.getProgramAPI().getUndoRedoManager().recordChanges(new UndoableChanges() {
            @Override
            public void executeChanges() {
                r.run();
            }
        });
    }

    private void refresh() {
        view.screen().show(script(), rows(), selected());
        sceneAt = 0; // what the picture finds follows at once
    }

    // -- the picture points ---------------------------------------------------------------------

    @Override
    public void addPoint() {
        if (pointCount() >= PickScript.MAX_POINTS) return;
        teachPosition(pointCount(), true);
    }

    @Override
    public void retake(int i) {
        teachPosition(i, false);
    }

    /** PolyScope's own move screen: the operator puts the arm where the camera sees the parts, then OK. */
    private void teachPosition(final int i, final boolean adding) {
        TeachPosition.teacher(robotType()).teach(api.getUserInterfaceAPI().getUserInteraction(), new TeachPosition.Done() {
            @Override
            public void taught(final JointPositions joints, double[] flange) {
                change(() -> {
                    model.set("point." + i + ".q", joints);
                    if (adding) {
                        PilotContribution inst = installation();
                        // a new point looks at the area the last one did, or the first taught one
                        int area = i > 0 ? areaOf(i - 1) : inst == null ? -1 : inst.firstTaughtArea();
                        model.set("point." + i + ".area", area);
                        model.set(KEY_POINTS, i + 1);
                    }
                    model.set(KEY_SELECTED, i);
                });
                refresh();
                view.screen().setStatus("picture " + (i + 1) + " taught — the camera must see the parts from here,"
                        + " at least 0.3 m away", Ui.Kind.OK);
            }
        });
    }

    @Override
    public void goTo(int i) {
        JointPositions q = model.get("point." + i + ".q", (JointPositions) null);
        if (q == null) return;
        api.getUserInterfaceAPI().getUserInteraction().getRobotMovement().requestUserToMoveRobot(q,
                done("picture " + (i + 1)));
        change(() -> model.set(KEY_SELECTED, i));
        refresh();
    }

    @Override
    public void remove(final int i) {
        final int n = pointCount();
        if (i < 0 || i >= n) return;
        change(() -> {
            for (int k = i; k < n - 1; k++) {
                JointPositions q = model.get("point." + (k + 1) + ".q", (JointPositions) null);
                if (q != null) model.set("point." + k + ".q", q);
                model.set("point." + k + ".area", model.get("point." + (k + 1) + ".area", -1));
            }
            model.remove("point." + (n - 1) + ".q");
            model.remove("point." + (n - 1) + ".area");
            model.set(KEY_POINTS, n - 1);
            model.set(KEY_SELECTED, Math.max(0, Math.min(i, n - 2)));
        });
        refresh();
        view.screen().setStatus("picture " + (i + 1) + " removed", Ui.Kind.INFO);
    }

    @Override
    public void select(int i) {
        change(() -> model.set(KEY_SELECTED, i));
        refresh();
    }

    /** Tap the area line of a point: the next taught area, then "live table", round again. */
    @Override
    public void cycleArea(final int i) {
        PilotContribution inst = installation();
        final int areas = inst == null ? 0 : inst.areaCount();
        int now = areaOf(i);
        int next = now;
        for (int step = 0; step <= areas; step++) {
            next = next + 1 >= areas ? -1 : next + 1;
            if (next == -1 || (inst != null && inst.areaPlane(next) != null)) break;
        }
        final int pick = next;
        change(() -> model.set("point." + i + ".area", pick));
        refresh();
        view.screen().setStatus(pick < 0 ? "picture " + (i + 1) + " finds the table live in every picture"
                : "picture " + (i + 1) + " looks at " + inst.areaName(pick) + " — parts outside it are left alone",
                Ui.Kind.OK);
        if (areas == 0) {
            view.screen().setStatus("no pick area is taught yet: Installation → URCaps → Perceptronic → Pick areas",
                    Ui.Kind.INFO);
        }
    }

    // -- the settings -----------------------------------------------------------------------------

    @Override
    public void setOrder(final String first, final String rows) {
        if (!PickScript.isOrder(first, rows)) return;
        change(() -> {
            model.set(KEY_ORDER_FIRST, first);
            model.set(KEY_ORDER_ROWS, rows);
        });
        refresh();
        view.screen().setStatus("pick order: " + PickScript.orderText(first, rows), Ui.Kind.OK);
    }

    @Override
    public void setNumber(final String key, double value) {
        PickScript s = script();
        final double v = s.set(key, value);
        change(() -> model.set(key, v));
        refresh();
    }

    @Override
    public void askNumber(final String key, JLabel anchor) {
        final PickScript.Num n = PickScript.BY_KEY.get(key);
        if (n == null) return;
        KeyboardNumberInput<Double> kb = api.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory()
                .createPositiveDoubleKeypadInput();
        kb.setInitialValue(model.get(key, n.def));
        kb.show(anchor, new KeyboardInputCallback<Double>() {
            @Override
            public void onOk(Double value) {
                if (value != null) setNumber(key, value);
            }
        });
    }

    @Override
    public void setShape(final String shape) {
        if (!PickScript.contains(PickScript.SHAPES, shape)) return;
        change(() -> model.set(KEY_SHAPE, shape));
        refresh();
        view.screen().setStatus("cyl".equals(shape) ? "a cylinder standing on its end: its diameter and its height"
                : "a box: length and width as it lies, and its height", Ui.Kind.OK);
    }

    @Override
    public void setFlag(final String key, final boolean on) {
        if (PickScreen.FLAG_GRIP_CHECK.equals(key)) {
            change(() -> model.set(KEY_GRIP_CHECK, on));
            view.screen().setStatus(on ? "grip check on: a part with less than the finger room on either side is"
                    + " skipped" : "grip check off: a part is picked however close its neighbours are", Ui.Kind.INFO);
        } else if (PickScreen.FLAG_GRIP_LONG.equals(key)) {
            change(() -> model.set(KEY_GRIP_LONG, on));
            view.screen().setStatus(on ? "the fingers close across the part's long side"
                    : "the fingers close across the part's short side", Ui.Kind.INFO);
        } else if (PickScreen.FLAG_CLOSE_LOOK.equals(key)) {
            change(() -> model.set(KEY_CLOSE_LOOK, on));
            view.screen().setStatus(on ? "closer look on: the arm moves in to measure the part again before the"
                    + " approach" : "closer look off: the approach uses the survey's measurement", Ui.Kind.INFO);
        }
        refresh();
    }

    @Override
    public void setDepthView(boolean on) {
        depthView = on;
        seq = 0;
    }

    @Override
    public void resetDefaults() {
        change(() -> {
            for (PickScript.Num n : PickScript.NUMBERS) model.set(n.key, n.def);
            model.set(KEY_SHAPE, "box");
            model.set(KEY_GRIP_CHECK, true);
            model.set(KEY_GRIP_LONG, false);
            model.set(KEY_CLOSE_LOOK, true);
        });
        refresh();
        view.screen().setStatus("every option back at its default (the picture points and the order are kept)",
                Ui.Kind.INFO);
    }

    // -- check the approach with PolyScope's move screen --------------------------------------------

    @Override
    public void checkApproach() {
        final PickScript s = script();
        final Cockpit c = cockpit();
        final int i = selected();
        view.screen().setStatus("asking the camera computer where the first part's approach is…", Ui.Kind.INFO);
        actions.submit(() -> {
            try {
                Map<String, Object> res = c.get("/api/pick/scene?opts=" + URLEncoder.encode(s.tokens(i), "UTF-8")
                        + "&approach_mm=" + PickScript.num(s.n("approachMm")), 15000);
                Object parts = res.get("parts");
                Map<?, ?> first = parts instanceof List && !((List<?>) parts).isEmpty()
                        && ((List<?>) parts).get(0) instanceof Map ? (Map<?, ?>) ((List<?>) parts).get(0) : null;
                if (!Boolean.TRUE.equals(res.get("ok")) || first == null) {
                    Object why = res.get("reason") != null ? res.get("reason") : res.get("error");
                    view.screen().setStatus("no part to check: " + (why == null ? "?" : why), Ui.Kind.WARN);
                    return;
                }
                final double[] p = Cockpit.six(first.get("polyscope_approach_pose"));
                if (p == null) {
                    view.screen().setStatus("the camera computer has no robot pose (is its robot link up?)", Ui.Kind.ERR);
                    return;
                }
                SwingUtilities.invokeLater(() -> {
                    Pose pose = api.getProgramAPI().getValueFactoryProvider().getPoseFactory()
                            .createPose(p[0], p[1], p[2], p[3], p[4], p[5], Length.Unit.M, Angle.Unit.RAD);
                    view.screen().setStatus("PolyScope's move screen: hold Move to go over the first part — fingertips "
                            + PickScript.num(s.n("approachMm")) + " mm over its top", Ui.Kind.OK);
                    api.getUserInterfaceAPI().getUserInteraction().getRobotMovement().requestUserToMoveRobot(pose,
                            done("approach over the first part"));
                });
            } catch (IOException e) {
                view.screen().setStatus(Cockpit.explain(e, c.base), Ui.Kind.ERR);
            } catch (RuntimeException e) {
                view.screen().setStatus("check: " + e.getMessage(), Ui.Kind.ERR);
            }
        });
    }

    private RobotMovementCallback done(final String what) {
        return new RobotMovementCallback() {
            @Override
            public void onComplete(MovementCompleteEvent event) {
                view.screen().setStatus("at the " + what, Ui.Kind.OK);
                sceneAt = 0;
            }

            @Override
            public void onCancel(MovementCancelEvent event) {
                view.screen().setStatus("move to the " + what + " cancelled", Ui.Kind.WARN);
            }

            @Override
            public void onError(MovementErrorEvent event) {
                view.screen().setStatus("move to the " + what + ": " + event.getErrorType(), Ui.Kind.ERR);
            }
        };
    }

    // -- the live picture ---------------------------------------------------------------------------

    private synchronized void startPolling() {
        if (poller != null && poller.isAlive()) return;
        Thread t = new Thread(this::pollLoop, "realsense-pick-feed");
        t.setDaemon(true);
        poller = t;
        t.start();
    }

    private void pollLoop() {
        boolean announced = false;
        String told = "";
        while (open && !Thread.currentThread().isInterrupted()) {
            Cockpit c = cockpit();
            try {
                boolean heat = depthView;
                Cockpit.Frame f = c.framePng(heat, seq, POLL_TIMEOUT_MS);
                if (f.status == 404 && heat) {
                    depthView = false; // a camera computer older than 0.7.0: no heatmap, the picture instead
                    view.screen().setStatus("this camera computer has no depth view yet — update it", Ui.Kind.WARN);
                    continue;
                }
                if (f.status != 200) {
                    announced = false;
                    String why = f.status == 503 ? Cockpit.noPicture(c.base, null)
                            : "The camera computer answers, but not as expected (HTTP " + f.status + ").\n• Update the"
                            + " camera computer's software and restart it";
                    view.screen().setLive(false, why);
                    view.screen().setStatus(why, Ui.Kind.WARN);
                    sleep(1000);
                    continue;
                }
                seq = f.seq;
                view.screen().setFrame(f.image);
                view.screen().setLive(true, null);
                if (System.currentTimeMillis() - sceneAt > SCENE_EVERY_MS) {
                    sceneAt = System.currentTimeMillis();
                    PickScript s = script();
                    Map<String, Object> res = c.get("/api/pick/scene?opts="
                            + URLEncoder.encode(s.tokens(pointCount() == 0 ? -1 : selected()), "UTF-8"), 3000);
                    if (res.get("pick_port") instanceof Number) keepPort(((Number) res.get("pick_port")).intValue());
                    scene = Scene.parse(res);
                    view.screen().setScene(scene);
                    if (!Boolean.TRUE.equals(res.get("ok")) && res.get("error") != null) {
                        view.screen().setStatus("the camera computer: " + res.get("error"), Ui.Kind.WARN);
                        announced = false;
                    } else if (announced && !scene.summary().equals(told)) {
                        told = scene.summary();
                        view.screen().setStatus(told, scene.parts.isEmpty() ? Ui.Kind.WARN : Ui.Kind.OK);
                    }
                }
                if (!announced) {
                    String p = problem();
                    told = scene.summary();
                    view.screen().setStatus(p == null ? told : "to do: " + p, p != null ? Ui.Kind.INFO
                            : scene.parts.isEmpty() ? Ui.Kind.WARN : Ui.Kind.OK);
                    announced = true;
                }
            } catch (IOException e) {
                announced = false;
                String why = Cockpit.explain(e, c.base);
                view.screen().setLive(false, why);
                view.screen().setStatus(why, Ui.Kind.ERR);
                sleep(1500);
            } catch (RuntimeException e) {
                announced = false;
                String why = Cockpit.explain(e, c.base);
                view.screen().setLive(false, why);
                view.screen().setStatus(why, Ui.Kind.ERR);
                sleep(1500);
            }
        }
    }

    /** The pick server's port as the camera computer reports it, kept for the program. */
    private void keepPort(final int port) {
        if (port <= 0 || port == model.get(KEY_PORT, PickScript.DEFAULT_PICK_PORT)) return;
        SwingUtilities.invokeLater(() -> change(() -> model.set(KEY_PORT, port)));
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
