package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.contribution.program.CreationContext;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.program.nodes.ProgramNodeFactory;
import com.ur.urcap.api.domain.program.nodes.builtin.CommentNode;
import com.ur.urcap.api.domain.program.nodes.contributable.URCapProgramNode;
import com.ur.urcap.api.domain.program.structure.TreeNode;
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
 * One Perceptronic Pick node in a program (0.5.0). Teach time — the node's screen open — the
 * live picture with the parts numbered in pick order ({@code GET /api/pick/scene}, asked with
 * exactly the options the program will send), the picture points, the pick order, and the
 * Options view. Run time: {@link PickScript}. Everything is in the node's data model;
 * the camera computer's address, the pick areas and the reach come from the Installation
 * node, one per robot.
 */
public class PickContribution implements ProgramNodeContribution, PickScreen.Actions {
    static final String KEY_NODE_ID = "nodeId";
    static final String KEY_TEMPLATED = "templated";
    static final String KEY_POINTS = "points";
    static final String KEY_SELECTED = "selectedPoint";
    static final String KEY_ORDER_FIRST = "orderFirst";
    static final String KEY_ORDER_ROWS = "orderRows";
    static final String KEY_GRIPPER = "gripper";
    static final String KEY_POPUP = "popupOnFail";
    static final String KEY_PER_POINT = "perPointRoutine";
    static final String KEY_PORT = "pickPort";
    static final String KEY_VARIABLE = "foundVariable";
    static final String KEY_LOC_VARIABLE = "locVariable";
    static final String KEY_SURVEY_040 = "surveyJoints"; // 0.4.0's single survey position
    static final String FOUND_NAME = "rs_pick_found";
    static final String LOC_NAME = "rs_pick_loc";
    static final String AFTER_PICK = "After the pick: insert what happens to the part (place it, …)";
    static final String GRIP_HERE = "Close the gripper here: insert your gripper's Close node";
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
        s.gripper = model.get(KEY_GRIPPER, "robotiq");
        s.popupOnFail = model.get(KEY_POPUP, true);
        s.polyscope = polyscopeVersion();
        if (inst != null) {
            double[] reach = inst.reachLimits();
            if (reach != null) {
                s.reachMinM = reach[0];
                s.reachMaxM = reach[1];
            }
        }
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

    private TreeNode tree() {
        return api.getProgramAPI().getProgramModel().getRootTreeNode(this);
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
        return "Perceptronic Pick (" + PickScript.num(Math.max(s.n("partLengthMm"), s.n("partWidthMm"))) + "×"
                + PickScript.num(Math.min(s.n("partLengthMm"), s.n("partWidthMm"))) + "×"
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
        for (String line : s.beforeChildren()) writer.appendLine(line);
        if (s.ownGripperNodes()) writer.writeChildren();
        for (String line : s.afterChildren()) writer.appendLine(line);
        if (!s.ownGripperNodes()) {
            writer.appendLine(s.afterPickOpen());
            writer.writeChildren();
            writer.appendLine("end");
        }
    }

    /** What still stops the node from running, for the screen; null when ready. */
    String problem() {
        PickScript s = script();
        String p = s.problem();
        if (p != null) return p;
        if (s.ownGripperNodes() && !hasNonComment()) {
            return "insert your gripper's Close node inside this node (it runs at the grip)";
        }
        return null;
    }

    private boolean hasNonComment() {
        try {
            for (TreeNode child : tree().getChildren()) {
                if (!(child.getProgramNode() instanceof CommentNode)) return true;
            }
        } catch (RuntimeException e) {
            return false;
        }
        return false;
    }

    // -- first open: identity, variables, the child template, 0.4.0's settings ------------------

    private void ensureTemplate() {
        if (!model.get(KEY_NODE_ID, "").isEmpty() && model.get(KEY_TEMPLATED, false)) return;
        change(() -> {
            if (model.get(KEY_NODE_ID, "").isEmpty()) model.set(KEY_NODE_ID, newId());
            boolean upgraded = model.get(KEY_TEMPLATED, false); // a 0.4.0 node opened in 0.5.0
            if (upgraded && !model.isSet(KEY_GRIPPER)) model.set(KEY_GRIPPER, "children"); // its children grip
            JointPositions survey = model.get(KEY_SURVEY_040, (JointPositions) null);
            if (survey != null && pointCount() == 0) {
                model.set("point.0.q", survey);
                model.set(KEY_POINTS, 1);
                model.remove(KEY_SURVEY_040);
            }
            model.set(KEY_TEMPLATED, true);
            try {
                TreeNode root = tree();
                if (root.getChildren().isEmpty()) {
                    CommentNode c = factory().createCommentNode();
                    c.setComment("children".equals(model.get(KEY_GRIPPER, "robotiq")) ? GRIP_HERE : AFTER_PICK);
                    root.addChild(c);
                }
            } catch (Exception e) {
                // an empty node is still usable: the screen says what to insert
            }
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

    private ProgramNodeFactory factory() {
        return api.getProgramAPI().getProgramModel().getProgramNodeFactory();
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
        view.screen().show(script(), rows(), selected(), model.get(KEY_PER_POINT, false));
        sceneAt = 0; // the picture's numbers follow at once
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
                if (adding && model.get(KEY_PER_POINT, false)) ensurePointRoutines();
                refresh();
                view.screen().setStatus("picture " + (i + 1) + " taught — the camera must see the parts from here,"
                        + " at least 0.25 m away", Ui.Kind.OK);
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
        view.screen().setStatus("pick order: " + PickScript.orderText(first, rows) + " — see the numbers",
                Ui.Kind.OK);
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
    public void setGripper(final String mode) {
        if (!PickScript.contains(PickScript.GRIPPERS, mode)) return;
        change(() -> model.set(KEY_GRIPPER, mode));
        refresh();
        view.screen().setStatus("children".equals(mode)
                ? "your gripper nodes go inside this node and run at the grip; what happens next follows the node"
                : "the node drives the " + PickScreen.gripperWords(mode) + "; its children are the routine after the"
                + " pick", Ui.Kind.OK);
    }

    @Override
    public void setFlag(final String key, final boolean on) {
        if (PickScreen.FLAG_POPUP.equals(key)) {
            change(() -> model.set(KEY_POPUP, on));
        } else if (PickScreen.FLAG_PER_POINT.equals(key)) {
            change(() -> model.set(KEY_PER_POINT, on));
            if (on) ensurePointRoutines();
            view.screen().setStatus(on ? "each picture point has its own routine inside this node (\"After picture N\")"
                    : "the per-picture routines stay in the program until you delete them", Ui.Kind.INFO);
        }
        refresh();
    }

    @Override
    public void resetDefaults() {
        change(() -> {
            for (PickScript.Num n : PickScript.NUMBERS) model.set(n.key, n.def);
            model.set(KEY_POPUP, true);
        });
        refresh();
        view.screen().setStatus("every option back at its default (the picture points and the order are kept)",
                Ui.Kind.INFO);
    }

    /** One "After picture N" child per picture point that doesn't have one yet. */
    private void ensurePointRoutines() {
        change(() -> {
            try {
                TreeNode root = tree();
                boolean[] have = new boolean[PickScript.MAX_POINTS + 1];
                for (TreeNode c : root.getChildren()) {
                    if (c.getProgramNode() instanceof URCapProgramNode) {
                        URCapProgramNode u = (URCapProgramNode) c.getProgramNode();
                        if (u.canGetAs(PickRoutineContribution.class)) {
                            int k = u.getAs(PickRoutineContribution.class).point();
                            if (k >= 1 && k <= PickScript.MAX_POINTS) have[k] = true;
                        }
                    }
                }
                Variable loc = model.get(KEY_LOC_VARIABLE, (Variable) null);
                for (int k = 1; k <= pointCount(); k++) {
                    if (have[k]) continue;
                    URCapProgramNode node = factory().createURCapProgramNode(PickRoutineService.class);
                    node.getAs(PickRoutineContribution.class).init(k, loc);
                    root.addChild(node);
                }
            } catch (Exception e) {
                view.screen().setStatus("could not add the per-picture routines: " + e.getMessage(), Ui.Kind.ERR);
            }
        });
    }

    // -- check the approach with PolyScope's move screen --------------------------------------------

    @Override
    public void checkApproach() {
        final PickScript s = script();
        final Cockpit c = cockpit();
        final int i = selected();
        view.screen().setStatus("asking the camera computer where part #1's approach is…", Ui.Kind.INFO);
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
                    view.screen().setStatus("PolyScope's move screen: hold Move to go over part #1 — fingertips "
                            + PickScript.num(s.n("approachMm")) + " mm over its top", Ui.Kind.OK);
                    api.getUserInterfaceAPI().getUserInteraction().getRobotMovement().requestUserToMoveRobot(pose,
                            done("approach over part #1"));
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
        while (open && !Thread.currentThread().isInterrupted()) {
            Cockpit c = cockpit();
            try {
                Cockpit.Frame f = c.colorPng(seq, POLL_TIMEOUT_MS);
                if (f.status != 200) {
                    announced = false;
                    view.screen().setLive(false, null);
                    view.screen().setStatus(f.status == 503 ? "the camera computer is up but has no picture yet"
                            + " (camera opening?)" : "the camera computer at " + c.base + " answered HTTP " + f.status,
                            Ui.Kind.WARN);
                    sleep(1000);
                    continue;
                }
                seq = f.seq;
                view.screen().setFrame(f.image);
                view.screen().setLive(true, f.fps);
                if (System.currentTimeMillis() - sceneAt > SCENE_EVERY_MS) {
                    sceneAt = System.currentTimeMillis();
                    PickScript s = script();
                    Map<String, Object> res = c.get("/api/pick/scene?opts="
                            + URLEncoder.encode(s.tokens(pointCount() == 0 ? -1 : selected()), "UTF-8"), 3000);
                    if (res.get("pick_port") instanceof Number) keepPort(((Number) res.get("pick_port")).intValue());
                    scene = Scene.parse(res);
                    view.screen().setScene(scene);
                    if (!Boolean.TRUE.equals(res.get("ok")) && res.get("error") != null) {
                        view.screen().setStatus("the camera computer: " + res.get("error") + " (update it to 0.5.0?)",
                                Ui.Kind.WARN);
                        announced = false;
                    }
                }
                if (!announced) {
                    String p = problem();
                    view.screen().setStatus(p == null ? "ready — live from " + c.base : "to do: " + p,
                            p == null ? Ui.Kind.OK : Ui.Kind.INFO);
                    announced = true;
                }
            } catch (IOException e) {
                announced = false;
                view.screen().setLive(false, null);
                view.screen().setStatus(Cockpit.explain(e, c.base), Ui.Kind.ERR);
                sleep(1500);
            } catch (RuntimeException e) {
                announced = false;
                view.screen().setLive(false, null);
                view.screen().setStatus(Cockpit.explain(e, c.base), Ui.Kind.ERR);
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
