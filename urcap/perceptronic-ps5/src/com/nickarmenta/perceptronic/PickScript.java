package com.nickarmenta.perceptronic;

import java.net.URI;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * The URScript a Perceptronic Pick node contributes — generated here with no UR API, so it is
 * tested on any JDK — and the node's settings (one table, {@link #NUMBERS}: key, default,
 * limits, label; the Options screen and the data model both read it).
 *
 * <p>One run of the node picks <b>one part</b>. It runs inside the operator's program, so it
 * works in Local mode and needs no Primary client:
 *
 * <ol>
 *   <li>force the TCP to the flange (the pick server answers in flange poses) and open the
 *       gripper fully — the fingers open while the arm travels;</li>
 *   <li>ask the pick server {@code NEXT}: a part it already saw at a picture point (the arm
 *       then goes straight to the close look over it — no trip back for a picture) or the
 *       picture point to look from;</li>
 *   <li>at a picture point: {@code movej} there, settle, {@code FIND} — the server finds the
 *       parts by their volume (the part's size above the taught plane), numbers them in the
 *       chosen order and answers the first; an empty picture point sends the arm on to the
 *       next one;</li>
 *   <li>the close look ({@code LOOK}: the camera halfway to the part, aimed at it) and
 *       {@code REFINE} — straight down, then leaned 12° and 24° while the controller's own IK
 *       can't solve approach + grip + lift;</li>
 *   <li>the approach — fingertips {@code approachMm} over the part's top, fingers fully open —
 *       down to the grip, close, lift. A close on nothing opens, lifts and tries the next
 *       part;</li>
 *   <li>the result variable is True only after a held lift, the location variable says which
 *       picture point it came from, and the node's children — the routine after the pick —
 *       run with the operator's TCP back. Anything that stops short says why in a popup.</li>
 * </ol>
 *
 * With the gripper set to "my nodes" the children instead run at the grip (0.4.0's contract)
 * and the routine after the pick is simply the program's next nodes.
 */
final class PickScript {
    static final String VERSION = "0.6.0";
    static final int DEFAULT_PICK_PORT = 7622;
    static final String SOCKET = "rs_pick";
    static final String RQ_SOCKET = "rs_rq";
    static final int MAX_POINTS = 12;
    static final String[] ORDERS = {"LR", "RL", "FB", "BF"};
    static final String[] GRIPPERS = {"robotiq", "digital", "children"};

    /** The pick server's status codes, as the operator should read them. */
    static final String[][] REASONS = {
        {"0", "no part in view at any picture point"},
        {"-1", "the part is wider than the open gripper"},
        {"-2", "the part is too close to the robot base"},
        {"-3", "the camera computer has no hand-eye calibration"},
        {"-4", "no fresh camera frame - is the camera computer's camera streaming?"},
        {"-5", "the closer look did not find the part again"},
        {"-7", "something is in view, but nothing the size of the part"},
        {"-8", "no answer from the camera computer within 10 s"},
        {"-9", "the camera computer did not understand the request (update it?)"},
        {"-10", "the only parts in view are out of reach"},
        {"-11", "no room for the open fingers beside any part"},
        {"-12", "the parts in view are outside the pick area"},
        {"-13", "the only part in view is cut off by the edge of the picture"},
    };

    /** One number the operator can set: where it lives, what it is, what it may be. */
    static final class Num {
        final String key;
        final String section;
        final String label;
        final String unit;
        final double def;
        final double min;
        final double max;
        final double step;
        final String help;

        Num(String key, String section, String label, String unit, double def, double min, double max,
                double step, String help) {
            this.key = key;
            this.section = section;
            this.label = label;
            this.unit = unit;
            this.def = def;
            this.min = min;
            this.max = max;
            this.step = step;
            this.help = help;
        }
    }

    static final Num[] NUMBERS = {
        new Num("partLengthMm", "part", "Length", "mm", 50, 5, 500, 1, "long side, as it lies"),
        new Num("partWidthMm", "part", "Width", "mm", 30, 5, 500, 1, "the fingers close across it"),
        new Num("partHeightMm", "part", "Height", "mm", 30, 5, 500, 1, "top above the table"),
        new Num("partTolPct", "part", "Tolerance", "%", 25, 5, 100, 5, "how far off still counts"),
        new Num("approachMm", "approach", "Approach", "mm", 25, 5, 200, 5,
                "fingertips over the top, fully open"),
        new Num("gripBelowTopMm", "approach", "Grip depth", "mm", 15, 0, 60, 1,
                "fingertips below the top to grip"),
        new Num("liftMm", "approach", "Lift", "mm", 60, 5, 300, 5, "straight up after the grip"),
        new Num("lookMm", "approach", "Close look", "mm", 90, 40, 400, 10,
                "camera over the top, if no better pose"),
        new Num("strokeMm", "gripper", "Open width", "mm", 50, 10, 300, 1, "the fingers' full opening (Hand-E: 50)"),
        new Num("gripperForcePct", "gripper", "Grip force", "%", 40, 0, 100, 5, "Robotiq force"),
        new Num("gripperSpeedPct", "gripper", "Finger speed", "%", 100, 1, 100, 5, "Robotiq speed"),
        new Num("gripperDo", "gripper", "Digital output", "", 0, 0, 7, 1, "the output that closes the gripper"),
        new Num("gripperWaitS", "gripper", "Close time", "s", 0.5, 0, 5, 0.1, "how long a digital-output gripper takes"),
        new Num("speedPct", "motion", "Speed", "%", 60, 10, 100, 10, "of the node's travel speed"),
        new Num("settleS", "motion", "Settle", "s", 0.2, 0, 2, 0.05,
                "before each picture: sharp vs fast"),
        new Num("maxAttempts", "motion", "Grasps per run", "", 3, 1, 10, 1, "tries before the node gives up"),
    };

    static final Map<String, Num> BY_KEY = new LinkedHashMap<String, Num>();

    static {
        for (Num n : NUMBERS) BY_KEY.put(n.key, n);
    }

    /** One picture point: joints to look from, and the taught pick area it looks at (or none). */
    static final class Point {
        final double[] joints;
        final double[] plane; // [x, y, z, rx, ry, rz] base, or null: the table is fitted live
        final double areaXmm;
        final double areaYmm;

        Point(double[] joints, double[] plane, double areaXmm, double areaYmm) {
            this.joints = joints;
            this.plane = plane;
            this.areaXmm = areaXmm;
            this.areaYmm = areaYmm;
        }
    }

    String host = "";
    int port = DEFAULT_PICK_PORT;
    String nodeId = "";
    final List<Point> points = new ArrayList<Point>();
    final Map<String, Double> values = new LinkedHashMap<String, Double>();
    String orderFirst = "LR";
    String orderRows = "FB";
    String gripper = "robotiq";
    double reachMinM = 0; // 0 and 0: no reach limit sent (the Installation screen sets it)
    double reachMaxM = 0;
    boolean popupOnFail = true;
    /** The PolyScope the script runs on ({major, minor, bugfix}), or null: the newest. */
    int[] polyscope = null;
    String foundVariable = "rs_pick_found";
    String locVariable = "rs_pick_loc";

    PickScript() {
        for (Num n : NUMBERS) values.put(n.key, n.def);
    }

    double n(String key) {
        Double v = values.get(key);
        return v == null ? BY_KEY.get(key).def : v;
    }

    /** Store {@code v} for {@code key}, clamped to its limits (whole steps round); returns what was stored. */
    double set(String key, double v) {
        Num n = BY_KEY.get(key);
        if (n == null) throw new IllegalArgumentException("no setting " + key);
        if (Double.isNaN(v) || Double.isInfinite(v)) v = n.def;
        double c = Math.max(n.min, Math.min(n.max, v));
        if (n.step >= 1) c = Math.round(c / n.step) * n.step;
        values.put(key, c);
        return c;
    }

    /**
     * {base outer radius, rated reach} (m) by PolyScope's robot type name — the same table as
     * {@code perceptronics.volume.BASE_RADIUS_M} / {@code urctl.safety.MODEL_REACH_M}; null when unknown.
     */
    static double[] modelReach(String type) {
        String t = type == null ? "" : type.toUpperCase(Locale.ROOT).replaceAll("[^A-Z0-9]", "");
        if (t.endsWith("E")) t = t.substring(0, t.length() - 1);
        if ("UR3".equals(t)) return new double[] {0.064, 0.5};
        if ("UR5".equals(t) || "UR7".equals(t)) return new double[] {0.0745, 0.85};
        if ("UR10".equals(t) || "UR12".equals(t)) return new double[] {0.095, 1.3};
        if ("UR16".equals(t)) return new double[] {0.095, 0.9};
        return null;
    }

    /** The cockpit host from its saved URL (the Installation node's field); "" if none. */
    static String hostOf(String cockpitBase) {
        try {
            String h = new URI(cockpitBase).getHost();
            return h == null ? "" : h;
        } catch (Exception e) {
            return "";
        }
    }

    // -- what the node can't do yet --------------------------------------------------------

    /** Why the node cannot generate a program yet, or null when it can. */
    String problem() {
        if (host.isEmpty()) return "set the camera computer's address in Installation → URCaps → Perceptronic";
        if (!host.matches("[A-Za-z0-9.:\\-]+")) return "the camera computer's host \"" + host + "\" is not an address";
        if (port < 1 || port > 65535) return "the pick port must be 1..65535";
        if (!nodeId.matches("[0-9a-f]{1,12}")) return "the node has no identity yet - open it once";
        if (points.isEmpty()) return "add a picture point: move the arm where the camera sees the parts and tap Add";
        if (points.size() > MAX_POINTS) return "at most " + MAX_POINTS + " picture points";
        for (int i = 0; i < points.size(); i++) {
            Point p = points.get(i);
            if (!finite(p.joints, 6)) return "picture point " + (i + 1) + " is not a joint position";
            if (p.plane != null && (!finite(p.plane, 6) || !(Math.abs(p.areaXmm) >= 5 && Math.abs(p.areaYmm) >= 5))) {
                return "picture point " + (i + 1) + "'s pick area is broken - re-teach it in the Installation";
            }
        }
        for (Num k : NUMBERS) {
            double v = n(k.key);
            if (!(v >= k.min && v <= k.max)) {
                return k.label.toLowerCase(Locale.ROOT) + " must be " + num(k.min) + ".." + num(k.max) + " " + k.unit;
            }
        }
        if (n("gripBelowTopMm") > n("partHeightMm") - 2) {
            return "the grip (" + num(n("gripBelowTopMm")) + " mm below the top) would put the fingertips on the table:"
                    + " the part is " + num(n("partHeightMm")) + " mm tall";
        }
        double shortSide = Math.min(n("partLengthMm"), n("partWidthMm"));
        if (shortSide > n("strokeMm") - 6) {
            return "the part's short side (" + num(shortSide) + " mm) needs fingers that open wider than "
                    + num(n("strokeMm")) + " mm";
        }
        if (!isOrder(orderFirst, orderRows)) return "the pick order must be one horizontal and one vertical direction";
        if (!contains(GRIPPERS, gripper)) return "unknown gripper \"" + gripper + "\"";
        if (!(reachMinM >= 0 && reachMinM < 3 && reachMaxM >= 0 && reachMaxM <= 3
                && (reachMaxM == 0 || reachMaxM > reachMinM))) {
            return "the reach limits are not valid (Installation → Perceptronic → Reach)";
        }
        if (!foundVariable.matches("[A-Za-z_][A-Za-z0-9_]*") || !locVariable.matches("[A-Za-z_][A-Za-z0-9_]*")) {
            return "the result variable names are not valid";
        }
        return null;
    }

    static boolean isOrder(String first, String rows) {
        if (!contains(ORDERS, first) || !contains(ORDERS, rows)) return false;
        return horizontal(first) != horizontal(rows);
    }

    static boolean horizontal(String o) {
        return "LR".equals(o) || "RL".equals(o);
    }

    // -- the request options -----------------------------------------------------------------

    /**
     * {@code " part=50x30x30 tol=25 order=LR,FB …"} for picture point {@code i} (0-based; -1:
     * none): exactly what the pick server's {@code parse_options} reads, and what the teach
     * screen asks with — so the pendant's numbers are the program's order.
     */
    String tokens(int i) {
        StringBuilder b = new StringBuilder();
        double l = Math.max(n("partLengthMm"), n("partWidthMm"));
        double w = Math.min(n("partLengthMm"), n("partWidthMm"));
        b.append(" part=").append(num(l)).append('x').append(num(w)).append('x').append(num(n("partHeightMm")));
        b.append(" tol=").append(num(n("partTolPct")));
        b.append(" order=").append(orderFirst).append(',').append(orderRows);
        b.append(" grip=").append(num(n("gripBelowTopMm"))).append(" stroke=").append(num(n("strokeMm")));
        if (reachMinM > 0 || reachMaxM > 0) b.append(String.format(Locale.ROOT, " reach=%.3f,%.3f", reachMinM, reachMaxM));
        Point p = i >= 0 && i < points.size() ? points.get(i) : null;
        if (p != null && p.plane != null) {
            b.append(" plane=").append(pose(p.plane));
            b.append(" area=").append(num(p.areaXmm)).append('x').append(num(p.areaYmm));
        }
        b.append(" node=").append(nodeId);
        if (i >= 0) b.append(" loc=").append(i + 1);
        b.append(" locs=").append(Math.max(1, points.size())).append(" proto=2");
        return b.toString();
    }

    String partText() {
        return num(Math.max(n("partLengthMm"), n("partWidthMm"))) + " x " + num(Math.min(n("partLengthMm"),
                n("partWidthMm"))) + " x " + num(n("partHeightMm")) + " mm +-" + num(n("partTolPct")) + " %";
    }

    static String orderText(String first, String rows) {
        return words(first) + ", rows " + words(rows);
    }

    static String words(String o) {
        return "LR".equals(o) ? "left to right" : "RL".equals(o) ? "right to left"
                : "FB".equals(o) ? "front to back" : "back to front";
    }

    boolean ownGripperNodes() {
        return "children".equals(gripper);
    }

    /**
     * The oldest PolyScope known to have {@code get_inverse_kin_has_solution}: 5.8.2 does not
     * compile it ({@code compile_error_name_not_found}), 5.9.4 does — the matrix's images,
     * 2026-09-29; 5.9.0-5.9.3 have no image to try, so they count as without it.
     */
    static final int[] IK_CHECK_SINCE = {5, 9, 4};

    /**
     * Does the controller's own IK get asked before the close look and each lean of the
     * approach? Before {@link #IK_CHECK_SINCE} it can't be: the arm takes the close look and
     * the straight-down approach unchecked, and an unreachable one stops the program with
     * PolyScope's own IK error instead of leaning.
     */
    boolean ikCheck() {
        return polyscope == null || atLeast(polyscope, IK_CHECK_SINCE);
    }

    static boolean atLeast(int[] version, int[] since) {
        for (int i = 0; i < since.length; i++) {
            int v = i < version.length ? version[i] : 0;
            if (v != since[i]) return v > since[i];
        }
        return true;
    }

    // -- the script ----------------------------------------------------------------------

    /** The lines before the child nodes. */
    List<String> beforeChildren() {
        String why = problem();
        if (why != null) throw new IllegalStateException(why);
        List<String> s = new ArrayList<String>();
        int np = points.size();
        int budget = np + (int) n("maxAttempts") + 1;
        double k = n("speedPct") / 100.0;
        String jv = f2(1.05 * k);
        String ja = f2(1.4 * k);
        String lv = f2(0.25 * k);
        String la = f2(0.6 * k);
        s.add("# Perceptronic Pick " + VERSION + " - camera computer " + host + ":" + port + " - part " + partText()
                + " - " + orderText(orderFirst, orderRows) + " - " + np + " picture point" + (np == 1 ? "" : "s"));
        if (!ikCheck()) {
            s.add("# PolyScope " + polyscope[0] + "." + polyscope[1] + "." + polyscope[2]
                    + ": no get_inverse_kin_has_solution - the close look and the approach go unchecked");
        }
        s.add(foundVariable + " = False");
        s.add(locVariable + " = 0");
        s.add("rs_tcp0 = get_tcp_offset()");
        s.add("set_tcp(p[0, 0, 0, 0, 0, 0])");
        s.add("rs_why = \"no answer from the camera computer within 10 s\"");
        s.add("rs_held = False");
        s.add("rs_try = 0");
        s.add("rs_ok = True");
        s.add("rs_loc = 1");
        s.add("rs_tok = \"\"");
        s.add("if socket_open(\"" + host + "\", " + port + ", \"" + SOCKET + "\"):");
        say(s, "  ", "start, flange ", "get_actual_tcp_pose()", true);
        say(s, "  ", "looking for " + partText(), null, true);
        gripperStart(s, "  ");
        s.add("  while (rs_ok) and (rs_try < " + budget + ") and (" + foundVariable + " == False):");
        s.add("    rs_try = rs_try + 1");
        s.add("    socket_send_line(str_cat(\"NEXT \", str_cat(to_str(get_actual_tcp_pose()), \" node=" + nodeId
                + " locs=" + np + " proto=2\")), \"" + SOCKET + "\")");
        read16(s, "    ");
        s.add("    if rs_r[0] == 16:");
        s.add("      rs_loc = floor(rs_r[11] + 0.5)");
        s.add("    end");
        s.add("    if (rs_loc < 1) or (rs_loc > " + np + "):");
        s.add("      rs_loc = 1");
        s.add("    end");
        for (int i = 0; i < np; i++) {
            s.add("    " + (i == 0 ? "if" : "elif") + " rs_loc == " + (i + 1) + ":");
            s.add("      rs_tok = \"" + tokens(i) + "\"");
        }
        s.add("    end");
        s.add("    if rs_st == 1:");
        say(s, "      ", "next part already seen, #", "rs_r[12]", true);
        s.add("    else:");
        for (int i = 0; i < np; i++) {
            s.add("      " + (i == 0 ? "if" : "elif") + " rs_loc == " + (i + 1) + ":");
            s.add("        movej(" + joints(points.get(i).joints) + ", a=" + ja + ", v=" + jv + ")");
        }
        s.add("      end");
        s.add("      sleep(" + f2(n("settleS")) + ")");
        say(s, "      ", "picture at point ", "rs_loc", true);
        s.add("      socket_send_line(str_cat(\"FIND \", str_cat(to_str(get_actual_tcp_pose()), rs_tok)), \""
                + SOCKET + "\")");
        read16(s, "      ");
        say(s, "      ", "FIND status ", "rs_st", true);
        s.add("      if rs_st != 1:");
        reasons(s, "        ");
        s.add("      end");
        s.add("    end");
        s.add("    if rs_st == 1:");
        s.add("      rs_c = p[rs_r[2], rs_r[3], rs_r[4], 0, 0, 0]");
        s.add("      rs_top = p[rs_r[5], rs_r[6], rs_r[7], rs_r[8], rs_r[9], rs_r[10]]");
        s.add("      rs_look = pose_trans(rs_top, p[0, 0, " + m(-n("lookMm")) + ", 0, 0, 0])");
        s.add("      socket_send_line(str_cat(\"LOOK \", str_cat(to_str(get_actual_tcp_pose()), str_cat(\" \","
                + " to_str(rs_c)))), \"" + SOCKET + "\")");
        // its own list: up to PolyScope 5.14 a list keeps its first size ("Resizing of 'List' is
        // not supported" when rs_r took 10 numbers after 16 — URControl.log of 5.9.4 .. 5.14.6 in
        // the matrix, 2026-09-29; 5.15.2 on resize it)
        s.add("      rs_lk = socket_read_ascii_float(10, \"" + SOCKET + "\", 10)");
        s.add("      if rs_lk[0] == 10:");
        s.add("        if rs_lk[1] == 1:");
        s.add("          rs_look = p[rs_lk[5], rs_lk[6], rs_lk[7], rs_lk[8], rs_lk[9], rs_lk[10]]");
        s.add("        end");
        s.add("      end");
        if (ikCheck()) {
            s.add("      if get_inverse_kin_has_solution(rs_look, get_actual_joint_positions()):");
            s.add("        movej(get_inverse_kin(rs_look, get_actual_joint_positions()), a=" + ja + ", v=" + jv + ")");
            s.add("        sleep(" + f2(n("settleS")) + ")");
            say(s, "        ", "close look ", "rs_look", true);
            s.add("      else:");
            say(s, "        ", "the close look is out of reach - looking again from here", null, true);
            s.add("      end");
        } else {
            s.add("      movej(get_inverse_kin(rs_look, get_actual_joint_positions()), a=" + ja + ", v=" + jv + ")");
            s.add("      sleep(" + f2(n("settleS")) + ")");
            say(s, "      ", "close look ", "rs_look", true);
        }
        s.add("      rs_leans = [0, 12, 24]");
        s.add("      rs_lean = 0");
        s.add("      rs_go = False");
        s.add("      rs_rs = -8");
        s.add("      while (rs_lean < 3) and (rs_go == False):");
        s.add("        socket_send_line(str_cat(\"REFINE \", str_cat(to_str(get_actual_tcp_pose()), str_cat(\" \","
                + " str_cat(to_str(rs_c), str_cat(rs_tok, str_cat(\" lean=\", to_str(rs_leans[rs_lean]))))))), \""
                + SOCKET + "\")");
        s.add("        rs_r = socket_read_ascii_float(16, \"" + SOCKET + "\", 10)");
        s.add("        rs_rs = -8");
        s.add("        if rs_r[0] == 16:");
        s.add("          rs_rs = rs_r[1]");
        s.add("        end");
        say(s, "        ", "REFINE status ", "rs_rs", true);
        s.add("        if rs_rs == 1:");
        s.add("          rs_c = p[rs_r[2], rs_r[3], rs_r[4], 0, 0, 0]");
        s.add("          rs_top = p[rs_r[5], rs_r[6], rs_r[7], rs_r[8], rs_r[9], rs_r[10]]");
        s.add("          rs_hover = pose_trans(rs_top, p[0, 0, " + m(-n("approachMm")) + ", 0, 0, 0])");
        s.add("          rs_grip = pose_trans(rs_top, p[0, 0, " + m(n("gripBelowTopMm")) + ", 0, 0, 0])");
        s.add("          rs_lift = pose_trans(rs_top, p[0, 0, " + m(-n("liftMm")) + ", 0, 0, 0])");
        if (ikCheck()) {
            s.add("          rs_q = get_actual_joint_positions()");
            s.add("          if get_inverse_kin_has_solution(rs_hover, rs_q) and get_inverse_kin_has_solution(rs_grip,"
                    + " rs_q) and get_inverse_kin_has_solution(rs_lift, rs_q):");
            s.add("            rs_go = True");
            s.add("          else:");
            say(s, "            ", "no IK for approach + grip + lift at lean ", "rs_leans[rs_lean]", true);
            s.add("          end");
        } else {
            s.add("          rs_go = True");
        }
        s.add("          rs_lean = rs_lean + 1");
        s.add("        else:");
        s.add("          rs_lean = 3");
        s.add("        end");
        s.add("      end");
        s.add("      if rs_go:");
        say(s, "        ", "approach ", "rs_hover", true);
        s.add("        movel(rs_hover, a=" + la + ", v=" + lv + ")");
        gripperOpenWait(s, "        ");
        say(s, "        ", "down to the grip ", "rs_grip", true);
        s.add("        movel(rs_grip, a=0.3, v=0.05)");
        s.add("        rs_held = False");
        if (ownGripperNodes()) {
            s.add("        set_tcp(rs_tcp0)");
            say(s, "        ", "your gripper nodes", null, true);
        } else {
            gripperClose(s, "        ");
        }
        return s;
    }

    /** The lines after the child nodes (with a gripper the node drives: before them — see {@link #render}). */
    List<String> afterChildren() {
        List<String> s = new ArrayList<String>();
        boolean own = ownGripperNodes();
        double k = n("speedPct") / 100.0;
        if (own) {
            s.add("        set_tcp(p[0, 0, 0, 0, 0, 0])");
            s.add("        rs_held = True");
        }
        say(s, "        ", "lift", null, true);
        s.add("        movel(rs_lift, a=0.5, v=" + f2(Math.max(0.05, 0.15 * k)) + ")");
        s.add("        if rs_held:");
        s.add("          " + foundVariable + " = True");
        s.add("          " + locVariable + " = rs_loc");
        say(s, "          ", "picked at point ", "rs_loc", true);
        s.add("        else:");
        s.add("          rs_why = \"the gripper closed on nothing\"");
        say(s, "          ", "closed on nothing - trying the next part", null, true);
        if (!own) gripperOpen(s, "          ");
        s.add("        end");
        s.add("      elif rs_rs != 1:");
        s.add("        rs_st = rs_rs");
        reasons(s, "        ");
        s.add("      else:");
        s.add("        rs_why = \"no approach the arm can reach (straight down or leaned to 24 deg)\"");
        s.add("      end");
        s.add("    end");
        s.add("  end");
        s.add("  if " + foundVariable + " == False:");
        s.add("    textmsg(\"Perceptronic Pick: no pick - \", rs_why)");
        s.add("    socket_send_line(str_cat(\"LOG no pick - \", rs_why), \"" + SOCKET + "\")");
        s.add("  end");
        s.add("  socket_close(\"" + SOCKET + "\")");
        s.add("else:");
        s.add("  rs_why = \"no camera computer at " + host + ":" + port
                + " - is it on, and is the address in Installation > Perceptronic right?\"");
        s.add("  textmsg(\"Perceptronic Pick: \", rs_why)");
        s.add("end");
        if ("robotiq".equals(gripper)) s.add("socket_close(\"" + RQ_SOCKET + "\")");
        s.add("set_tcp(rs_tcp0)");
        if (popupOnFail) {
            s.add("if " + foundVariable + " == False:");
            s.add("  popup(str_cat(\"Perceptronic Pick: no pick - \", rs_why), \"Perceptronic Pick\", False, True,"
                    + " blocking=True)");
            s.add("end");
        }
        return s;
    }

    /** The line that opens the routine after the pick (the children), or null in "my nodes" mode. */
    String afterPickOpen() {
        return ownGripperNodes() ? null : "if " + foundVariable + ":";
    }

    /** The whole contribution with {@code children} where the child nodes go — tests and previews. */
    String render(String children) {
        StringBuilder b = new StringBuilder();
        for (String l : beforeChildren()) b.append(l).append('\n');
        boolean own = ownGripperNodes();
        if (own && children != null && !children.isEmpty()) b.append(children).append('\n');
        for (String l : afterChildren()) b.append(l).append('\n');
        if (!own) {
            b.append(afterPickOpen()).append('\n');
            if (children != null && !children.isEmpty()) b.append(children).append('\n');
            b.append("end\n");
        }
        return b.toString();
    }

    // -- the gripper ---------------------------------------------------------------------

    /** At the start of the run: connect (Robotiq), activate if it lost it, open fully without waiting. */
    private void gripperStart(List<String> s, String in) {
        if ("robotiq".equals(gripper)) {
            s.add(in + "rs_ok = socket_open(\"127.0.0.1\", 63352, \"" + RQ_SOCKET + "\")");
            s.add(in + "if rs_ok:");
            s.add(in + "  socket_send_line(\"GET ACT\", \"" + RQ_SOCKET + "\")");
            s.add(in + "  rs_act = socket_read_string(\"" + RQ_SOCKET + "\", timeout=2.0)");
            s.add(in + "  if str_find(rs_act, \"1\") < 0:");
            say(s, in + "    ", "activating the gripper", null, true);
            rq(s, in + "    ", "ACT", 1);
            s.add(in + "    rs_t = 0");
            s.add(in + "    while rs_t < 40:");
            s.add(in + "      socket_send_line(\"GET STA\", \"" + RQ_SOCKET + "\")");
            s.add(in + "      if str_find(socket_read_string(\"" + RQ_SOCKET + "\", timeout=2.0), \"3\") >= 0:");
            s.add(in + "        rs_t = 40");
            s.add(in + "      else:");
            s.add(in + "        sleep(0.25)");
            s.add(in + "        rs_t = rs_t + 1");
            s.add(in + "      end");
            s.add(in + "    end");
            s.add(in + "  end");
            rq(s, in + "  ", "SPE", (int) Math.round(n("gripperSpeedPct") * 2.55));
            rq(s, in + "  ", "FOR", (int) Math.round(n("gripperForcePct") * 2.55));
            gripperOpen(s, in + "  ");
            s.add(in + "else:");
            s.add(in + "  rs_why = \"no Robotiq gripper on this controller (is its URCap installed?) - or choose"
                    + " another gripper in the node's Options\"");
            s.add(in + "end");
        } else if ("digital".equals(gripper)) {
            gripperOpen(s, in);
        }
    }

    /** Open fully, without waiting (the fingers open while the arm moves). */
    private void gripperOpen(List<String> s, String in) {
        if ("robotiq".equals(gripper)) {
            rq(s, in, "POS", 0);
            rq(s, in, "GTO", 1);
        } else if ("digital".equals(gripper)) {
            s.add(in + "set_standard_digital_out(" + (int) n("gripperDo") + ", False)");
        }
    }

    /** At the approach: the fingers must be fully open before the tool goes down. */
    private void gripperOpenWait(List<String> s, String in) {
        if (!"robotiq".equals(gripper)) return; // a digital output opened at the start and after every miss
        s.add(in + "rs_t = 0");
        s.add(in + "while rs_t < 30:");
        s.add(in + "  socket_send_line(\"GET OBJ\", \"" + RQ_SOCKET + "\")");
        s.add(in + "  if str_find(socket_read_string(\"" + RQ_SOCKET + "\", timeout=2.0), \"3\") >= 0:");
        s.add(in + "    rs_t = 30");
        s.add(in + "  else:");
        s.add(in + "    sleep(0.1)");
        s.add(in + "    rs_t = rs_t + 1");
        s.add(in + "  end");
        s.add(in + "end");
    }

    private void gripperClose(List<String> s, String in) {
        if ("robotiq".equals(gripper)) {
            rq(s, in, "POS", 255);
            rq(s, in, "GTO", 1);
            s.add(in + "sleep(0.2)");
            s.add(in + "rs_obj = \"\"");
            s.add(in + "rs_t = 0");
            s.add(in + "while rs_t < 40:");
            s.add(in + "  socket_send_line(\"GET OBJ\", \"" + RQ_SOCKET + "\")");
            s.add(in + "  rs_obj = socket_read_string(\"" + RQ_SOCKET + "\", timeout=2.0)");
            s.add(in + "  if str_find(rs_obj, \"0\") < 0:");
            s.add(in + "    rs_t = 40");
            s.add(in + "  else:");
            s.add(in + "    sleep(0.1)");
            s.add(in + "    rs_t = rs_t + 1");
            s.add(in + "  end");
            s.add(in + "end");
            // OBJ 1/2: stopped on contact = holding it; 3: closed all the way = nothing between the fingers
            s.add(in + "rs_held = (str_find(rs_obj, \"1\") >= 0) or (str_find(rs_obj, \"2\") >= 0)");
            say(s, in, "gripper ", "rs_obj", true);
        } else {
            s.add(in + "set_standard_digital_out(" + (int) n("gripperDo") + ", True)");
            s.add(in + "sleep(" + f2(n("gripperWaitS")) + ")");
            s.add(in + "rs_held = True");
        }
    }

    private static void rq(List<String> s, String in, String var, int value) {
        s.add(in + "socket_send_line(\"SET " + var + " " + value + "\", \"" + RQ_SOCKET + "\")");
        s.add(in + "rs_ack = socket_read_string(\"" + RQ_SOCKET + "\", timeout=2.0)");
    }

    // -- helpers ---------------------------------------------------------------------------

    private static void read16(List<String> s, String in) {
        s.add(in + "rs_r = socket_read_ascii_float(16, \"" + SOCKET + "\", 10)");
        // index rs_r[1] only after a full read: a timed-out read has nothing there
        s.add(in + "rs_st = -8");
        s.add(in + "if rs_r[0] == 16:");
        s.add(in + "  rs_st = rs_r[1]");
        s.add(in + "end");
    }

    private static void reasons(List<String> s, String in) {
        s.add(in + "rs_why = str_cat(\"pick server status \", to_str(rs_st))");
        for (int i = 0; i < REASONS.length; i++) {
            s.add(in + (i == 0 ? "if" : "elif") + " rs_st == " + REASONS[i][0] + ":");
            s.add(in + "  rs_why = \"" + REASONS[i][1] + "\"");
        }
        s.add(in + "end");
        // no camera computer, no calibration, a request it can't read: nothing later in this run fixes it
        s.add(in + "if (rs_st == -3) or (rs_st == -8) or (rs_st == -9):");
        s.add(in + "  rs_ok = False");
        s.add(in + "end");
    }

    /**
     * One stage report: {@code textmsg} for the Log tab and a {@code LOG} line to the pick
     * server. {@code text} is a constant this class writes (ASCII, no quotes); {@code value}
     * an optional URScript expression.
     */
    private static void say(List<String> s, String indent, String text, String value, boolean socket) {
        String v = value == null ? "\"\"" : value;
        s.add(indent + "textmsg(\"Perceptronic Pick: " + text + "\", " + v + ")");
        if (socket) {
            String line = value == null ? "\"LOG " + text + "\"" : "str_cat(\"LOG " + text + "\", to_str(" + value + "))";
            s.add(indent + "socket_send_line(" + line + ", \"" + SOCKET + "\")");
        }
    }

    /** One decimal at most, no trailing zeros, never a locale's comma. */
    static String num(double v) {
        String t = String.format(Locale.ROOT, "%.1f", v);
        return t.endsWith(".0") ? t.substring(0, t.length() - 2) : t;
    }

    private static String f2(double v) {
        return String.format(Locale.ROOT, "%.2f", v);
    }

    static String pose(double[] p) {
        StringBuilder b = new StringBuilder("p[");
        for (int i = 0; i < 6; i++) {
            if (i > 0) b.append(", ");
            b.append(String.format(Locale.ROOT, "%.5f", p[i]));
        }
        return b.append(']').toString();
    }

    private static String joints(double[] q) {
        StringBuilder b = new StringBuilder("[");
        for (int i = 0; i < q.length; i++) {
            if (i > 0) b.append(", ");
            b.append(String.format(Locale.ROOT, "%.6f", q[i]));
        }
        return b.append(']').toString();
    }

    private static String m(double mm) {
        return String.format(Locale.ROOT, "%.4f", mm / 1000.0);
    }

    private static boolean finite(double[] xs, int n) {
        if (xs == null || xs.length != n) return false;
        for (double x : xs) {
            if (Double.isNaN(x) || Double.isInfinite(x)) return false;
        }
        return true;
    }

    static boolean contains(String[] xs, String x) {
        for (String s : xs) {
            if (s.equals(x)) return true;
        }
        return false;
    }
}
