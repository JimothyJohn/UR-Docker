package io.advin.perceptronic;

import java.net.URI;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * The URScript a 3D Pick node contributes — generated here with no UR API, so it is tested on
 * any JDK — and the node's settings (one table, {@link #NUMBERS}: key, default, limits,
 * label; the Options screen and the data model both read it).
 *
 * <p>One run of the node is one move sequence that <b>positions</b> the tool on one part and
 * touches nothing else: <b>the node does not drive the gripper</b> (0.8.0, Nick 2026-10-01).
 * The program opens the gripper before the node and closes it after; the node starts with
 * the survey and ends with the fingertips at the grip, around the part. It has no children.
 * It runs inside the operator's program, so it works in Local mode and needs no Primary
 * client:
 *
 * <ol>
 *   <li>force the TCP to the flange (the pick server answers in flange poses);</li>
 *   <li>ask the pick server {@code NEXT}: a part it already saw at a picture point (the arm
 *       then goes straight to the closer look over it — no trip back for a picture) or the
 *       picture point to look from;</li>
 *   <li>the survey, at a picture point: {@code movej} there, settle, {@code FIND} — the
 *       server finds the parts by their volume, keeps those the arm's kinematics reach and
 *       that have the finger room asked for, numbers them in the chosen order and answers
 *       the first; an empty picture point sends the arm on to the next one;</li>
 *   <li>the closer look, unless switched off ({@code LOOK}: the camera halfway to the part,
 *       the part clear of the gripper in the picture) and {@code REFINE} — straight down,
 *       then leaned 12° and 24° while the controller's own IK can't solve approach + grip.
 *       With the closer look off every part is measured from its picture point;</li>
 *   <li>the approach — fingertips {@code approachMm} over the part's top — and down to the
 *       grip: the fingers across the part's short side, or its long one;</li>
 *   <li>the result variable is True once the tool is at the grip, the location variable says
 *       which picture point the part came from, and the operator's TCP is back. Anything
 *       that stops short says why in a popup.</li>
 * </ol>
 */
final class PickScript {
    static final String VERSION = "0.9.0";
    static final int DEFAULT_PICK_PORT = 7622;
    static final String SOCKET = "rs_pick";
    static final int MAX_POINTS = 12;
    static final String[] ORDERS = {"LR", "RL", "FB", "BF"};
    static final String[] SHAPES = {"box", "cyl"};
    static final double SPEED = 0.6; // of the node's own joint / linear limits
    static final double SETTLE_S = 0.2; // before each picture
    static final int MAX_ATTEMPTS = 3; // grasps per run
    /** How wide the open fingers are taken to be when the closer look keeps the part clear of them (mm). */
    static final double LOOK_STROKE_MM = 50;

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
        {"-10", "the only parts in view are out of the arm's reach"},
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
                "fingertips over the top"),
        new Num("gripBelowTopMm", "approach", "Grip depth", "mm", 15, 0, 60, 1,
                "fingertips below the top to grip"),
        new Num("fingerRoomMm", "approach", "Finger room", "mm", 20, 0, 100, 5,
                "clear space on each side of the part"),
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
    /** "box" (length × width × height) or "cyl": an upright cylinder, {@code partLengthMm} its diameter. */
    String shape = "box";
    /** Skip a part that has less than {@code fingerRoomMm} of clear space on either side it is gripped from. */
    boolean gripCheck = true;
    /** A box: the fingers close across its long side instead of its short one. */
    boolean gripLongSide = false;
    /** Move in for a second, closer measurement before the approach. */
    boolean closeLook = true;
    /** PolyScope's name for the arm ("UR3"): the pick server asks its kinematics what is in reach. "": not sent. */
    String arm = "";
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
        if (points.isEmpty()) return "add a picture point: move the arm where the camera sees the parts and tap +";
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
        if (!contains(SHAPES, shape)) return "unknown part shape \"" + shape + "\"";
        if (!arm.isEmpty() && !arm.matches("[A-Za-z0-9]{1,8}")) return "the robot model \"" + arm + "\" is not a name";
        if (n("gripBelowTopMm") > n("partHeightMm") - 2) {
            return "the grip (" + num(n("gripBelowTopMm")) + " mm below the top) would put the fingertips on the table:"
                    + " the part is " + num(n("partHeightMm")) + " mm tall";
        }
        if (!isOrder(orderFirst, orderRows)) return "the pick order must be one horizontal and one vertical direction";
        if (!foundVariable.matches("[A-Za-z_][A-Za-z0-9_]*") || !locVariable.matches("[A-Za-z_][A-Za-z0-9_]*")) {
            return "the result variable names are not valid";
        }
        return null;
    }

    boolean round() {
        return "cyl".equals(shape);
    }

    /** The footprint's long side (a cylinder: its diameter). */
    double longSide() {
        return round() ? n("partLengthMm") : Math.max(n("partLengthMm"), n("partWidthMm"));
    }

    /** The side the fingers close across (a cylinder: its diameter). */
    double shortSide() {
        return round() ? n("partLengthMm") : Math.min(n("partLengthMm"), n("partWidthMm"));
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
        b.append(" part=").append(num(longSide())).append('x').append(num(shortSide())).append('x')
                .append(num(n("partHeightMm")));
        b.append(" tol=").append(num(n("partTolPct")));
        if (round()) b.append(" shape=cyl");
        b.append(" order=").append(orderFirst).append(',').append(orderRows);
        b.append(" grip=").append(num(n("gripBelowTopMm")));
        b.append(" approach=").append(num(n("approachMm")));
        b.append(" gripcheck=").append(gripCheck ? 1 : 0).append(" room=").append(num(n("fingerRoomMm")));
        if (gripLongSide && !round()) b.append(" across=long");
        if (!arm.isEmpty()) b.append(" arm=").append(arm);
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

    /**
     * {@link #tokens} without the part's size (no {@code part=}, {@code tol=}, {@code shape=})
     * and without the grip check: what the Part step asks for when the operator taps the
     * picture, so everything in view is measured whatever size the node holds now (0.9.0).
     */
    String teachTokens(int i) {
        StringBuilder b = new StringBuilder();
        for (String t : tokens(i).trim().split(" ")) {
            if (t.startsWith("part=") || t.startsWith("tol=") || t.startsWith("shape=") || t.startsWith("room=")
                    || t.startsWith("across=")) {
                continue;
            }
            b.append(' ').append(t.startsWith("gripcheck=") ? "gripcheck=0" : t);
        }
        return b.toString();
    }

    /** ASCII (it goes into the script). */
    String partText() {
        return (round() ? "cylinder D" + num(longSide()) : num(longSide()) + " x " + num(shortSide())) + " x "
                + num(n("partHeightMm")) + " mm +-" + num(n("partTolPct")) + " %";
    }

    static String orderText(String first, String rows) {
        return words(first) + ", rows " + words(rows);
    }

    static String words(String o) {
        return "LR".equals(o) ? "left to right" : "RL".equals(o) ? "right to left"
                : "FB".equals(o) ? "front to back" : "back to front";
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

    /** The node's whole contribution, line by line. */
    List<String> lines() {
        String why = problem();
        if (why != null) throw new IllegalStateException(why);
        List<String> s = new ArrayList<String>();
        int np = points.size();
        int budget = np + MAX_ATTEMPTS + 1;
        String jv = f2(1.05 * SPEED);
        String ja = f2(1.4 * SPEED);
        String lv = f2(0.25 * SPEED);
        String la = f2(0.6 * SPEED);
        String settle = f2(SETTLE_S);
        s.add("# 3D Pick " + VERSION + " - camera computer " + host + ":" + port + " - part " + partText()
                + " - " + orderText(orderFirst, orderRows) + " - " + np + " picture point" + (np == 1 ? "" : "s")
                + (closeLook ? "" : " - no closer look")
                + (gripCheck ? " - finger room " + num(n("fingerRoomMm")) + " mm" : " - no grip check")
                + (gripLongSide && !round() ? " - across the long side" : ""));
        if (!ikCheck()) {
            s.add("# PolyScope " + polyscope[0] + "." + polyscope[1] + "." + polyscope[2]
                    + ": no get_inverse_kin_has_solution - the closer look and the approach go unchecked");
        }
        s.add(foundVariable + " = False");
        s.add(locVariable + " = 0");
        s.add("rs_tcp0 = get_tcp_offset()");
        s.add("set_tcp(p[0, 0, 0, 0, 0, 0])");
        s.add("rs_why = \"no answer from the camera computer within 10 s\"");
        s.add("rs_try = 0");
        s.add("rs_ok = True");
        s.add("rs_loc = 1");
        s.add("rs_tok = \"\"");
        s.add("if socket_open(\"" + host + "\", " + port + ", \"" + SOCKET + "\"):");
        say(s, "  ", "start, flange ", "get_actual_tcp_pose()", true);
        say(s, "  ", "looking for " + partText(), null, true);
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
        if (closeLook) {
            s.add("    if rs_st == 1:");
            say(s, "      ", "next part already seen, #", "rs_r[12]", true);
            s.add("    else:");
            survey(s, "      ", ja, jv, settle);
            s.add("    end");
        } else {
            // every part is measured from its picture point: the parts queued at the last one
            // were seen from there too, but the arm is no longer where it could look again
            survey(s, "    ", ja, jv, settle);
        }
        s.add("    if rs_st == 1:");
        s.add("      rs_c = p[rs_r[2], rs_r[3], rs_r[4], 0, 0, 0]");
        s.add("      rs_top = p[rs_r[5], rs_r[6], rs_r[7], rs_r[8], rs_r[9], rs_r[10]]");
        if (closeLook) {
            s.add("      socket_send_line(str_cat(\"LOOK \", str_cat(to_str(get_actual_tcp_pose()), str_cat(\" \","
                    + " str_cat(to_str(rs_c), \" stroke=" + num(LOOK_STROKE_MM) + "\")))), \"" + SOCKET + "\")");
            // its own list: up to PolyScope 5.14 a list keeps its first size ("Resizing of 'List' is
            // not supported" when rs_r took 10 numbers after 16 — URControl.log of 5.9.4 .. 5.14.6 in
            // the matrix, 2026-09-29; 5.15.2 on resize it)
            s.add("      rs_lk = socket_read_ascii_float(10, \"" + SOCKET + "\", 10)");
            s.add("      rs_see = False");
            s.add("      rs_look = rs_top");
            s.add("      if rs_lk[0] == 10:");
            s.add("        if rs_lk[1] == 1:");
            s.add("          rs_look = p[rs_lk[5], rs_lk[6], rs_lk[7], rs_lk[8], rs_lk[9], rs_lk[10]]");
            s.add("          rs_see = True");
            s.add("        end");
            s.add("      end");
            if (ikCheck()) {
                s.add("      if rs_see:");
                s.add("        rs_see = get_inverse_kin_has_solution(rs_look, get_actual_joint_positions())");
                s.add("      end");
            }
            s.add("      if rs_see:");
            s.add("        movej(get_inverse_kin(rs_look, get_actual_joint_positions()), a=" + ja + ", v=" + jv + ")");
            s.add("        sleep(" + settle + ")");
            say(s, "        ", "closer look ", "rs_look", true);
            s.add("      else:");
            say(s, "        ", "no closer look from here - measuring again where the arm is", null, true);
            s.add("      end");
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
        if (ikCheck()) {
            s.add("          rs_q = get_actual_joint_positions()");
            s.add("          if get_inverse_kin_has_solution(rs_hover, rs_q) and get_inverse_kin_has_solution(rs_grip,"
                    + " rs_q):");
            s.add("            rs_go = True");
            s.add("          else:");
            say(s, "            ", "no IK for approach + grip at lean ", "rs_leans[rs_lean]", true);
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
        say(s, "        ", "down to the grip ", "rs_grip", true);
        s.add("        movel(rs_grip, a=0.3, v=0.05)");
        s.add("        " + foundVariable + " = True");
        s.add("        " + locVariable + " = rs_loc");
        say(s, "        ", "at the grip, part from point ", "rs_loc", true);
        s.add("      elif rs_rs != 1:");
        s.add("        rs_st = rs_rs");
        reasons(s, "        ");
        s.add("      else:");
        s.add("        rs_why = \"the controller has no joint solution for the approach (straight down or leaned to"
                + " 24 deg)\"");
        s.add("      end");
        s.add("    end");
        s.add("  end");
        s.add("  if " + foundVariable + " == False:");
        s.add("    textmsg(\"3D Pick: no pick - \", rs_why)");
        s.add("    socket_send_line(str_cat(\"LOG no pick - \", rs_why), \"" + SOCKET + "\")");
        s.add("  end");
        s.add("  socket_close(\"" + SOCKET + "\")");
        s.add("else:");
        s.add("  rs_why = \"no camera computer at " + host + ":" + port
                + " - is it on, and is the address in Installation > Perceptronic right?\"");
        s.add("  textmsg(\"3D Pick: \", rs_why)");
        s.add("end");
        s.add("set_tcp(rs_tcp0)");
        if (popupOnFail) {
            s.add("if " + foundVariable + " == False:");
            s.add("  popup(str_cat(\"3D Pick: no pick - \", rs_why), \"3D Pick\", False, True, blocking=True)");
            s.add("end");
        }
        return s;
    }

    /** The survey: to picture point {@code rs_loc}, settle, FIND. */
    private void survey(List<String> s, String in, String ja, String jv, String settle) {
        for (int i = 0; i < points.size(); i++) {
            s.add(in + (i == 0 ? "if" : "elif") + " rs_loc == " + (i + 1) + ":");
            s.add(in + "  movej(" + joints(points.get(i).joints) + ", a=" + ja + ", v=" + jv + ")");
        }
        s.add(in + "end");
        s.add(in + "sleep(" + settle + ")");
        say(s, in, "survey at point ", "rs_loc", true);
        s.add(in + "socket_send_line(str_cat(\"FIND \", str_cat(to_str(get_actual_tcp_pose()), rs_tok)), \""
                + SOCKET + "\")");
        read16(s, in);
        say(s, in, "FIND status ", "rs_st", true);
        s.add(in + "if rs_st != 1:");
        reasons(s, in + "  ");
        s.add(in + "end");
    }

    /** The whole contribution as one text — tests and previews. */
    String render() {
        StringBuilder b = new StringBuilder();
        for (String l : lines()) b.append(l).append('\n');
        return b.toString();
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
        s.add(indent + "textmsg(\"3D Pick: " + text + "\", " + v + ")");
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
