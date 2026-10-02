package io.advin.perceptronic;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.geom.Ellipse2D;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;
import java.util.Locale;
import javax.swing.JComponent;

/**
 * The small drawings that explain a setting as it changes: the pick-order tiles, the part
 * (box or cylinder) with its dimensions, the approach seen from the side, and the cell from
 * above (the arm's reach and the taught pick areas). No UR API.
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
final class Diagrams {
    private Diagrams() {
    }

    /**
     * The pick number of each cell of a {@code cols × rows} grid (row 0 = the top of the
     * picture) for order {@code first} within a row and {@code rowsDir} from row to row —
     * the same rule as {@code perceptronics.volume.order_parts}.
     */
    static int[][] orderGrid(final String first, final String rowsDir, int cols, int rows) {
        List<int[]> cells = new ArrayList<int[]>();
        for (int r = 0; r < rows; r++) {
            for (int c = 0; c < cols; c++) cells.add(new int[] {c, r});
        }
        cells.sort(Comparator.<int[]>comparingInt(cell -> key(rowsDir, cell)).thenComparingInt(cell -> key(first, cell)));
        int[][] out = new int[rows][cols];
        for (int i = 0; i < cells.size(); i++) out[cells.get(i)[1]][cells.get(i)[0]] = i + 1;
        return out;
    }

    private static int key(String dir, int[] cell) {
        if ("LR".equals(dir)) return cell[0];
        if ("RL".equals(dir)) return -cell[0];
        if ("FB".equals(dir)) return -cell[1]; // front = the bottom of the picture, first
        return cell[1];
    }

    /** One pick-order choice: a 3 × 2 grid of parts with the numbers they'd get, and a path through them. */
    static final class OrderTile extends JComponent {
        final String first;
        final String rows;
        private boolean selected;

        OrderTile(String first, String rows, final Runnable onPick) {
            this.first = first;
            this.rows = rows;
            setToolTipText(PickScript.orderText(first, rows));
            setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseReleased(MouseEvent e) {
                    onPick.run();
                }
            });
        }

        void setSelected(boolean on) {
            selected = on;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(62, 50);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            g.setColor(selected ? Ui.ACCENT_SOFT : Ui.CARD);
            g.fillRoundRect(1, 1, w - 3, h - 3, 12, 12);
            g.setColor(selected ? Ui.ACCENT : Ui.LINE);
            g.setStroke(new BasicStroke(selected ? 2.2f : 1f));
            g.drawRoundRect(1, 1, w - 3, h - 3, 12, 12);
            int[][] n = orderGrid(first, rows, 3, 2);
            int[][] at = new int[7][];
            for (int r = 0; r < 2; r++) {
                for (int c = 0; c < 3; c++) at[n[r][c]] = new int[] {12 + c * (w - 24) / 2, 15 + r * (h - 30)};
            }
            g.setColor(selected ? Ui.ACCENT : Ui.FAINT);
            g.setStroke(new BasicStroke(1.3f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            for (int i = 1; i < 6; i++) g.drawLine(at[i][0], at[i][1], at[i + 1][0], at[i + 1][1]);
            g.setFont(Ui.font(10f, true));
            for (int i = 1; i <= 6; i++) {
                g.setColor(i == 1 ? (selected ? Ui.ACCENT : Ui.INK) : (selected ? new Color(0x7ea6ec) : new Color(0xa8b3c0)));
                g.fill(new Ellipse2D.Double(at[i][0] - 8, at[i][1] - 8, 16, 16));
                g.setColor(Color.WHITE);
                Ui.centre(g, String.valueOf(i), at[i][0], at[i][1]);
            }
            g.dispose();
        }
    }

    /** The part, drawn in proportion, with its dimensions: a box, or a cylinder standing on its end. */
    static final class PartDrawing extends JComponent {
        private double l = 50;
        private double wd = 30;
        private double ht = 30;
        private boolean round;

        void set(double length, double width, double height, boolean cylinder) {
            l = Math.max(length, width);
            wd = Math.min(length, width);
            ht = height;
            round = cylinder;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(300, 240);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            if (round) paintCylinder(g, getWidth(), getHeight());
            else paintBox(g, getWidth(), getHeight());
            g.dispose();
        }

        private void paintCylinder(Graphics2D g, int w, int h) {
            double k = Math.min(Math.min((w - 130) / l, (h - 50) / (ht + l * 0.35)), 4.0);
            int rx = (int) (l * k / 2), ry = (int) (l * k * 0.35 / 2), hh = (int) (ht * k);
            int cx = w / 2 - 14, top = (h - hh - 2 * ry) / 2 + ry, bot = top + hh;
            g.setColor(new Color(0xb4cdf5));
            g.fillRect(cx - rx, top, 2 * rx, hh);
            g.fillOval(cx - rx, bot - ry, 2 * rx, 2 * ry);
            g.setColor(new Color(0xe8f0fd));
            g.fillOval(cx - rx, top - ry, 2 * rx, 2 * ry);
            g.setColor(Ui.ACCENT);
            g.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g.drawOval(cx - rx, top - ry, 2 * rx, 2 * ry);
            g.drawArc(cx - rx, bot - ry, 2 * rx, 2 * ry, 180, 180);
            g.drawLine(cx - rx, top, cx - rx, bot);
            g.drawLine(cx + rx, top, cx + rx, bot);
            g.setFont(Ui.font(12f, true));
            g.setColor(Ui.INK);
            Ui.centre(g, "Ø " + PickScript.num(l), cx, top);
            Ui.centre(g, PickScript.num(ht), cx + rx + 22, (top + bot) / 2);
        }

        private void paintBox(Graphics2D g, int w, int h) {
            double cos30 = Math.cos(Math.toRadians(30)), sin30 = 0.5;
            double span = (l + wd) * cos30, tall = ht + (l + wd) * sin30;
            double k = Math.min(Math.min((w - 110) / span, (h - 44) / tall), 4.0);
            double ox = (w - span * k) / 2 + wd * cos30 * k, oy = h - 22 - (h - 44 - tall * k) / 2;
            // iso: x along the length (right-up), y along the width (left-up), z up
            double[][] base = {{0, 0}, {l, 0}, {l, wd}, {0, wd}};
            int[][] bot = new int[4][], top = new int[4][];
            for (int i = 0; i < 4; i++) {
                double x = base[i][0], y = base[i][1];
                double sx = ox + (x - y) * cos30 * k, sy = oy - ((x + y) * sin30) * k;
                bot[i] = new int[] {(int) sx, (int) sy};
                top[i] = new int[] {(int) sx, (int) (sy - ht * k)};
            }
            Polygon topFace = poly(top[0], top[1], top[2], top[3]);
            Polygon right = poly(bot[0], bot[1], top[1], top[0]);
            Polygon left = poly(bot[3], bot[0], top[0], top[3]);
            g.setColor(new Color(0xcfe0fb));
            g.fillPolygon(right);
            g.setColor(new Color(0xb4cdf5));
            g.fillPolygon(left);
            g.setColor(new Color(0xe8f0fd));
            g.fillPolygon(topFace);
            g.setColor(Ui.ACCENT);
            g.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            g.drawPolygon(topFace);
            g.drawPolygon(right);
            g.drawPolygon(left);
            g.setFont(Ui.font(12f, true));
            g.setColor(Ui.INK);
            Ui.centre(g, PickScript.num(l), (bot[0][0] + bot[1][0]) / 2 + 10, (bot[0][1] + bot[1][1]) / 2 + 13);
            Ui.centre(g, PickScript.num(wd), (bot[3][0] + bot[0][0]) / 2 - 12, (bot[3][1] + bot[0][1]) / 2 + 13);
            Ui.centre(g, PickScript.num(ht), bot[1][0] + 18, (bot[1][1] + top[1][1]) / 2);
        }

        private static Polygon poly(int[]... pts) {
            Polygon p = new Polygon();
            for (int[] q : pts) p.addPoint(q[0], q[1]);
            return p;
        }
    }

    /** The approach from the side: the table, the part, the open fingers over it and how deep they grip. */
    static final class ApproachDrawing extends JComponent {
        private double approach = 25;
        private double grip = 15;
        private double height = 30;
        private double width = 30;
        private double room = 20;

        /** {@code widthMm}: the side the fingers close across; {@code roomMm}: the clear space checked beside it (0: none). */
        void set(double approachMm, double gripMm, double heightMm, double widthMm, double roomMm) {
            approach = approachMm;
            grip = gripMm;
            height = heightMm;
            width = widthMm;
            room = roomMm;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(260, 220);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            double open = width + 2 * Math.max(4, room * 0.5); // the fingers, somewhere inside the room
            double total = height + approach + 45; // mm shown top to bottom
            double k = Math.min(Math.min((h - 26) / total, (w - 190) / (Math.max(open, width + 2 * room) + 20)), 3.0);
            int table = h - 14;
            int cx = w / 2 - 50;
            g.setColor(new Color(0xe6eaf0));
            g.fillRect(0, table, w, 14);
            g.setColor(Ui.FAINT);
            g.drawLine(0, table, w, table);
            int pw = (int) (width * k), ph = (int) (height * k);
            g.setColor(new Color(0xcfe0fb));
            g.fillRect(cx - pw / 2, table - ph, pw, ph);
            g.setColor(Ui.ACCENT);
            g.drawRect(cx - pw / 2, table - ph, pw, ph);
            int topY = table - ph;
            // fingers, fully open, tips `approach` over the top
            int tipY = topY - (int) (approach * k);
            int half = (int) (open * k / 2);
            g.setColor(new Color(0x3b4756));
            for (int sx : new int[] {-1, 1}) {
                int x = cx + sx * half;
                g.fillRoundRect(x - (sx < 0 ? 7 : 0), tipY - 38, 7, 38, 3, 3);
            }
            g.fillRoundRect(cx - half - 10, tipY - 50, 2 * half + 20, 12, 6, 6);
            // where the fingertips stop to grip
            g.setColor(Ui.JAW);
            g.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 1f, new float[] {4f, 4f}, 0f));
            int gripY = topY + (int) (grip * k);
            g.drawLine(cx - half - 16, gripY, cx + half + 16, gripY);
            g.setStroke(new BasicStroke(1.2f));
            g.setFont(Ui.font(11.5f, true));
            int dx = cx + half + 16;
            dim(g, dx, tipY, topY, "approach " + PickScript.num(approach), Ui.ACCENT);
            dim(g, dx, topY, gripY, "grip " + PickScript.num(grip), new Color(0x9a6a00));
            if (room > 0) {
                // the finger room: the clear space wanted on each side of the part
                int rw = (int) (room * k);
                g.setColor(new Color(61, 220, 132, 60));
                g.fillRect(cx - pw / 2 - rw, topY, rw, ph);
                g.fillRect(cx + pw / 2, topY, rw, ph);
                g.setColor(Ui.OK);
                g.drawString("room " + PickScript.num(room), cx + pw / 2 + 4, table - 4);
            }
            g.dispose();
        }

        private static void dim(Graphics2D g, int x, int y0, int y1, String text, Color c) {
            g.setColor(c);
            g.drawLine(x, y0, x, y1);
            g.drawLine(x - 4, y0, x + 4, y0);
            g.drawLine(x - 4, y1, x + 4, y1);
            g.drawString(text, x + 6, (y0 + y1) / 2 + 4);
        }
    }

    /**
     * The cell from above: the base, how far the arm reaches (its rated reach, drawn and
     * labelled), the keep-out round the base, and every taught pick area — so the operator
     * sees at once where an area sits in the arm's reach. Which parts are actually pickable
     * is the arm's kinematics' answer, part by part; this is the map.
     */
    static final class ReachMap extends JComponent {
        private double baseR = 0.064;
        private double keepOutR = 0.214;
        private double reachR = 0.5;
        private String model = "";
        private final List<double[]> areas = new ArrayList<double[]>(); // each: 4 corners (x, y) m
        private final List<String> names = new ArrayList<String>();
        private int highlight = -1;

        /** {@code reach} 0: the model is unknown, no reach circle. */
        void set(String modelName, double baseRadius, double keepOut, double reach, List<double[]> corners,
                List<String> labels, int hi) {
            model = modelName == null ? "" : modelName;
            baseR = baseRadius;
            keepOutR = keepOut;
            reachR = reach;
            areas.clear();
            areas.addAll(corners);
            names.clear();
            names.addAll(labels);
            highlight = hi;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(300, 300);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            double extent = Math.max(reachR > 0 ? reachR : keepOutR * 2, keepOutR) * 1.12;
            for (double[] a : areas) {
                for (int i = 0; i < 8; i += 2) extent = Math.max(extent, Math.max(Math.abs(a[i]), Math.abs(a[i + 1])) * 1.1);
            }
            double k = (Math.min(w, h - 34) / 2.0 - 6) / extent;
            int cx = w / 2, cy = 12 + (h - 34) / 2;
            g.setColor(Ui.BG);
            g.fillRoundRect(0, 0, w, h, 14, 14);
            if (reachR > 0) {
                int R = (int) (reachR * k);
                g.setColor(new Color(0xe2f5eb));
                g.fillOval(cx - R, cy - R, 2 * R, 2 * R);
                g.setColor(Ui.OK);
                g.setStroke(new BasicStroke(1.8f));
                g.drawOval(cx - R, cy - R, 2 * R, 2 * R);
            }
            int r = (int) (keepOutR * k);
            g.setColor(new Color(0xfde3e3));
            g.fillOval(cx - r, cy - r, 2 * r, 2 * r);
            g.setColor(Ui.ERR);
            g.setStroke(new BasicStroke(1.2f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 1f, new float[] {5f, 4f}, 0f));
            g.drawOval(cx - r, cy - r, 2 * r, 2 * r);
            int b = (int) (baseR * k);
            g.setStroke(new BasicStroke(1.5f));
            g.setColor(new Color(0x3b4756));
            g.fillOval(cx - b, cy - b, 2 * b, 2 * b);
            // base axes: +X right, +Y up on the drawing
            g.setColor(Ui.ERR);
            g.drawLine(cx, cy, cx + b + 14, cy);
            g.setColor(Ui.OK);
            g.drawLine(cx, cy, cx, cy - b - 14);
            g.setFont(Ui.font(10f, true));
            g.setColor(Ui.MUTED);
            g.drawString("X", cx + b + 16, cy + 4);
            g.drawString("Y", cx - 3, cy - b - 16);
            for (int i = 0; i < areas.size(); i++) {
                double[] a = areas.get(i);
                Polygon p = new Polygon();
                double mx = 0, my = 0;
                for (int j = 0; j < 8; j += 2) {
                    p.addPoint(cx + (int) (a[j] * k), cy - (int) (a[j + 1] * k));
                    mx += a[j] / 4;
                    my += a[j + 1] / 4;
                }
                g.setColor(i == highlight ? new Color(28, 100, 216, 90) : new Color(28, 100, 216, 45));
                g.fillPolygon(p);
                g.setColor(Ui.ACCENT);
                g.setStroke(new BasicStroke(i == highlight ? 2.4f : 1.4f));
                g.drawPolygon(p);
                g.setColor(Ui.INK);
                g.setFont(Ui.font(11f, true));
                Ui.centre(g, i < names.size() ? names.get(i) : String.valueOf(i + 1), cx + (int) (mx * k),
                        cy - (int) (my * k));
            }
            if (reachR > 0) {
                // the reach, named on its own circle (drawn last: an area never hides it)
                int R = (int) (reachR * k);
                g.setFont(Ui.font(12f, true));
                String label = String.format(Locale.ROOT, "reach %.0f mm", reachR * 1000);
                int tw = g.getFontMetrics().stringWidth(label);
                g.setColor(Ui.OK);
                g.fillRoundRect(cx - tw / 2 - 8, cy - R - 9, tw + 16, 20, 20, 20);
                g.setColor(Color.WHITE);
                Ui.centre(g, label, cx, cy - R + 1);
            }
            g.setFont(Ui.font(11f, false));
            g.setColor(Ui.MUTED);
            g.drawString(reachR > 0 ? "green: the " + model + "'s reach · red: too near its base"
                    : "robot model unknown: no reach to draw", 8, h - 8);
            g.dispose();
        }
    }

    /** How far the farthest corner of a taught area is from the base axis (m). */
    static double farthest(double[] corners) {
        double far = 0;
        for (int i = 0; i < 8; i += 2) far = Math.max(far, Math.hypot(corners[i], corners[i + 1]));
        return far;
    }

    /** The four corners (x, y; m) of a taught area {@code [pose(6), sizeX, sizeY]}, for {@link ReachMap}. */
    static double[] corners(double[] plane) {
        double[][] m = PoseMath.matrix(Arrays.copyOf(plane, 6));
        double sx = plane[6], sy = plane[7];
        double[] out = new double[8];
        double[][] uv = {{0, 0}, {sx, 0}, {sx, sy}, {0, sy}};
        for (int i = 0; i < 4; i++) {
            out[2 * i] = m[0][3] + uv[i][0] * m[0][0] + uv[i][1] * m[0][1];
            out[2 * i + 1] = m[1][3] + uv[i][0] * m[1][0] + uv[i][1] * m[1][1];
        }
        return out;
    }
}
