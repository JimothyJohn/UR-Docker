package com.nickarmenta.perceptronic;

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
 * The small live drawings that explain a setting as it changes: the pick-order tiles, the
 * part with its dimensions, the approach seen from the side, and the cell from above (reach
 * and the taught pick areas). No UR API.
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

    /** The part, drawn in proportion, with its length, width and height. */
    static final class PartDrawing extends JComponent {
        private double l = 50;
        private double wd = 30;
        private double ht = 30;

        void set(double length, double width, double height) {
            l = Math.max(length, width);
            wd = Math.min(length, width);
            ht = height;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(140, 140);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            double cos30 = Math.cos(Math.toRadians(30)), sin30 = 0.5;
            double span = (l + wd) * cos30, tall = ht + (l + wd) * sin30;
            double k = Math.min((w - 30) / span, (h - 34) / tall);
            double ox = 15 + wd * cos30 * k, oy = h - 18 - 0.0;
            // iso: x along the length (right-down), y along the width (left-down), z up
            double[][] base = {{0, 0}, {l, 0}, {l, wd}, {0, wd}};
            int[][] bot = new int[4][], top = new int[4][];
            for (int i = 0; i < 4; i++) {
                double x = base[i][0], y = base[i][1];
                double sx = ox + (x - y) * cos30 * k, sy = oy - ((x + y) * sin30) * k;
                bot[i] = new int[] {(int) sx, (int) (sy - 0)};
                top[i] = new int[] {(int) sx, (int) (sy - ht * k)};
            }
            // shift so the drawing's lowest point (the near corner, index 0) sits at the bottom
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
            g.setFont(Ui.font(11.5f, true));
            g.setColor(Ui.INK);
            label(g, PickScript.num(l), (bot[0][0] + bot[1][0]) / 2 + 8, (bot[0][1] + bot[1][1]) / 2 + 12);
            label(g, PickScript.num(wd), (bot[3][0] + bot[0][0]) / 2 - 10, (bot[3][1] + bot[0][1]) / 2 + 12);
            label(g, PickScript.num(ht), bot[0][0] + 16, (bot[0][1] + top[0][1]) / 2 + 2);
            // the jaws: against the two long faces (the fingers close across the width)
            g.setColor(Ui.JAW);
            g.setStroke(new BasicStroke(4.5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            double ex = cos30, ey = -sin30; // screen direction of the length
            double len = l * cos30 * k * 0.22 / cos30;
            // near long face (width 0): its middle, pushed out toward the viewer (down-right)
            double fx = (bot[0][0] + bot[1][0] + top[0][0] + top[1][0]) / 4.0 + cos30 * 9;
            double fy = (bot[0][1] + bot[1][1] + top[0][1] + top[1][1]) / 4.0 + sin30 * 9;
            g.drawLine((int) (fx - ex * len), (int) (fy - ey * len), (int) (fx + ex * len), (int) (fy + ey * len));
            // far long face: along the top's far edge, pushed away (up-left)
            double bx = (top[2][0] + top[3][0]) / 2.0 - cos30 * 9, by = (top[2][1] + top[3][1]) / 2.0 - sin30 * 9 - 4;
            g.drawLine((int) (bx - ex * len), (int) (by - ey * len), (int) (bx + ex * len), (int) (by + ey * len));
            g.dispose();
        }

        private static Polygon poly(int[]... pts) {
            Polygon p = new Polygon();
            for (int[] q : pts) p.addPoint(q[0], q[1]);
            return p;
        }

        private static void label(Graphics2D g, String s, int x, int y) {
            Ui.centre(g, s, x, y);
        }
    }

    /** The approach from the side: the table, the part, the open fingers over it, the grip and the lift. */
    static final class ApproachDrawing extends JComponent {
        private double approach = 25;
        private double grip = 15;
        private double lift = 60;
        private double height = 30;
        private double width = 30;
        private double stroke = 50;

        void set(double approachMm, double gripMm, double liftMm, double heightMm, double widthMm, double strokeMm) {
            approach = approachMm;
            grip = gripMm;
            lift = liftMm;
            height = heightMm;
            width = widthMm;
            stroke = strokeMm;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(150, 170);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            double total = height + Math.max(approach, lift) + 40; // mm shown top to bottom
            double k = Math.min((h - 26) / total, (w - 80) / Math.max(stroke + 20, width + 20));
            int table = h - 14;
            int cx = w / 2 - 6;
            // table
            g.setColor(new Color(0xe6eaf0));
            g.fillRect(0, table, w, 14);
            g.setColor(Ui.FAINT);
            g.drawLine(0, table, w, table);
            // part
            int pw = (int) (width * k), ph = (int) (height * k);
            g.setColor(new Color(0xcfe0fb));
            g.fillRect(cx - pw / 2, table - ph, pw, ph);
            g.setColor(Ui.ACCENT);
            g.drawRect(cx - pw / 2, table - ph, pw, ph);
            int topY = table - ph;
            // fingers, fully open, tips `approach` over the top
            int tipY = topY - (int) (approach * k);
            int half = (int) (stroke * k / 2);
            g.setColor(new Color(0x3b4756));
            for (int sx : new int[] {-1, 1}) {
                int x = cx + sx * half;
                g.fillRoundRect(x - (sx < 0 ? 7 : 0), tipY - 38, 7, 38, 3, 3);
            }
            g.fillRoundRect(cx - half - 10, tipY - 50, 2 * half + 20, 12, 6, 6);
            // the grip depth
            g.setColor(Ui.JAW);
            g.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_BUTT, BasicStroke.JOIN_MITER, 1f, new float[] {4f, 4f}, 0f));
            int gripY = topY + (int) (grip * k);
            g.drawLine(cx - half - 16, gripY, cx + half + 16, gripY);
            // dimensions
            g.setStroke(new BasicStroke(1.2f));
            g.setFont(Ui.font(11f, true));
            int dx = cx + half + 14;
            dim(g, dx, tipY, topY, PickScript.num(approach), Ui.ACCENT);
            dim(g, dx, topY, gripY, PickScript.num(grip), new Color(0x9a6a00));
            int liftY = topY - (int) (lift * k);
            g.setColor(Ui.OK);
            int ax = cx - half - 18;
            g.drawLine(ax, gripY, ax, liftY);
            g.fillPolygon(new int[] {ax - 5, ax + 5, ax}, new int[] {liftY + 8, liftY + 8, liftY}, 3);
            g.drawString("lift " + PickScript.num(lift), Math.max(2, ax - 12), Math.max(12, liftY - 6));
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
     * The cell from above: the base, the ring the parts may be in (the reach limits), and every
     * taught pick area — so the operator sees at once that an area is out of reach.
     */
    static final class ReachMap extends JComponent {
        private double baseR = 0.064;
        private double minR = 0.214;
        private double maxR = 0.35;
        private final List<double[]> areas = new ArrayList<double[]>(); // each: 4 corners (x, y) m
        private final List<String> names = new ArrayList<String>();
        private int highlight = -1;

        void set(double baseRadius, double min, double max, List<double[]> corners, List<String> labels, int hi) {
            baseR = baseRadius;
            minR = min;
            maxR = max;
            areas.clear();
            areas.addAll(corners);
            names.clear();
            names.addAll(labels);
            highlight = hi;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(240, 240);
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            double extent = Math.max(maxR > 0 ? maxR : minR * 2, minR) * 1.15;
            for (double[] a : areas) {
                for (int i = 0; i < 8; i += 2) extent = Math.max(extent, Math.max(Math.abs(a[i]), Math.abs(a[i + 1])) * 1.1);
            }
            double k = (Math.min(w, h) / 2.0 - 8) / extent;
            int cx = w / 2, cy = h / 2;
            g.setColor(Ui.BG);
            g.fillRoundRect(0, 0, w, h, 14, 14);
            if (maxR > 0) {
                int R = (int) (maxR * k);
                g.setColor(new Color(0xe2f5eb));
                g.fillOval(cx - R, cy - R, 2 * R, 2 * R);
                g.setColor(Ui.OK);
                g.drawOval(cx - R, cy - R, 2 * R, 2 * R);
            }
            int r = (int) (minR * k);
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
            g.setFont(Ui.font(10.5f, false));
            g.setColor(Ui.MUTED);
            g.drawString(String.format(Locale.ROOT, "pick between %.0f and %s mm from the base axis", minR * 1000,
                    maxR > 0 ? String.format(Locale.ROOT, "%.0f", maxR * 1000) : "∞"), 8, h - 8);
            g.dispose();
        }
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
