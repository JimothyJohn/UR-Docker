package com.nickarmenta.perceptronic;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * The Installation screen's "Pick areas" view — no UR API. A pick area is a patch of the work
 * surface taught by touching it with the fingertips at three points (its corner, a point
 * along one edge, a point on the far side): the program's detector then measures parts'
 * heights from exactly that plane and ignores anything outside it. Beside the list, the cell
 * from above: the base, the ring the parts must be in (the reach limits, set here too), and
 * every area — one out of reach shows at a glance.
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
final class LocationsScreen extends JPanel {
    interface Actions {
        void addArea();

        void teach(int area, int point);

        void rename(int area, JLabel anchor);

        void remove(int area);

        void select(int area);

        void setReach(String key, double value);

        void askReach(String key, JLabel anchor);
    }

    /** One pick area as the screen shows it. */
    static final class Area {
        final String name;
        final boolean[] taught; // corner, along the edge, far side
        final double[] plane; // [pose(6), sizeX, sizeY] once all three are taught, else null

        Area(String name, boolean[] taught, double[] plane) {
            this.name = name;
            this.taught = taught;
            this.plane = plane;
        }
    }

    static final String[] POINTS = {"Corner", "Edge", "Far side"};
    static final String[] POINT_HELP = {
        "touch the table at one corner of the area",
        "then a point along one edge — that edge is the area's X",
        "then any point on the opposite side",
    };

    private final Actions actions;
    private final JPanel list = Ui.column();
    private final Diagrams.ReachMap map = new Diagrams.ReachMap();
    private final Ui.Note note = new Ui.Note();
    private final Ui.Stepper inner;
    private final Ui.Stepper outer;
    private final Ui.Stepper tip;
    private final JLabel model = Ui.label("", 12f, false, Ui.MUTED);

    LocationsScreen(Actions actions) {
        this.actions = actions;
        setOpaque(false);
        setLayout(new BorderLayout(12, 0));
        setBorder(BorderFactory.createEmptyBorder(6, 0, 0, 0));

        JPanel left = Ui.column();
        JPanel head = new JPanel(new BorderLayout());
        head.setOpaque(false);
        head.add(Ui.label("Pick areas", 17f, true, Ui.INK), BorderLayout.WEST);
        JButton add = Ui.button("+  New area", Ui.Style.PRIMARY);
        add.addActionListener(e -> actions.addArea());
        head.add(add, BorderLayout.EAST);
        head.setMaximumSize(new Dimension(Integer.MAX_VALUE, Ui.TAP));
        left.add(Ui.left(head));
        left.add(Ui.left(Ui.label("touch the table with the fingertips at three points — the picture points look at these",
                11.5f, false, Ui.MUTED)));
        left.add(Box.createVerticalStrut(6));
        left.add(Ui.left(list));
        left.add(Box.createVerticalGlue());
        left.add(Ui.left(note));
        add(left, BorderLayout.CENTER);

        JPanel right = Ui.column();
        right.setPreferredSize(new Dimension(300, 10));
        right.add(Ui.left(Ui.label("Reach", 17f, true, Ui.INK)));
        right.add(Ui.left(model));
        right.add(Box.createVerticalStrut(4));
        map.setAlignmentX(Component.LEFT_ALIGNMENT);
        map.setMaximumSize(new Dimension(300, 250));
        right.add(map);
        right.add(Box.createVerticalStrut(6));
        inner = new Ui.Stepper("Inner margin", "past the base's outer radius", step("reachInnerMm", 10));
        outer = new Ui.Stepper("Outer margin", "inside the arm's rated reach", step("reachOuterMm", 10));
        tip = new Ui.Stepper("Fingertip length", "flange to fingertips, for the touches", step("tipMm", 1));
        right.add(inner);
        right.add(outer);
        right.add(tip);
        add(right, BorderLayout.EAST);
    }

    private Ui.Step step(final String key, final double by) {
        return new Ui.Step() {
            @Override
            public void step(int direction) {
                actions.setReach(key, direction * by);
            }

            @Override
            public void type(JLabel anchor) {
                actions.askReach(key, anchor);
            }
        };
    }

    /**
     * Refresh: the areas, which is selected, the reach ring ({@code baseM} the base's outer
     * radius, the margins in mm, {@code reachM} the rated reach; 0 when the model is unknown).
     */
    void show(final List<Area> areas, final int selected, final String modelName, final double baseM,
            final double reachM, final double innerMm, final double outerMm, final double tipMm) {
        PickScreen.onEdt(() -> {
            list.removeAll();
            List<double[]> corners = new ArrayList<double[]>();
            List<String> names = new ArrayList<String>();
            int hi = -1;
            for (int i = 0; i < areas.size(); i++) {
                list.add(row(i, areas.get(i), i == selected));
                list.add(Box.createVerticalStrut(6));
                if (areas.get(i).plane != null) {
                    if (i == selected) hi = corners.size();
                    corners.add(Diagrams.corners(areas.get(i).plane));
                    names.add(areas.get(i).name);
                }
            }
            if (areas.isEmpty()) {
                list.add(Ui.left(Ui.label("no pick area yet: without one the table is found live in every picture",
                        12.5f, false, Ui.MUTED)));
            }
            double min = baseM + innerMm / 1000.0;
            double max = reachM > 0 ? Math.max(0, reachM - outerMm / 1000.0) : 0;
            map.set(baseM, min, max, corners, names, hi);
            model.setText(reachM > 0
                    ? String.format(Locale.ROOT, "%s: base Ø%.0f mm, reach %.0f mm", modelName, baseM * 2000, reachM * 1000)
                    : "robot model unknown — set the margins by hand");
            inner.setValue(Ui.value(innerMm, "mm"));
            outer.setValue(Ui.value(outerMm, "mm"));
            tip.setValue(Ui.value(tipMm, "mm"));
            String warn = null;
            for (Area a : areas) {
                if (a.plane == null) continue;
                double tilt = PoseMath.tiltDeg(a.plane);
                if (tilt > 2.0) {
                    warn = a.name + " is tilted " + String.format(Locale.ROOT, "%.1f", tilt)
                            + "° from level — the table is flat, so re-teach it (touch the table, not a part)";
                }
            }
            note.set(warn != null ? warn : "Parts are picked only inside an area a picture point looks at, and only"
                    + " inside the reach ring.", warn != null ? Ui.Kind.WARN : Ui.Kind.INFO);
            revalidate();
            repaint();
        });
    }

    private JComponent row(final int i, Area a, boolean selected) {
        JPanel card = new JPanel(new BorderLayout(10, 4)) {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = Ui.smooth(g);
                g2.setColor(getBackground());
                g2.fillRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 14, 14);
                g2.setColor(getForeground());
                g2.drawRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 14, 14);
                g2.dispose();
            }
        };
        card.setOpaque(false);
        card.setBackground(selected ? Ui.ACCENT_SOFT : Ui.CARD);
        card.setForeground(selected ? Ui.ACCENT : Ui.LINE);
        card.setBorder(BorderFactory.createEmptyBorder(8, 10, 8, 8));
        JPanel head = new JPanel(new BorderLayout());
        head.setOpaque(false);
        JLabel name = Ui.label(a.name, 15f, true, Ui.INK);
        name.setToolTipText("tap to rename");
        name.addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mouseReleased(java.awt.event.MouseEvent e) {
                actions.rename(i, name);
            }
        });
        head.add(name, BorderLayout.WEST);
        String facts = a.plane == null ? "teach all three points"
                : String.format(Locale.ROOT, "%.0f × %.0f mm · table at z %.0f mm · tilt %.1f°", Math.abs(a.plane[6]) * 1000,
                        Math.abs(a.plane[7]) * 1000, a.plane[2] * 1000, PoseMath.tiltDeg(a.plane));
        head.add(Ui.label(facts, 12f, false, a.plane == null ? Ui.WARN : Ui.MUTED), BorderLayout.CENTER);
        ((JLabel) head.getComponent(1)).setHorizontalAlignment(JLabel.CENTER);
        JButton del = Ui.button("Delete", Ui.Style.GHOST);
        del.addActionListener(e -> actions.remove(i));
        head.add(del, BorderLayout.EAST);
        card.add(head, BorderLayout.NORTH);
        JPanel pts = new JPanel(new GridLayout(1, 3, 6, 0));
        pts.setOpaque(false);
        for (int k = 0; k < 3; k++) pts.add(pointButton(i, k, a.taught[k]));
        card.add(pts, BorderLayout.CENTER);
        card.addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mouseReleased(java.awt.event.MouseEvent e) {
                actions.select(i);
            }
        });
        card.setAlignmentX(Component.LEFT_ALIGNMENT);
        card.setMaximumSize(new Dimension(Integer.MAX_VALUE, 110));
        return card;
    }

    private JComponent pointButton(final int area, final int k, final boolean done) {
        JPanel p = new JPanel(new BorderLayout(6, 0)) {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = Ui.smooth(g);
                g2.setColor(done ? Ui.OK_SOFT : Ui.BG);
                g2.fillRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 12, 12);
                g2.dispose();
            }
        };
        p.setOpaque(false);
        p.setBorder(BorderFactory.createEmptyBorder(4, 8, 4, 4));
        JComponent tick = new JComponent() {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(24, 24);
            }

            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = Ui.smooth(g);
                int y = (getHeight() - 22) / 2;
                g2.setColor(done ? Ui.OK : Color.WHITE);
                g2.fillOval(1, y, 22, 22);
                g2.setColor(done ? Color.WHITE : Ui.FAINT);
                g2.drawOval(1, y, 22, 22);
                g2.setFont(Ui.font(11.5f, true));
                Ui.centre(g2, done ? "✓" : String.valueOf(k + 1), 12, y + 11);
                g2.dispose();
            }
        };
        p.add(tick, BorderLayout.WEST);
        JPanel t = Ui.column();
        t.add(Ui.left(Ui.label(POINTS[k], 12.5f, true, Ui.INK)));
        t.add(Ui.left(Ui.label(done ? "taught" : "not taught", 11f, false, done ? Ui.OK : Ui.MUTED)));
        p.add(t, BorderLayout.CENTER);
        JButton teach = Ui.button(done ? "Redo" : "Teach", done ? Ui.Style.GHOST : Ui.Style.SECONDARY);
        teach.setToolTipText(POINT_HELP[k]);
        teach.addActionListener(e -> actions.teach(area, k));
        JPanel b = new JPanel(new FlowLayout(FlowLayout.RIGHT, 0, 0));
        b.setOpaque(false);
        b.add(teach);
        p.add(b, BorderLayout.EAST);
        return p;
    }
}
