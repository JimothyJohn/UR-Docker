package com.nickarmenta.perceptronic;

import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * The Perceptronic Pick node's screen, in two views — no UR API, so a harness renders it too.
 *
 * <p><b>Main</b>: the live picture takes most of the screen, the parts numbered in pick order
 * on it. Beside it, all the operator does day to day: the picture points (as many as they
 * like — the program cycles through them looking for parts), the pick order (eight tiles;
 * tapping one renumbers the parts on the picture at once) and a one-line summary of the rest.
 *
 * <p><b>Options</b>: every default, grouped, each with a drawing that changes as it does —
 * the part and its size, the approach seen from the side, the gripper, the motion.
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
final class PickScreen extends JPanel {
    /** What the screen asks its node to do. */
    interface Actions {
        void addPoint();

        void goTo(int i);

        void retake(int i);

        void remove(int i);

        void select(int i);

        void cycleArea(int i);

        void setOrder(String first, String rows);

        void setNumber(String key, double value);

        void askNumber(String key, JLabel anchor);

        void setGripper(String mode);

        void setFlag(String key, boolean on);

        void checkApproach();

        void resetDefaults();
    }

    /** One picture point as the screen shows it. */
    static final class PointRow {
        final String area; // "Table A", or "table found live"
        final boolean areaTaught;

        PointRow(String area, boolean areaTaught) {
            this.area = area;
            this.areaTaught = areaTaught;
        }
    }

    static final String FLAG_POPUP = "popupOnFail";
    static final String FLAG_PER_POINT = "perPointRoutine";

    private final Actions actions;
    private final CardLayout cards = new CardLayout();
    private final JPanel deck = new JPanel(cards);
    final LiveView live = new LiveView();
    private final Ui.Note status = new Ui.Note();
    private final JPanel pointList = Ui.column();
    private final JLabel pointCount = Ui.label("", 12f, false, Ui.MUTED);
    private final List<Diagrams.OrderTile> tiles = new ArrayList<Diagrams.OrderTile>();
    private final JLabel orderWords = Ui.label("", 12f, false, Ui.MUTED);
    private final JLabel summaryPart = Ui.label("", 13f, true, Ui.INK);
    private final JLabel summaryApproach = Ui.label("", 12f, false, Ui.MUTED);
    private final JButton add = Ui.button("+  Add picture point here", Ui.Style.PRIMARY);
    private final Map<String, Ui.Stepper> steppers = new LinkedHashMap<String, Ui.Stepper>();
    private final Diagrams.PartDrawing partDrawing = new Diagrams.PartDrawing();
    private final Diagrams.ApproachDrawing approachDrawing = new Diagrams.ApproachDrawing();
    private final JPanel gripperFields = Ui.column();
    private Ui.Segmented gripper;
    private Ui.Switch popup;
    private Ui.Switch perPoint;
    private final Ui.Note optionsNote = new Ui.Note();

    PickScreen(Actions actions) {
        this.actions = actions;
        setOpaque(true);
        setBackground(Ui.BG);
        setLayout(new BorderLayout());
        deck.setOpaque(false);
        deck.add(main(), "main");
        deck.add(options(), "options");
        add(deck, BorderLayout.CENTER);
    }

    void showMain() {
        cards.show(deck, "main");
    }

    void showOptions() {
        cards.show(deck, "options");
    }

    // -- main ------------------------------------------------------------------------------

    private JComponent main() {
        JPanel p = new JPanel(new BorderLayout(10, 0));
        p.setOpaque(false);
        p.setBorder(BorderFactory.createEmptyBorder(8, 8, 8, 8));
        p.add(live, BorderLayout.CENTER);

        JPanel side = Ui.column();
        JPanel head = new JPanel(new BorderLayout());
        head.setOpaque(false);
        JPanel brand = Ui.row(8);
        brand.add(new Logo.Mark(26));
        brand.add(Ui.label("Perceptronic Pick", 20f, true, Ui.INK));
        head.add(brand, BorderLayout.WEST);
        head.add(Ui.label("v" + PickScript.VERSION, 12f, false, Ui.FAINT), BorderLayout.EAST);
        side.add(Ui.left(head));
        side.add(Box.createVerticalStrut(6));
        side.add(Ui.left(status));
        side.add(Box.createVerticalStrut(10));

        JPanel pts = new JPanel(new BorderLayout());
        pts.setOpaque(false);
        pts.add(Ui.label("Picture points", 15f, true, Ui.INK), BorderLayout.WEST);
        pts.add(pointCount, BorderLayout.EAST);
        side.add(Ui.left(pts));
        side.add(Ui.left(Ui.label("the program visits them in turn, looking for parts", 11.5f, false, Ui.MUTED)));
        side.add(Box.createVerticalStrut(4));
        side.add(Ui.left(pointList));
        side.add(Box.createVerticalStrut(6));
        add.addActionListener(e -> actions.addPoint());
        side.add(Ui.left(add));
        side.add(Box.createVerticalStrut(12));

        side.add(Ui.left(Ui.label("Pick order", 15f, true, Ui.INK)));
        side.add(Ui.left(orderWords));
        side.add(Box.createVerticalStrut(4));
        JPanel grid = new JPanel(new GridLayout(2, 4, 5, 5));
        grid.setOpaque(false);
        String[][] choices = {
            {"LR", "FB"}, {"RL", "FB"}, {"LR", "BF"}, {"RL", "BF"},
            {"FB", "LR"}, {"FB", "RL"}, {"BF", "LR"}, {"BF", "RL"},
        };
        for (final String[] c : choices) {
            Diagrams.OrderTile t = new Diagrams.OrderTile(c[0], c[1], () -> actions.setOrder(c[0], c[1]));
            tiles.add(t);
            grid.add(t);
        }
        grid.setMaximumSize(new Dimension(292, 108));
        side.add(Ui.left(grid));
        side.add(Box.createVerticalStrut(12));

        JPanel summary = Ui.card(null);
        JPanel sc = Ui.column();
        sc.add(Ui.left(summaryPart));
        sc.add(Ui.left(summaryApproach));
        summary.add(sc, BorderLayout.CENTER);
        summary.setMaximumSize(new Dimension(292, 64));
        side.add(Ui.left(summary));
        JPanel buttons = new JPanel(new GridLayout(1, 2, 6, 0));
        buttons.setOpaque(false);
        JButton opts = Ui.button("Options", Ui.Style.SECONDARY);
        opts.addActionListener(e -> showOptions());
        JButton check = Ui.button("Check approach", Ui.Style.SECONDARY);
        check.setToolTipText("PolyScope's move screen, over part #1 with the fingers open — hold to move");
        check.addActionListener(e -> actions.checkApproach());
        buttons.add(opts);
        buttons.add(check);
        // the list packs to the top at any screen height; the two buttons stay at the bottom
        JPanel sidebar = new JPanel(new BorderLayout(0, 8));
        sidebar.setOpaque(false);
        sidebar.setPreferredSize(new Dimension(300, 10));
        sidebar.add(side, BorderLayout.NORTH);
        sidebar.add(buttons, BorderLayout.SOUTH);
        p.add(sidebar, BorderLayout.EAST);
        return p;
    }

    /** One row of the picture-point list. */
    private JComponent pointRow(final int i, PointRow row, boolean selected) {
        JPanel r = new JPanel(new BorderLayout(8, 0)) {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = Ui.smooth(g);
                g2.setColor(getBackground());
                g2.fillRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 12, 12);
                g2.setColor(getForeground());
                g2.drawRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 12, 12);
                g2.dispose();
            }
        };
        r.setOpaque(false);
        r.setBackground(selected ? Ui.ACCENT_SOFT : Ui.CARD);
        r.setForeground(selected ? Ui.ACCENT : Ui.LINE);
        r.setBorder(BorderFactory.createEmptyBorder(4, 6, 4, 4));
        JComponent badge = new JComponent() {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(30, 30);
            }

            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = Ui.smooth(g);
                g2.setColor(selected ? Ui.ACCENT : new Color(0x2b3a4a));
                g2.fillOval(1, (getHeight() - 28) / 2, 28, 28);
                g2.setColor(Color.WHITE);
                g2.setFont(Ui.font(13f, true));
                Ui.centre(g2, String.valueOf(i + 1), 15, getHeight() / 2);
                g2.dispose();
            }
        };
        r.add(badge, BorderLayout.WEST);
        JPanel text = Ui.column();
        text.add(Ui.left(Ui.label("Picture " + (i + 1), 13.5f, true, Ui.INK)));
        JLabel area = Ui.label((row.areaTaught ? "▣ " : "◌ ") + row.area + "  ›", 11.5f, false,
                row.areaTaught ? Ui.ACCENT : Ui.MUTED);
        area.setToolTipText("tap: which taught pick area this picture looks at");
        area.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        area.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseReleased(MouseEvent e) {
                actions.cycleArea(i);
            }
        });
        text.add(Ui.left(area));
        r.add(text, BorderLayout.CENTER);
        JPanel b = new JPanel(new FlowLayout(FlowLayout.RIGHT, 0, 0));
        b.setOpaque(false);
        JButton go = small("Go", "move the arm there (hold to move)");
        go.addActionListener(e -> actions.goTo(i));
        JButton here = small("Here", "retake this picture point where the arm is now");
        here.addActionListener(e -> actions.retake(i));
        JButton del = small("✕", "remove this picture point");
        del.addActionListener(e -> actions.remove(i));
        b.add(go);
        b.add(here);
        b.add(del);
        r.add(b, BorderLayout.EAST);
        r.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseReleased(MouseEvent e) {
                actions.select(i);
            }
        });
        r.setMaximumSize(new Dimension(292, 50));
        r.setAlignmentX(Component.LEFT_ALIGNMENT);
        return r;
    }

    private static JButton small(String text, String tip) {
        JButton b = new Ui.Pill(text, Ui.Style.GHOST) {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(Math.max(34, getFontMetrics(getFont()).stringWidth(getText()) + 12), 40);
            }
        };
        b.setFont(Ui.font(12.5f, true));
        b.setToolTipText(tip);
        return b;
    }

    // -- options ---------------------------------------------------------------------------

    private JComponent options() {
        JPanel p = new JPanel(new BorderLayout(0, 8));
        p.setOpaque(false);
        p.setBorder(BorderFactory.createEmptyBorder(8, 10, 8, 10));
        JPanel head = new JPanel(new BorderLayout());
        head.setOpaque(false);
        JButton back = Ui.button("‹  Back to the picture", Ui.Style.SECONDARY);
        back.addActionListener(e -> showMain());
        head.add(back, BorderLayout.WEST);
        JLabel title = Ui.label("Options", 20f, true, Ui.INK);
        title.setHorizontalAlignment(JLabel.CENTER);
        head.add(title, BorderLayout.CENTER);
        JButton reset = Ui.button("Reset to defaults", Ui.Style.GHOST);
        reset.addActionListener(e -> actions.resetDefaults());
        head.add(reset, BorderLayout.EAST);
        p.add(head, BorderLayout.NORTH);

        JPanel grid = new JPanel(new GridLayout(2, 2, 10, 10));
        grid.setOpaque(false);
        grid.add(section("The part", "part", partDrawing));
        grid.add(section("Approach and grip", "approach", approachDrawing));
        grid.add(gripperSection());
        grid.add(motionSection());
        p.add(grid, BorderLayout.CENTER);
        p.add(optionsNote, BorderLayout.SOUTH);
        return p;
    }

    private JComponent section(String title, String section, JComponent drawing) {
        JPanel card = Ui.card(title);
        JPanel body = new JPanel(new BorderLayout(10, 0));
        body.setOpaque(false);
        JPanel fields = Ui.column();
        for (PickScript.Num n : PickScript.NUMBERS) {
            if (!n.section.equals(section)) continue;
            fields.add(stepper(n));
        }
        body.add(fields, BorderLayout.CENTER);
        if (drawing != null) body.add(drawing, BorderLayout.EAST);
        card.add(body, BorderLayout.CENTER);
        return card;
    }

    private Ui.Stepper stepper(final PickScript.Num n) {
        Ui.Stepper s = new Ui.Stepper(n.label, n.help, new Ui.Step() {
            @Override
            public void step(int direction) {
                Double now = current.get(n.key);
                actions.setNumber(n.key, (now == null ? n.def : now) + direction * n.step);
            }

            @Override
            public void type(JLabel anchor) {
                actions.askNumber(n.key, anchor);
            }
        });
        steppers.put(n.key, s);
        return s;
    }

    private final Map<String, Double> current = new LinkedHashMap<String, Double>();

    private JComponent gripperSection() {
        JPanel card = Ui.card("Gripper");
        JPanel body = Ui.column();
        gripper = new Ui.Segmented(new String[] {"Robotiq Hand-E", "Digital output", "My own nodes"}, 0,
                i -> actions.setGripper(PickScript.GRIPPERS[i]));
        body.add(Ui.left(gripper));
        body.add(Box.createVerticalStrut(6));
        body.add(Ui.left(gripperFields));
        for (PickScript.Num n : PickScript.NUMBERS) {
            if (n.section.equals("gripper")) stepper(n);
        }
        card.add(body, BorderLayout.CENTER);
        return card;
    }

    private JComponent motionSection() {
        JPanel card = Ui.card("Motion and behaviour");
        JPanel body = Ui.column();
        for (PickScript.Num n : PickScript.NUMBERS) {
            if (n.section.equals("motion")) body.add(stepper(n));
        }
        popup = new Ui.Switch("Popup when nothing is picked", "and the program waits", true,
                on -> actions.setFlag(FLAG_POPUP, on));
        perPoint = new Ui.Switch("A routine per picture point", "a place routine for each, as children",
                false, on -> actions.setFlag(FLAG_PER_POINT, on));
        body.add(popup);
        body.add(perPoint);
        card.add(body, BorderLayout.CENTER);
        return card;
    }

    // -- refresh from the node ---------------------------------------------------------------

    /** Show {@code s} (the node's settings), its picture points and which one is selected. */
    void show(final PickScript s, final List<PointRow> points, final int selected, final boolean perPointRoutine) {
        onEdt(() -> {
            pointList.removeAll();
            for (int i = 0; i < points.size(); i++) {
                pointList.add(pointRow(i, points.get(i), i == selected));
                pointList.add(Box.createVerticalStrut(4));
            }
            if (points.isEmpty()) {
                pointList.add(Ui.left(Ui.label("none yet — move the arm where the camera sees the parts", 12f, false,
                        Ui.MUTED)));
            }
            pointCount.setText(points.size() + " of " + PickScript.MAX_POINTS);
            add.setEnabled(points.size() < PickScript.MAX_POINTS);
            for (Diagrams.OrderTile t : tiles) t.setSelected(t.first.equals(s.orderFirst) && t.rows.equals(s.orderRows));
            orderWords.setText(PickScript.orderText(s.orderFirst, s.orderRows));
            summaryPart.setText("Part " + Ui.value(Math.max(s.n("partLengthMm"), s.n("partWidthMm")), "") + " × "
                    + Ui.value(Math.min(s.n("partLengthMm"), s.n("partWidthMm")), "") + " × "
                    + Ui.value(s.n("partHeightMm"), "mm") + "  ±" + Ui.value(s.n("partTolPct"), "%"));
            summaryApproach.setText("open, " + Ui.value(s.n("approachMm"), "mm") + " over the top · grip "
                    + Ui.value(s.n("gripBelowTopMm"), "mm"));
            for (PickScript.Num n : PickScript.NUMBERS) {
                current.put(n.key, s.n(n.key));
                Ui.Stepper st = steppers.get(n.key);
                if (st != null) st.setValue(Ui.value(s.n(n.key), n.unit));
            }
            partDrawing.set(s.n("partLengthMm"), s.n("partWidthMm"), s.n("partHeightMm"));
            approachDrawing.set(s.n("approachMm"), s.n("gripBelowTopMm"), s.n("liftMm"), s.n("partHeightMm"),
                    Math.min(s.n("partLengthMm"), s.n("partWidthMm")), s.n("strokeMm"));
            int gi = 0;
            for (int i = 0; i < PickScript.GRIPPERS.length; i++) {
                if (PickScript.GRIPPERS[i].equals(s.gripper)) gi = i;
            }
            gripper.setSelected(gi);
            gripperFields.removeAll();
            String[] keys = gi == 0 ? new String[] {"strokeMm", "gripperForcePct", "gripperSpeedPct"}
                    : gi == 1 ? new String[] {"strokeMm", "gripperDo", "gripperWaitS"} : new String[] {"strokeMm"};
            for (String k : keys) gripperFields.add(steppers.get(k));
            if (gi == 2) {
                gripperFields.add(Ui.left(Ui.label("put your gripper's Close nodes inside this node: they run at the"
                        + " grip", 11.5f, false, Ui.MUTED)));
            }
            popup.setOn(s.popupOnFail);
            perPoint.setOn(perPointRoutine);
            String problem = s.problem();
            optionsNote.set(problem == null ? "Every change shows on the picture at once: go back to see the parts"
                    + " renumbered and re-measured." : problem, problem == null ? Ui.Kind.INFO : Ui.Kind.WARN);
            revalidate();
            repaint();
        });
    }

    static String gripperWords(String g) {
        return "robotiq".equals(g) ? "Robotiq Hand-E" : "digital".equals(g) ? "digital-output gripper"
                : "your gripper nodes";
    }

    void setStatus(final String text, final Ui.Kind kind) {
        onEdt(() -> {
            status.set(text, kind);
            revalidate();
        });
    }

    void setFrame(BufferedImage image) {
        live.setFrame(image);
    }

    void setScene(Scene s) {
        live.setScene(s);
    }

    void setLive(boolean on, String fps) {
        live.setLive(on, fps);
    }

    static void onEdt(Runnable r) {
        if (SwingUtilities.isEventDispatchThread()) r.run();
        else SwingUtilities.invokeLater(r);
    }
}
