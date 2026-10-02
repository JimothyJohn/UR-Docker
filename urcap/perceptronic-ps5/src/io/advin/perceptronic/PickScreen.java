package io.advin.perceptronic;

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
 * The 3D Pick node's screen, in two views — no UR API, so a harness renders it too. Nothing
 * on it scrolls: every list has a fixed place and a fixed size (0.7.0, Nick 2026-09-30).
 *
 * <p><b>Main</b>: the live picture takes most of the screen. Beside it, all the operator does
 * day to day: the picture points (a fixed grid of numbered buttons — the program visits them
 * in turn — with the selected one's actions under it) and the pick order (eight tiles).
 *
 * <p><b>Options</b>: two tabs and nothing else — <i>Part</i> (box or cylinder, its size, the
 * tolerance) and <i>Approach</i> (how far over the top, how deep the grip, the grip check and
 * its finger room, which side of a box the fingers close across, the closer look). Speeds are
 * not this node's business, and neither is the gripper: the program opens it before the node
 * and closes it after.
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

        void setShape(String shape);

        void setFlag(String key, boolean on);

        void setDepthView(boolean on);

        void checkApproach();

        void resetDefaults();
    }

    /** One picture point as the screen shows it. */
    static final class PointRow {
        final String area; // "Table A", or "live table"
        final boolean areaTaught;

        PointRow(String area, boolean areaTaught) {
            this.area = area;
            this.areaTaught = areaTaught;
        }
    }

    static final String FLAG_GRIP_CHECK = "gripCheck";
    static final String FLAG_GRIP_LONG = "gripLongSide";
    static final String FLAG_CLOSE_LOOK = "closeLook";
    private static final int SIDE = 300;
    private static final int CHIP_COLS = 6;
    private static final int FIELDS = 460; // the options' column of fields: the − / + stay near their labels

    private final Actions actions;
    private final CardLayout cards = new CardLayout();
    private final JPanel deck = new JPanel(cards);
    final LiveView live = new LiveView();
    private final Ui.Note status = new Ui.Note();
    private final JPanel chips = new JPanel(new GridLayout(PickScript.MAX_POINTS / CHIP_COLS, CHIP_COLS, 5, 5));
    private final JPanel selectedRow = new JPanel(new BorderLayout(6, 0));
    private final JLabel pointCount = Ui.label("", 12f, false, Ui.MUTED);
    private final List<Diagrams.OrderTile> tiles = new ArrayList<Diagrams.OrderTile>();
    private final JLabel orderWords = Ui.label("", 12f, false, Ui.MUTED);
    private final JLabel summary = Ui.label("", 12.5f, true, Ui.INK);
    private final Map<String, Ui.Stepper> steppers = new LinkedHashMap<String, Ui.Stepper>();
    private final Map<String, Double> current = new LinkedHashMap<String, Double>();
    private final Diagrams.PartDrawing partDrawing = new Diagrams.PartDrawing();
    private final Diagrams.ApproachDrawing approachDrawing = new Diagrams.ApproachDrawing();
    private final CardLayout tabCards = new CardLayout();
    private final JPanel tabDeck = new JPanel(tabCards);
    private Ui.Segmented tabs;
    private Ui.Segmented shape;
    private Ui.Check gripCheck;
    private Ui.Check gripLong;
    private Ui.Check closeLook;
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
        live.setViewListener(on -> actions.setDepthView(on));
    }

    void showMain() {
        cards.show(deck, "main");
    }

    void showOptions() {
        cards.show(deck, "options");
    }

    /** The Options view on its Part (0) or Approach (1) tab. */
    void showOptions(int tab) {
        tabs.setSelected(tab);
        tabCards.show(tabDeck, tab == 0 ? "part" : "approach");
        showOptions();
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
        brand.add(Ui.label("3D Pick", 20f, true, Ui.INK));
        head.add(brand, BorderLayout.WEST);
        head.add(Ui.label("v" + PickScript.VERSION, 12f, false, Ui.FAINT), BorderLayout.EAST);
        head.setMaximumSize(new Dimension(SIDE - 8, 30));
        side.add(Ui.left(head));
        side.add(Box.createVerticalStrut(6));
        status.setMaximumSize(new Dimension(SIDE - 8, 52));
        side.add(Ui.left(status));
        side.add(Box.createVerticalStrut(10));

        JPanel pts = new JPanel(new BorderLayout());
        pts.setOpaque(false);
        pts.add(Ui.label("Picture points", 15f, true, Ui.INK), BorderLayout.WEST);
        pts.add(pointCount, BorderLayout.EAST);
        pts.setMaximumSize(new Dimension(SIDE - 8, 22));
        side.add(Ui.left(pts));
        side.add(Box.createVerticalStrut(4));
        chips.setOpaque(false);
        chips.setMaximumSize(new Dimension(SIDE - 8, 2 * Ui.TAP + 5));
        chips.setPreferredSize(new Dimension(SIDE - 8, 2 * Ui.TAP + 5));
        side.add(Ui.left(chips));
        side.add(Box.createVerticalStrut(5));
        selectedRow.setOpaque(false);
        selectedRow.setMaximumSize(new Dimension(SIDE - 8, Ui.TAP));
        selectedRow.setPreferredSize(new Dimension(SIDE - 8, Ui.TAP));
        side.add(Ui.left(selectedRow));
        side.add(Box.createVerticalStrut(10));

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
        grid.setMaximumSize(new Dimension(SIDE - 8, 108));
        side.add(Ui.left(grid));
        side.add(Box.createVerticalStrut(8));
        side.add(Ui.left(summary));

        JPanel buttons = new JPanel(new GridLayout(1, 2, 6, 0));
        buttons.setOpaque(false);
        JButton opts = Ui.button("Options", Ui.Style.SECONDARY);
        opts.addActionListener(e -> showOptions());
        JButton check = Ui.button("Check approach", Ui.Style.SECONDARY);
        check.setToolTipText("PolyScope's move screen, over the first part with the fingers open — hold to move");
        check.addActionListener(e -> actions.checkApproach());
        buttons.add(opts);
        buttons.add(check);
        // the column packs to the top at any screen height; the two buttons stay at the bottom
        JPanel sidebar = new JPanel(new BorderLayout(0, 8));
        sidebar.setOpaque(false);
        sidebar.setPreferredSize(new Dimension(SIDE, 10));
        sidebar.add(side, BorderLayout.NORTH);
        sidebar.add(buttons, BorderLayout.SOUTH);
        p.add(sidebar, BorderLayout.EAST);
        return p;
    }

    /** One numbered button of the picture-point grid; {@code i < 0} is the "+" that adds one. */
    private JComponent chip(final int i, final boolean selected) {
        JComponent c = new JComponent() {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = Ui.smooth(g);
                int w = getWidth() - 1, h = getHeight() - 1;
                g2.setColor(i < 0 ? Ui.ACCENT_SOFT : selected ? Ui.ACCENT : Ui.CARD);
                g2.fillRoundRect(0, 0, w, h, 12, 12);
                g2.setColor(i < 0 || selected ? Ui.ACCENT : Ui.LINE);
                g2.drawRoundRect(0, 0, w, h, 12, 12);
                g2.setFont(Ui.font(i < 0 ? 20f : 15f, true));
                g2.setColor(i < 0 ? Ui.ACCENT : selected ? Color.WHITE : Ui.INK);
                Ui.centre(g2, i < 0 ? "+" : String.valueOf(i + 1), getWidth() / 2, getHeight() / 2);
                g2.dispose();
            }
        };
        c.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        c.setToolTipText(i < 0 ? "add a picture point where the arm is now" : "picture point " + (i + 1));
        c.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseReleased(MouseEvent e) {
                if (i < 0) actions.addPoint();
                else actions.select(i);
            }
        });
        return c;
    }

    /** The selected picture point: which pick area it looks at, and Go / Here / remove. */
    private void fillSelectedRow(final int i, PointRow row) {
        selectedRow.removeAll();
        if (row == null) {
            selectedRow.add(Ui.label("tap + with the arm where the camera sees the parts", 12f, false, Ui.MUTED),
                    BorderLayout.CENTER);
            return;
        }
        JPanel text = Ui.column();
        text.add(Ui.left(Ui.label("Picture " + (i + 1), 13.5f, true, Ui.INK)));
        JLabel area = Ui.label((row.areaTaught ? "▣ " : "◌ ") + row.area + "  ›", 11.5f, false,
                row.areaTaught ? Ui.ACCENT : Ui.MUTED);
        area.setToolTipText("tap: which taught pick area this picture looks at");
        area.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        MouseAdapter cycle = new MouseAdapter() {
            @Override
            public void mouseReleased(MouseEvent e) {
                actions.cycleArea(i);
            }
        };
        area.addMouseListener(cycle);
        text.addMouseListener(cycle);
        text.add(Ui.left(area));
        selectedRow.add(text, BorderLayout.CENTER);
        JPanel b = new JPanel(new FlowLayout(FlowLayout.RIGHT, 2, 0));
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
        selectedRow.add(b, BorderLayout.EAST);
    }

    private static JButton small(String text, String tip) {
        JButton b = new Ui.Pill(text, Ui.Style.SECONDARY) {
            @Override
            public Dimension getPreferredSize() {
                return new Dimension(Math.max(Ui.TAP, getFontMetrics(getFont()).stringWidth(getText()) + 20), Ui.TAP);
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
        JPanel head = new JPanel(new BorderLayout(12, 0));
        head.setOpaque(false);
        JButton back = Ui.button("‹  Back to the picture", Ui.Style.SECONDARY);
        back.addActionListener(e -> showMain());
        head.add(back, BorderLayout.WEST);
        tabs = new Ui.Segmented(new String[] {"Part", "Approach"}, 0,
                i -> tabCards.show(tabDeck, i == 0 ? "part" : "approach"));
        JPanel mid = new JPanel(new FlowLayout(FlowLayout.CENTER, 0, 2));
        mid.setOpaque(false);
        tabs.setPreferredSize(new Dimension(300, 40));
        mid.add(tabs);
        head.add(mid, BorderLayout.CENTER);
        JButton reset = Ui.button("Reset to defaults", Ui.Style.GHOST);
        reset.addActionListener(e -> actions.resetDefaults());
        head.add(reset, BorderLayout.EAST);
        p.add(head, BorderLayout.NORTH);

        tabDeck.setOpaque(false);
        tabDeck.add(partTab(), "part");
        tabDeck.add(approachTab(), "approach");
        p.add(tabDeck, BorderLayout.CENTER);
        p.add(optionsNote, BorderLayout.SOUTH);
        return p;
    }

    private JComponent partTab() {
        JPanel fields = Ui.column();
        shape = new Ui.Segmented(new String[] {"Box", "Cylinder"}, 0, i -> actions.setShape(PickScript.SHAPES[i]));
        shape.setPreferredSize(new Dimension(260, 40));
        JPanel shapeRow = Ui.row(0);
        shapeRow.add(shape);
        shapeRow.setMaximumSize(new Dimension(Integer.MAX_VALUE, 40));
        fields.add(shapeRow);
        fields.add(Box.createVerticalStrut(8));
        addSteppers(fields, "part");
        return tab("The part, as it lies on the table", fields, partDrawing);
    }

    private JComponent approachTab() {
        JPanel fields = Ui.column();
        addSteppers(fields, "approach");
        fields.add(Box.createVerticalStrut(6));
        gripCheck = new Ui.Check("Grip check", "skip a part with less than the finger room on either side",
                true, on -> actions.setFlag(FLAG_GRIP_CHECK, on));
        fields.add(gripCheck);
        gripLong = new Ui.Check("Grip across the long side", "off: the fingers close across the short side",
                false, on -> actions.setFlag(FLAG_GRIP_LONG, on));
        fields.add(gripLong);
        closeLook = new Ui.Check("Closer look", "a second, nearer measurement before the approach",
                true, on -> actions.setFlag(FLAG_CLOSE_LOOK, on));
        fields.add(closeLook);
        return tab("Approach and grip", fields, approachDrawing);
    }

    private static JComponent tab(String title, JPanel fields, JComponent drawing) {
        JPanel card = Ui.card(title);
        JPanel body = new JPanel(new BorderLayout(16, 0));
        body.setOpaque(false);
        JPanel left = new JPanel(new BorderLayout());
        left.setOpaque(false);
        left.setPreferredSize(new Dimension(FIELDS, 10));
        left.add(fields, BorderLayout.NORTH); // packed to the top; nothing stretches
        body.add(left, BorderLayout.WEST);
        JPanel right = new JPanel(new BorderLayout());
        right.setOpaque(false);
        right.add(drawing, BorderLayout.NORTH); // beside the fields it explains, not at the bottom
        body.add(right, BorderLayout.CENTER);
        card.add(body, BorderLayout.CENTER);
        return card;
    }

    private void addSteppers(JPanel into, String section) {
        for (final PickScript.Num n : PickScript.NUMBERS) {
            if (!n.section.equals(section)) continue;
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
            into.add(s);
        }
    }

    // -- refresh from the node ---------------------------------------------------------------

    /** Show {@code s} (the node's settings), its picture points and which one is selected. */
    void show(final PickScript s, final List<PointRow> points, final int selected) {
        onEdt(() -> {
            chips.removeAll();
            for (int i = 0; i < points.size(); i++) chips.add(chip(i, i == selected));
            if (points.size() < PickScript.MAX_POINTS) chips.add(chip(-1, false));
            for (int i = points.size() + 1; i < PickScript.MAX_POINTS; i++) chips.add(Box.createGlue());
            fillSelectedRow(selected, selected >= 0 && selected < points.size() ? points.get(selected) : null);
            pointCount.setText(points.size() + " of " + PickScript.MAX_POINTS);
            for (Diagrams.OrderTile t : tiles) t.setSelected(t.first.equals(s.orderFirst) && t.rows.equals(s.orderRows));
            orderWords.setText(PickScript.orderText(s.orderFirst, s.orderRows));
            summary.setText(partWords(s));
            for (PickScript.Num n : PickScript.NUMBERS) {
                current.put(n.key, s.n(n.key));
                Ui.Stepper st = steppers.get(n.key);
                if (st != null) st.setValue(Ui.value(s.n(n.key), n.unit));
            }
            boolean round = s.round();
            shape.setSelected(round ? 1 : 0);
            steppers.get("partLengthMm").setLabel(round ? "Diameter" : "Length",
                    round ? "across the top, standing on its end" : "long side, as it lies");
            steppers.get("partWidthMm").setVisible(!round);
            partDrawing.set(s.longSide(), s.shortSide(), s.n("partHeightMm"), round);
            boolean longWay = s.gripLongSide && !round;
            approachDrawing.set(s.n("approachMm"), s.n("gripBelowTopMm"), s.n("partHeightMm"),
                    longWay ? s.longSide() : s.shortSide(), s.gripCheck ? s.n("fingerRoomMm") : 0);
            gripCheck.setOn(s.gripCheck);
            gripLong.setOn(s.gripLongSide);
            gripLong.setVisible(!round); // a cylinder has no side to choose
            steppers.get("fingerRoomMm").setVisible(s.gripCheck);
            closeLook.setOn(s.closeLook);
            String problem = s.problem();
            optionsNote.set(problem == null ? "Every change is used by the picture at once: go back to see what is"
                    + " found." : problem, problem == null ? Ui.Kind.INFO : Ui.Kind.WARN);
            revalidate();
            repaint();
        });
    }

    /** {@code 50 × 30 × 30 mm ±25 %} / {@code cylinder Ø40 × 30 mm ±25 %}. */
    static String partWords(PickScript s) {
        String size = s.round() ? "cylinder Ø" + Ui.value(s.longSide(), "")
                : Ui.value(s.longSide(), "") + " × " + Ui.value(s.shortSide(), "");
        return size + " × " + Ui.value(s.n("partHeightMm"), "mm") + "  ±" + Ui.value(s.n("partTolPct"), "%");
    }

    /** One short line beside the picture; the long story of a lost camera is on the picture itself. */
    void setStatus(final String text, final Ui.Kind kind) {
        onEdt(() -> {
            String t = text == null ? "" : text;
            int nl = t.indexOf('\n');
            status.set(nl < 0 ? t : t.substring(0, nl), kind);
            revalidate();
        });
    }

    void setFrame(BufferedImage image) {
        live.setFrame(image);
    }

    void setScene(Scene s) {
        live.setScene(s);
    }

    /** {@code why}: with no picture, what to check (shown on the picture's place); ignored when live. */
    void setLive(boolean on, String why) {
        if (!on && why != null) live.setEmptyText(why);
        live.setLive(on);
    }

    static void onEdt(Runnable r) {
        if (SwingUtilities.isEventDispatchThread()) r.run();
        else SwingUtilities.invokeLater(r);
    }
}
