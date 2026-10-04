package io.advin.perceptronic;

import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
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
 * The 3D Pick node's screen — no UR API, so a harness renders it too. Nothing on it scrolls
 * (0.7.0, Nick 2026-09-30).
 *
 * <p><b>Main</b> (0.9.0, Nick 2026-10-02: at most three simple stages, tap to teach, fewer
 * screens): the live picture takes most of the screen; beside it a step rail — <i>1 Look</i>,
 * <i>2 Part</i>, <i>3 Grip</i> — and the one step being done:
 * <ol>
 *   <li><b>Look</b>: put the arm where the camera sees the parts, tap <i>Use this view</i>.
 *       More views are optional (a small row of numbered buttons).
 *   <li><b>Part</b>: tap one part in the picture; the node measures it and every part like it
 *       turns green with its number.
 *   <li><b>Grip</b>: how deep the fingertips go, and <i>Check approach</i> on the first part.
 * </ol>
 * The pick order is not on it: the camera computer clears the way (a part pinned by others
 * goes after them), and the tie-break is one control under Options.
 *
 * <p><b>Options</b>: two tabs — <i>Part</i> (box or cylinder, its size by hand, the
 * tolerance) and <i>Approach</i> (approach, grip depth, the grip check and its finger room,
 * the side a box is gripped across, the closer look, the order when it does not matter).
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

        /** The operator tapped the picture on the Part step, at picture pixel ({@code u}, {@code v}). */
        void teachAt(int u, int v);
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
    static final String[] STEPS = {"Look", "Part", "Grip"};
    static final int LOOK = 0;
    static final int PART = 1;
    static final int GRIP = 2;
    static final String[][] ORDERS = {
        {"LR", "FB"}, {"RL", "FB"}, {"LR", "BF"}, {"RL", "BF"},
        {"FB", "LR"}, {"FB", "RL"}, {"BF", "LR"}, {"BF", "RL"},
    };
    private static final int SIDE = 300;
    private static final int CHIP_COLS = 6;
    private static final int FIELDS = 460; // the options' column of fields: the − / + stay near their labels
    private static final String TAP_HINT = "Tap one part";

    private final Actions actions;
    private final CardLayout cards = new CardLayout();
    private final JPanel deck = new JPanel(cards);
    final LiveView live = new LiveView();
    private final Ui.Note status = new Ui.Note();
    private final StepRail rail = new StepRail();
    private final CardLayout stepCards = new CardLayout();
    private final JPanel stepDeck = new JPanel(stepCards);
    private int step = LOOK;
    private boolean stepChosen; // a step is on screen by choice: refreshes leave it where it is
    private final JButton next = Ui.button("Next  ›", Ui.Style.PRIMARY);

    // Look
    private final JButton useView = Ui.button("Use this view", Ui.Style.PRIMARY);
    private final JPanel chips = new JPanel(new GridLayout(PickScript.MAX_POINTS / CHIP_COLS, CHIP_COLS, 4, 4));
    private final JPanel selectedRow = new JPanel(new BorderLayout(6, 0));
    // Part
    private final JLabel partSize = Ui.label("", 22f, true, Ui.INK);
    private final JLabel partHow = Ui.label("", 12f, false, Ui.MUTED);
    // Grip
    private final Diagrams.ApproachDrawing gripDrawing = new Diagrams.ApproachDrawing();

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
    private final JButton order = Ui.button("", Ui.Style.SECONDARY);
    private String[] orderNow = ORDERS[0];
    private final Ui.Note optionsNote = new Ui.Note();
    private boolean[] done = new boolean[3];

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
        live.setTapListener((u, v) -> {
            if (step == PART) actions.teachAt(u, v);
        });
        showStep(LOOK);
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

    /** Show step {@link #LOOK}, {@link #PART} or {@link #GRIP}. */
    void showStep(int s) {
        step = Math.max(LOOK, Math.min(GRIP, s));
        stepCards.show(stepDeck, STEPS[step]);
        rail.repaint();
        live.setHint(step == PART ? TAP_HINT : null);
        next.setText(step == GRIP ? "Done" : "Next  ›");
        showMain();
    }

    int step() {
        return step;
    }

    /** The node's view is opening again: the next {@link #show} opens on the first step not done. */
    void reopen() {
        stepChosen = false;
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
        side.add(Box.createVerticalStrut(8));
        rail.setMaximumSize(new Dimension(SIDE - 8, 46));
        rail.setPreferredSize(new Dimension(SIDE - 8, 46));
        side.add(Ui.left(rail));
        side.add(Box.createVerticalStrut(8));

        stepDeck.setOpaque(false);
        stepDeck.add(lookStep(), STEPS[LOOK]);
        stepDeck.add(partStep(), STEPS[PART]);
        stepDeck.add(gripStep(), STEPS[GRIP]);
        stepDeck.setAlignmentX(LEFT_ALIGNMENT);
        side.add(stepDeck);

        JPanel buttons = new JPanel(new GridLayout(1, 2, 6, 0));
        buttons.setOpaque(false);
        JButton opts = Ui.button("Options", Ui.Style.SECONDARY);
        opts.addActionListener(e -> showOptions());
        next.addActionListener(e -> {
            if (step < GRIP) showStep(step + 1);
        });
        buttons.add(opts);
        buttons.add(next);
        // the column packs to the top at any screen height; the two buttons stay at the bottom
        JPanel sidebar = new JPanel(new BorderLayout(0, 8));
        sidebar.setOpaque(false);
        sidebar.setPreferredSize(new Dimension(SIDE, 10));
        sidebar.add(side, BorderLayout.NORTH);
        sidebar.add(buttons, BorderLayout.SOUTH);
        p.add(sidebar, BorderLayout.EAST);
        return p;
    }

    private static JPanel stepCard(String title, String line) {
        JPanel c = Ui.column();
        c.add(Ui.left(Ui.label(title, 17f, true, Ui.INK)));
        c.add(Box.createVerticalStrut(2));
        c.add(Ui.left(Ui.label(line, 12.5f, false, Ui.MUTED)));
        c.add(Box.createVerticalStrut(10));
        return c;
    }

    private JComponent lookStep() {
        JPanel c = stepCard("Look", "Put the arm where the camera sees the parts");
        useView.setToolTipText("PolyScope's move screen: put the arm there, then OK — 0.3 m or more above the parts");
        useView.addActionListener(e -> actions.addPoint());
        JPanel one = new JPanel(new BorderLayout());
        one.setOpaque(false);
        one.add(useView, BorderLayout.CENTER);
        one.setMaximumSize(new Dimension(SIDE - 8, Ui.TAP));
        c.add(Ui.left(one));
        c.add(Box.createVerticalStrut(12));
        c.add(Ui.left(Ui.label("Views", 13f, true, Ui.INK)));
        c.add(Box.createVerticalStrut(4));
        chips.setOpaque(false);
        chips.setMaximumSize(new Dimension(SIDE - 8, 2 * 36 + 4));
        chips.setPreferredSize(new Dimension(SIDE - 8, 2 * 36 + 4));
        c.add(Ui.left(chips));
        c.add(Box.createVerticalStrut(6));
        selectedRow.setOpaque(false);
        selectedRow.setMaximumSize(new Dimension(SIDE - 8, Ui.TAP));
        selectedRow.setPreferredSize(new Dimension(SIDE - 8, Ui.TAP));
        c.add(Ui.left(selectedRow));
        return c;
    }

    private JComponent partStep() {
        JPanel c = stepCard("Part", "Tap one part in the picture");
        c.add(Ui.left(partSize));
        c.add(Box.createVerticalStrut(2));
        c.add(Ui.left(partHow));
        c.add(Box.createVerticalStrut(10));
        c.add(Ui.left(Ui.label("Green: picked, in that order", 12.5f, false, Ui.OK)));
        c.add(Ui.left(Ui.label("Yellow: nearly, and why not", 12.5f, false, Ui.WARN)));
        c.add(Box.createVerticalStrut(6));
        c.add(Ui.left(Ui.label("A cylinder, or a size by hand: Options", 11.5f, false, Ui.FAINT)));
        return c;
    }

    private JComponent gripStep() {
        JPanel c = stepCard("Grip", "How deep the fingertips go, then check it");
        JPanel drawing = new JPanel(new BorderLayout());
        drawing.setOpaque(false);
        drawing.add(gripDrawing, BorderLayout.CENTER);
        drawing.setPreferredSize(new Dimension(SIDE - 8, 132));
        drawing.setMaximumSize(new Dimension(SIDE - 8, 132));
        c.add(Ui.left(drawing));
        c.add(Box.createVerticalStrut(4));
        PickScript.Num grip = PickScript.BY_KEY.get("gripBelowTopMm");
        Ui.Stepper s = stepper(grip, "below the part's top");
        s.setMaximumSize(new Dimension(SIDE - 8, Ui.TAP + 6));
        c.add(s);
        steppers.put("grip:" + grip.key, s);
        c.add(Box.createVerticalStrut(8));
        JButton check = Ui.button("Check approach", Ui.Style.SECONDARY);
        check.setToolTipText("PolyScope's move screen, over the first part with the fingers open — hold to move");
        check.addActionListener(e -> actions.checkApproach());
        JPanel one = new JPanel(new BorderLayout());
        one.setOpaque(false);
        one.add(check, BorderLayout.CENTER);
        one.setMaximumSize(new Dimension(SIDE - 8, Ui.TAP));
        c.add(Ui.left(one));
        return c;
    }

    /** One numbered button of the views grid; {@code i < 0} is the "+" that adds one. */
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
                g2.setFont(Ui.font(i < 0 ? 18f : 14f, true));
                g2.setColor(i < 0 ? Ui.ACCENT : selected ? Color.WHITE : Ui.INK);
                Ui.centre(g2, i < 0 ? "+" : String.valueOf(i + 1), getWidth() / 2, getHeight() / 2);
                g2.dispose();
            }
        };
        c.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        c.setToolTipText(i < 0 ? "another view, where the arm is now" : "view " + (i + 1));
        c.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseReleased(MouseEvent e) {
                if (i < 0) actions.addPoint();
                else actions.select(i);
            }
        });
        return c;
    }

    /** The selected view: which pick area it looks at (taught ones only), and Go / Here / remove. */
    private void fillSelectedRow(final int i, PointRow row) {
        selectedRow.removeAll();
        if (row == null) {
            selectedRow.add(Ui.label("no view yet", 12f, false, Ui.MUTED), BorderLayout.CENTER);
            return;
        }
        JPanel text = Ui.column();
        text.add(Ui.left(Ui.label("View " + (i + 1), 13.5f, true, Ui.INK)));
        // pick areas are an Installation extra (Nick, 2026-10-02: fewer screens): the table is
        // found live, and a taught area is named here only when one is in use
        JLabel area = Ui.label(row.areaTaught ? "▣ " + row.area + "  ›" : "table found live", 11.5f, false,
                row.areaTaught ? Ui.ACCENT : Ui.FAINT);
        area.setToolTipText("tap: a pick area taught in the Installation, or the table found live");
        area.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
        area.addMouseListener(new MouseAdapter() {
            @Override
            public void mouseReleased(MouseEvent e) {
                actions.cycleArea(i);
            }
        });
        text.add(Ui.left(area));
        selectedRow.add(text, BorderLayout.CENTER);
        JPanel b = new JPanel(new FlowLayout(FlowLayout.RIGHT, 2, 0));
        b.setOpaque(false);
        JButton go = small("Go", "move the arm there (hold to move)");
        go.addActionListener(e -> actions.goTo(i));
        JButton here = small("Here", "retake this view where the arm is now");
        here.addActionListener(e -> actions.retake(i));
        JButton del = small("✕", "remove this view");
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

    /** The three steps, numbered; a tick on each one that is done; tap one to go to it. */
    private final class StepRail extends JComponent {
        StepRail() {
            setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseReleased(MouseEvent e) {
                    stepChosen = true;
                    showStep(Math.min(GRIP, e.getX() * STEPS.length / Math.max(1, getWidth())));
                }
            });
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = Ui.smooth(g);
            int n = STEPS.length, gap = 6, w = (getWidth() - gap * (n - 1)) / n, h = getHeight() - 1;
            for (int i = 0; i < n; i++) {
                int x = i * (w + gap);
                boolean on = i == step;
                g2.setColor(on ? Ui.ACCENT : done[i] ? Ui.OK_SOFT : Ui.CARD);
                g2.fillRoundRect(x, 0, w, h, 14, 14);
                g2.setColor(on ? Ui.ACCENT : done[i] ? Ui.OK : Ui.LINE);
                g2.drawRoundRect(x, 0, w, h, 14, 14);
                g2.setFont(Ui.font(14f, true));
                g2.setColor(on ? Color.WHITE : done[i] ? Ui.OK : Ui.INK);
                Ui.centre(g2, (done[i] && !on ? "✓ " : (i + 1) + "  ") + STEPS[i], x + w / 2, h / 2);
            }
            g2.dispose();
        }
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
        fields.add(Box.createVerticalStrut(4));
        // the order: parts that free others go first anyway; this only breaks ties (Nick, 2026-10-02)
        JPanel orderRow = new JPanel(new BorderLayout(10, 0));
        orderRow.setOpaque(false);
        JPanel words = Ui.column();
        words.add(Ui.left(Ui.label("Order", 14f, true, Ui.INK)));
        words.add(Ui.left(Ui.label("when it doesn't matter; tap to change", 11.5f, false, Ui.MUTED)));
        orderRow.add(words, BorderLayout.CENTER);
        order.addActionListener(e -> {
            int k = 0;
            for (int i = 0; i < ORDERS.length; i++) {
                if (ORDERS[i][0].equals(orderNow[0]) && ORDERS[i][1].equals(orderNow[1])) k = i;
            }
            String[] o = ORDERS[(k + 1) % ORDERS.length];
            actions.setOrder(o[0], o[1]);
        });
        orderRow.add(order, BorderLayout.EAST);
        orderRow.setAlignmentX(LEFT_ALIGNMENT);
        orderRow.setMaximumSize(new Dimension(Integer.MAX_VALUE, Ui.TAP + 4));
        fields.add(orderRow);
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

    private Ui.Stepper stepper(final PickScript.Num n, String help) {
        return new Ui.Stepper(n.label, help, new Ui.Step() {
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
    }

    private void addSteppers(JPanel into, String section) {
        for (final PickScript.Num n : PickScript.NUMBERS) {
            if (!n.section.equals(section)) continue;
            Ui.Stepper s = stepper(n, n.help);
            steppers.put(n.key, s);
            into.add(s);
        }
    }

    // -- refresh from the node ---------------------------------------------------------------

    /** Show {@code s} (the node's settings), its views and which one is selected. */
    void show(final PickScript s, final List<PointRow> points, final int selected) {
        show(s, points, selected, false);
    }

    /**
     * As {@link #show(PickScript, List, int)}; {@code partTaught}: the part's size came from a tap
     * (or was set by hand). Until the operator picks a step, the screen opens on the first one
     * not done.
     */
    void show(final PickScript s, final List<PointRow> points, final int selected, final boolean partTaught) {
        onEdt(() -> {
            chips.removeAll();
            for (int i = 0; i < points.size(); i++) chips.add(chip(i, i == selected));
            if (points.size() < PickScript.MAX_POINTS) chips.add(chip(-1, false));
            for (int i = points.size() + 1; i < PickScript.MAX_POINTS; i++) chips.add(Box.createGlue());
            fillSelectedRow(selected, selected >= 0 && selected < points.size() ? points.get(selected) : null);
            useView.setText(points.isEmpty() ? "Use this view" : "Add a view");
            done = new boolean[] {!points.isEmpty(), partTaught, false};
            if (!stepChosen) { // opening: the first step not done
                showStep(points.isEmpty() ? LOOK : !partTaught ? PART : GRIP);
                stepChosen = true;
            }
            boolean round = s.round();
            partSize.setText(round ? "Ø" + Ui.value(s.longSide(), "") + " × " + Ui.value(s.n("partHeightMm"), "mm")
                    : Ui.value(s.longSide(), "") + " × " + Ui.value(s.shortSide(), "") + " × "
                    + Ui.value(s.n("partHeightMm"), "mm"));
            partHow.setText((round ? "cylinder" : "box") + ", ±" + Ui.value(s.n("partTolPct"), "%")
                    + (partTaught ? " · measured from the picture" : " · not taught yet: tap a part"));
            orderNow = new String[] {s.orderFirst, s.orderRows};
            order.setText(orderShort(s.orderFirst, s.orderRows) + "  ›");
            for (PickScript.Num n : PickScript.NUMBERS) {
                current.put(n.key, s.n(n.key));
                Ui.Stepper st = steppers.get(n.key);
                if (st != null) st.setValue(Ui.value(s.n(n.key), n.unit));
                Ui.Stepper again = steppers.get("grip:" + n.key);
                if (again != null) again.setValue(Ui.value(s.n(n.key), n.unit));
            }
            shape.setSelected(round ? 1 : 0);
            steppers.get("partLengthMm").setLabel(round ? "Diameter" : "Length",
                    round ? "across the top, standing on its end" : "long side, as it lies");
            steppers.get("partWidthMm").setVisible(!round);
            partDrawing.set(s.longSide(), s.shortSide(), s.n("partHeightMm"), round);
            boolean longWay = s.gripLongSide && !round;
            double across = longWay ? s.longSide() : s.shortSide();
            double room = s.gripCheck ? s.n("fingerRoomMm") : 0;
            approachDrawing.set(s.n("approachMm"), s.n("gripBelowTopMm"), s.n("partHeightMm"), across, room);
            gripDrawing.set(s.n("approachMm"), s.n("gripBelowTopMm"), s.n("partHeightMm"), across, room);
            gripCheck.setOn(s.gripCheck);
            gripLong.setOn(s.gripLongSide);
            gripLong.setVisible(!round); // a cylinder has no side to choose
            steppers.get("fingerRoomMm").setVisible(s.gripCheck);
            closeLook.setOn(s.closeLook);
            String problem = s.problem();
            optionsNote.set(problem == null ? "Every change is used by the picture at once: go back to see what is"
                    + " found." : problem, problem == null ? Ui.Kind.INFO : Ui.Kind.WARN);
            rail.repaint();
            revalidate();
            repaint();
        });
    }

    /** The order on one button: {@code left to right, front first}. */
    static String orderShort(String first, String rows) {
        String lead = "FB".equals(rows) ? "front" : "BF".equals(rows) ? "back" : "LR".equals(rows) ? "left" : "right";
        return PickScript.words(first) + ", " + lead + " first";
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
