package com.nickarmenta.perceptronic;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.RenderingHints;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.util.ArrayList;
import java.util.List;
import javax.swing.BorderFactory;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;

/**
 * The URCap's look: one palette, touch-sized controls (PolyScope 5's pendant is a 12"
 * touchscreen — nothing smaller than 44 px to tap), painted by hand so PolyScope's own
 * look-and-feel can't restyle them. No UR API: the screens built from these render in a
 * test harness too.
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
final class Ui {
    private Ui() {
    }

    static final Color BG = new Color(0xf3f5f8);
    static final Color CARD = Color.WHITE;
    static final Color INK = new Color(0x15202b);
    static final Color MUTED = new Color(0x5d6b7b);
    static final Color FAINT = new Color(0x98a4b2);
    static final Color LINE = new Color(0xdbe1e8);
    static final Color ACCENT = new Color(0x1c64d8);
    static final Color ACCENT_SOFT = new Color(0xe3edfd);
    static final Color OK = new Color(0x1a9a5c);
    static final Color OK_SOFT = new Color(0xe2f5eb);
    static final Color WARN = new Color(0xc98300);
    static final Color WARN_SOFT = new Color(0xfff3d6);
    static final Color ERR = new Color(0xd23f3f);
    static final Color ERR_SOFT = new Color(0xfde3e3);
    static final Color STAGE = new Color(0x0d131a);
    static final Color PART = new Color(0x3ddc84);
    static final Color JAW = new Color(0xffc53d);
    static final int TAP = 44;
    static final int RADIUS = 14;

    enum Kind { INFO, OK, WARN, ERR }

    static Color fg(Kind k) {
        return k == Kind.OK ? OK : k == Kind.WARN ? WARN : k == Kind.ERR ? ERR : ACCENT;
    }

    static Color bg(Kind k) {
        return k == Kind.OK ? OK_SOFT : k == Kind.WARN ? WARN_SOFT : k == Kind.ERR ? ERR_SOFT : ACCENT_SOFT;
    }

    static Font font(float size, boolean bold) {
        Font base = new JLabel().getFont();
        if (base == null) base = new Font(Font.SANS_SERIF, Font.PLAIN, 14);
        return base.deriveFont(bold ? Font.BOLD : Font.PLAIN, size);
    }

    static Graphics2D smooth(Graphics g) {
        Graphics2D g2 = (Graphics2D) g.create();
        g2.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_TEXT_ANTIALIASING, RenderingHints.VALUE_TEXT_ANTIALIAS_ON);
        g2.setRenderingHint(RenderingHints.KEY_STROKE_CONTROL, RenderingHints.VALUE_STROKE_PURE);
        return g2;
    }

    static JLabel label(String text, float size, boolean bold, Color color) {
        JLabel l = new JLabel(text);
        l.setFont(font(size, bold));
        l.setForeground(color);
        return l;
    }

    /** A white rounded card with a hairline edge; {@code title} (may be null) as its header. */
    static JPanel card(String title) {
        JPanel p = new JPanel() {
            @Override
            protected void paintComponent(Graphics g) {
                Graphics2D g2 = smooth(g);
                g2.setColor(CARD);
                g2.fillRoundRect(0, 0, getWidth() - 1, getHeight() - 1, RADIUS, RADIUS);
                g2.setColor(LINE);
                g2.drawRoundRect(0, 0, getWidth() - 1, getHeight() - 1, RADIUS, RADIUS);
                g2.dispose();
            }
        };
        p.setOpaque(false);
        p.setLayout(new BorderLayout(0, 6));
        p.setBorder(BorderFactory.createEmptyBorder(10, 12, 10, 12));
        if (title != null) p.add(label(title, 15f, true, INK), BorderLayout.NORTH);
        return p;
    }

    static JPanel column() {
        JPanel p = new JPanel();
        p.setOpaque(false);
        p.setLayout(new BoxLayout(p, BoxLayout.Y_AXIS));
        return p;
    }

    static JPanel row(int gap) {
        JPanel p = new JPanel(new FlowLayout(FlowLayout.LEFT, gap, 0));
        p.setOpaque(false);
        p.setAlignmentX(Component.LEFT_ALIGNMENT);
        return p;
    }

    static JComponent left(JComponent c) {
        c.setAlignmentX(Component.LEFT_ALIGNMENT);
        return c;
    }

    // -- buttons ---------------------------------------------------------------------------

    enum Style { PRIMARY, SECONDARY, GHOST, DANGER }

    /** A rounded, hand-painted button at least {@link #TAP} tall. */
    static JButton button(String text, Style style) {
        return new Pill(text, style);
    }

    static class Pill extends JButton {
        private final Style style;
        private boolean down;

        Pill(String text, Style style) {
            super(text);
            this.style = style;
            setContentAreaFilled(false);
            setBorderPainted(false);
            setFocusPainted(false);
            setOpaque(false);
            setFont(font(14f, true));
            setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            setMargin(new Insets(0, 14, 0, 14));
            addMouseListener(new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent e) {
                    down = true;
                    repaint();
                }

                @Override
                public void mouseReleased(MouseEvent e) {
                    down = false;
                    repaint();
                }
            });
        }

        @Override
        public Dimension getPreferredSize() {
            FontMetrics fm = getFontMetrics(getFont());
            return new Dimension(fm.stringWidth(getText()) + 32, TAP);
        }

        @Override
        public Dimension getMaximumSize() {
            return getPreferredSize();
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = smooth(g);
            int w = getWidth(), h = getHeight();
            boolean on = isEnabled();
            Color fill, text, edge;
            switch (style) {
                case PRIMARY:
                    fill = on ? (down ? ACCENT.darker() : ACCENT) : LINE;
                    text = Color.WHITE;
                    edge = null;
                    break;
                case DANGER:
                    fill = on ? (down ? ERR.darker() : ERR) : LINE;
                    text = Color.WHITE;
                    edge = null;
                    break;
                case GHOST:
                    fill = down ? ACCENT_SOFT : new Color(0, 0, 0, 0);
                    text = on ? ACCENT : FAINT;
                    edge = null;
                    break;
                default:
                    fill = down ? ACCENT_SOFT : CARD;
                    text = on ? INK : FAINT;
                    edge = LINE;
            }
            g2.setColor(fill);
            g2.fillRoundRect(0, 0, w - 1, h - 1, h, h);
            if (edge != null) {
                g2.setColor(edge);
                g2.drawRoundRect(0, 0, w - 1, h - 1, h, h);
            }
            g2.setFont(getFont());
            g2.setColor(text);
            FontMetrics fm = g2.getFontMetrics();
            String t = getText();
            g2.drawString(t, (w - fm.stringWidth(t)) / 2, (h + fm.getAscent() - fm.getDescent()) / 2);
            g2.dispose();
        }
    }

    // -- a segmented control ------------------------------------------------------------------

    interface Pick {
        void picked(int index);
    }

    /** One-of-N, pill-shaped; the selected segment filled. */
    static final class Segmented extends JComponent {
        private final String[] labels;
        private int selected;
        private final Pick onPick;

        Segmented(String[] labels, int selected, Pick onPick) {
            this.labels = labels;
            this.selected = selected;
            this.onPick = onPick;
            setFont(font(13f, true));
            setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            addMouseListener(new MouseAdapter() {
                @Override
                public void mouseReleased(MouseEvent e) {
                    int i = Math.max(0, Math.min(Segmented.this.labels.length - 1,
                            e.getX() * Segmented.this.labels.length / Math.max(1, getWidth())));
                    if (i != Segmented.this.selected) {
                        Segmented.this.selected = i;
                        repaint();
                        Segmented.this.onPick.picked(i);
                    }
                }
            });
        }

        void setSelected(int i) {
            selected = i;
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            FontMetrics fm = getFontMetrics(getFont());
            int w = 0;
            for (String l : labels) w = Math.max(w, fm.stringWidth(l));
            return new Dimension((w + 28) * labels.length, 40);
        }

        @Override
        public Dimension getMaximumSize() {
            return new Dimension(Integer.MAX_VALUE, 40);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = smooth(g);
            int w = getWidth(), h = getHeight(), n = labels.length;
            g2.setColor(BG);
            g2.fillRoundRect(0, 0, w - 1, h - 1, h, h);
            g2.setColor(LINE);
            g2.drawRoundRect(0, 0, w - 1, h - 1, h, h);
            g2.setFont(getFont());
            FontMetrics fm = g2.getFontMetrics();
            for (int i = 0; i < n; i++) {
                int x0 = i * w / n, x1 = (i + 1) * w / n;
                if (i == selected) {
                    g2.setColor(ACCENT);
                    g2.fillRoundRect(x0 + 3, 3, x1 - x0 - 6, h - 7, h - 6, h - 6);
                }
                g2.setColor(i == selected ? Color.WHITE : INK);
                String t = labels[i];
                g2.drawString(t, x0 + (x1 - x0 - fm.stringWidth(t)) / 2, (h + fm.getAscent() - fm.getDescent()) / 2);
            }
            g2.dispose();
        }
    }

    // -- a stepper --------------------------------------------------------------------------

    interface Step {
        void step(int direction);

        void type(JLabel anchor);
    }

    /**
     * {@code label ........ [ − ]  50 mm  [ + ]} — tap − / + to step, tap the value to type it
     * on PolyScope's keypad. {@link #setValue} refreshes it from the model.
     */
    static final class Stepper extends JPanel {
        private final JLabel value = new JLabel("", SwingConstants.CENTER);
        private final JLabel help;

        Stepper(String label, String helpText, final Step step) {
            setOpaque(false);
            setLayout(new BorderLayout(8, 0));
            setAlignmentX(Component.LEFT_ALIGNMENT);
            JPanel text = column();
            text.add(left(label(label, 14f, true, INK)));
            help = label(helpText == null ? "" : helpText, 11.5f, false, MUTED);
            if (helpText != null) text.add(left(help));
            add(text, BorderLayout.CENTER);
            JPanel ctl = new JPanel(new FlowLayout(FlowLayout.RIGHT, 4, 0));
            ctl.setOpaque(false);
            JButton minus = new Round("−", step, -1);
            JButton plus = new Round("+", step, 1);
            value.setFont(font(16f, true));
            value.setForeground(ACCENT);
            value.setPreferredSize(new Dimension(74, TAP));
            value.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            value.setToolTipText("tap to type a value");
            value.addMouseListener(new MouseAdapter() {
                @Override
                public void mouseReleased(MouseEvent e) {
                    step.type(value);
                }
            });
            ctl.add(minus);
            ctl.add(value);
            ctl.add(plus);
            add(ctl, BorderLayout.EAST);
        }

        void setValue(String text) {
            value.setText(text);
        }

        @Override
        public Dimension getMaximumSize() {
            return new Dimension(Integer.MAX_VALUE, getPreferredSize().height);
        }
    }

    /** The round − / + of a stepper. */
    static final class Round extends JButton {
        Round(String text, final Step step, final int dir) {
            super(text);
            setContentAreaFilled(false);
            setBorderPainted(false);
            setFocusPainted(false);
            setOpaque(false);
            setFont(font(20f, true));
            setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            addActionListener(e -> step.step(dir));
        }

        @Override
        public Dimension getPreferredSize() {
            return new Dimension(40, TAP);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = smooth(g);
            boolean down = getModel().isPressed();
            g2.setColor(down ? ACCENT : ACCENT_SOFT);
            int d = Math.min(getWidth(), getHeight()) - 4;
            g2.fillOval((getWidth() - d) / 2, (getHeight() - d) / 2, d, d);
            g2.setColor(down ? Color.WHITE : ACCENT);
            g2.setStroke(new BasicStroke(2.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
            int cx = getWidth() / 2, cy = getHeight() / 2, r = 7;
            g2.drawLine(cx - r, cy, cx + r, cy);
            if ("+".equals(getText())) g2.drawLine(cx, cy - r, cx, cy + r);
            g2.dispose();
        }
    }

    // -- a switch -------------------------------------------------------------------------------

    interface Flip {
        void flipped(boolean on);
    }

    static final class Switch extends JPanel {
        private boolean on;
        private final JComponent knob;

        Switch(String label, String helpText, boolean initial, final Flip flip) {
            this.on = initial;
            setOpaque(false);
            setLayout(new BorderLayout(8, 0));
            setAlignmentX(Component.LEFT_ALIGNMENT);
            JPanel text = column();
            text.add(left(label(label, 14f, true, INK)));
            if (helpText != null) text.add(left(label(helpText, 11.5f, false, MUTED)));
            add(text, BorderLayout.CENTER);
            knob = new JComponent() {
                @Override
                public Dimension getPreferredSize() {
                    return new Dimension(60, TAP);
                }

                @Override
                protected void paintComponent(Graphics g) {
                    Graphics2D g2 = smooth(g);
                    int y = (getHeight() - 30) / 2;
                    g2.setColor(Switch.this.on ? ACCENT : LINE);
                    g2.fillRoundRect(4, y, 52, 30, 30, 30);
                    g2.setColor(Color.WHITE);
                    g2.fillOval(Switch.this.on ? 29 : 7, y + 3, 24, 24);
                    g2.dispose();
                }
            };
            knob.setCursor(Cursor.getPredefinedCursor(Cursor.HAND_CURSOR));
            MouseAdapter toggle = new MouseAdapter() {
                @Override
                public void mouseReleased(MouseEvent e) {
                    Switch.this.on = !Switch.this.on;
                    knob.repaint();
                    flip.flipped(Switch.this.on);
                }
            };
            knob.addMouseListener(toggle);
            add(knob, BorderLayout.EAST);
        }

        void setOn(boolean v) {
            on = v;
            knob.repaint();
        }

        @Override
        public Dimension getMaximumSize() {
            return new Dimension(Integer.MAX_VALUE, getPreferredSize().height);
        }
    }

    // -- a status line --------------------------------------------------------------------

    /** A rounded, tinted message that wraps. */
    static final class Note extends JComponent {
        private String text = "";
        private Kind kind = Kind.INFO;

        void set(String t, Kind k) {
            text = t == null ? "" : t;
            kind = k;
            revalidate();
            repaint();
        }

        @Override
        public Dimension getPreferredSize() {
            int w = getWidth() > 0 ? getWidth() : 280;
            return new Dimension(w, 16 + 18 * wrap(text, getFontMetrics(font(13f, false)), w - 34).size());
        }

        @Override
        public Dimension getMaximumSize() {
            return new Dimension(Integer.MAX_VALUE, getPreferredSize().height);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Graphics2D g2 = smooth(g);
            g2.setColor(bg(kind));
            g2.fillRoundRect(0, 0, getWidth() - 1, getHeight() - 1, 12, 12);
            g2.setColor(fg(kind));
            g2.fillOval(10, 12, 9, 9);
            g2.setFont(font(13f, false));
            g2.setColor(INK);
            FontMetrics fm = g2.getFontMetrics();
            int y = 8 + fm.getAscent();
            for (String line : wrap(text, fm, getWidth() - 34)) {
                g2.drawString(line, 26, y);
                y += 18;
            }
            g2.dispose();
        }
    }

    static List<String> wrap(String text, FontMetrics fm, int width) {
        List<String> out = new ArrayList<String>();
        for (String para : text.split("\n")) {
            StringBuilder line = new StringBuilder();
            for (String word : para.split(" ")) {
                String next = line.length() == 0 ? word : line + " " + word;
                if (fm.stringWidth(next) > width && line.length() > 0) {
                    out.add(line.toString());
                    line = new StringBuilder(word);
                } else {
                    line = new StringBuilder(next);
                }
            }
            out.add(line.toString());
        }
        return out;
    }

    /** A number with its unit, one decimal at most. */
    static String value(double v, String unit) {
        return PickScript.num(v) + (unit == null || unit.isEmpty() ? "" : " " + unit);
    }

    static void centre(Graphics2D g2, String s, int cx, int cy) {
        FontMetrics fm = g2.getFontMetrics();
        g2.drawString(s, cx - fm.stringWidth(s) / 2, cy + (fm.getAscent() - fm.getDescent()) / 2);
    }
}
