package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.installation.swing.SwingInstallationNodeView;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardInputCallback;
import com.ur.urcap.api.domain.userinteraction.keyboard.KeyboardTextInput;
import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.CardLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Insets;
import java.awt.Rectangle;
import java.awt.RenderingHints;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.image.BufferedImage;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextArea;
import javax.swing.JTextField;
import javax.swing.SwingUtilities;

/**
 * The PolyScope X node's page, in Swing: title + live dot + fps, the Cockpit field with
 * Save, the colour feed (hover for depth, tap a point, the yellow mark), a status line,
 * the located target, and Move (PolyScope) / Move (cockpit) / Bring up / STOP / Clear.
 * "Open cockpit" is gone — the pendant has no browser to open it in. A second tab holds the
 * pick areas the Perceptronic Pick node looks at, and the reach ({@link LocationsScreen}).
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
public class PilotView implements SwingInstallationNodeView<PilotContribution> {
    enum Kind { INFO, OK, WARN, ERR }

    // the PolyScope X node's palette (urcap/perceptronic/.../main.js CSS)
    private static final Color INK = new Color(0x1f2a37);
    private static final Color MUTED = new Color(0x5b6b7d);
    private static final Color STATUS_BG = new Color(0xeef2f7);
    private static final Color OK_BG = new Color(0xe3f5ea);
    private static final Color WARN_BG = new Color(0xfff4d6);
    private static final Color ERR_BG = new Color(0xfde2e2);
    private static final Color LIVE = new Color(0x1d9a5a);
    private static final Color DEAD = new Color(0xd64545);
    private static final Color IDLE = new Color(0xc0c8d2);
    private static final Color PRIMARY = new Color(0x1f5fbf);
    private static final Color DANGER = new Color(0xd64545);
    private static final Color STAGE = new Color(0x0f1620);
    private static final Color MARK = new Color(0xffd23f);

    private final ViewAPIProvider api;
    private final JLabel dot = new JLabel("●");
    private final JLabel fps = new JLabel("");
    private final JTextField url = new JTextField(34);
    private final Feed feed = new Feed();
    private final JTextArea status = area(false);
    private final JTextArea target = area(true);
    private final JButton movePs = button("Move (PolyScope)", PRIMARY, Color.WHITE);
    private final JButton moveCk = button("Move (cockpit)", null, null);
    private final JButton bringUp = button("Bring up", null, null);
    private final JButton stop = button("STOP", DANGER, Color.WHITE);
    private final JButton clear = button("Clear", null, null);
    private PilotContribution node;
    private LocationsScreen areas;

    PilotView(ViewAPIProvider api) {
        this.api = api;
    }

    @Override
    public void buildUI(JPanel host, final PilotContribution contribution) {
        this.node = contribution;
        // PolyScope's own panel refuses most setters (setBorder throws "Method not
        // supported from URCaps", verified in URSim 5.26): lay it out, add one panel we own.
        host.setLayout(new BorderLayout());
        JPanel outer = new JPanel(new BorderLayout(0, 6));
        outer.setBackground(Ui.BG);
        outer.setBorder(BorderFactory.createEmptyBorder(8, 12, 8, 12));
        host.add(outer, BorderLayout.CENTER);
        JPanel panel = new JPanel();
        panel.setOpaque(false);
        panel.setLayout(new BoxLayout(panel, BoxLayout.Y_AXIS));
        final CardLayout cards = new CardLayout();
        final JPanel deck = new JPanel(cards);
        deck.setOpaque(false);
        deck.add(panel, "camera");
        areas = new LocationsScreen(contribution);
        deck.add(areas, "areas");
        outer.add(deck, BorderLayout.CENTER);

        JPanel top = new JPanel(new BorderLayout());
        top.setOpaque(false);
        JPanel title = row();
        title.setOpaque(false);
        dot.setForeground(IDLE);
        dot.setFont(dot.getFont().deriveFont(16f));
        JLabel name = new JLabel("Perceptronic");
        name.setFont(name.getFont().deriveFont(Font.BOLD, 20f));
        name.setForeground(INK);
        fps.setForeground(MUTED);
        title.add(new Logo.Mark(28));
        title.add(name);
        title.add(dot);
        title.add(fps);
        top.add(title, BorderLayout.WEST);
        Ui.Segmented tabs = new Ui.Segmented(new String[] {"Camera", "Pick areas + reach"}, 0,
                i -> cards.show(deck, i == 0 ? "camera" : "areas"));
        tabs.setPreferredSize(new Dimension(340, 40));
        top.add(tabs, BorderLayout.EAST);
        outer.add(top, BorderLayout.NORTH);

        JPanel cockpitRow = row();
        JLabel label = new JLabel("Cockpit");
        label.setForeground(INK);
        url.setFont(url.getFont().deriveFont(16f));
        url.setToolTipText("http://<camera-computer-ip>:7621 (empty = this controller:7621)");
        url.addMouseListener(new MouseAdapter() {
            @Override
            public void mousePressed(MouseEvent e) {
                showKeyboard();
            }
        });
        JButton save = button("Save", null, null);
        save.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                node.saveUrl(url.getText());
            }
        });
        cockpitRow.add(label);
        cockpitRow.add(url);
        cockpitRow.add(save);
        panel.add(cockpitRow);

        feed.setAlignmentX(Component.LEFT_ALIGNMENT);
        panel.add(feed);
        panel.add(Box.createVerticalStrut(6));
        panel.add(status);
        panel.add(Box.createVerticalStrut(4));
        panel.add(target);

        JPanel buttons = row();
        movePs.setToolTipText("The controller's joint solution + PolyScope's move screen (hold to move)");
        moveCk.setToolTipText("The cockpit moves the arm over Primary through urctl's safety envelope");
        bringUp.setToolTipText("power on + brake release + unlock protective stop, via the cockpit");
        for (JButton b : new JButton[] {movePs, moveCk, bringUp, stop, clear}) buttons.add(b);
        movePs.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                node.movePolyScope();
            }
        });
        moveCk.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                node.moveCockpit();
            }
        });
        bringUp.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                node.bringUp();
            }
        });
        stop.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                node.stop();
            }
        });
        clear.addActionListener(new ActionListener() {
            @Override
            public void actionPerformed(ActionEvent e) {
                node.clear();
            }
        });
        panel.add(buttons);

        JLabel hint = new JLabel("Tap = segment at the pixel → point in the base frame through the hand-eye → "
                + "approach pose above it. Reach is checked before a move is offered.");
        hint.setForeground(MUTED);
        hint.setFont(hint.getFont().deriveFont(11f));
        hint.setAlignmentX(Component.LEFT_ALIGNMENT);
        panel.add(hint);

        movePs.setEnabled(false);
        moveCk.setEnabled(false);
        setStatus("connecting…", Kind.INFO);
    }

    private void showKeyboard() {
        KeyboardTextInput kb = api.getUserInterfaceAPI().getUserInteraction().getKeyboardInputFactory()
                .createStringKeyboardInput();
        kb.setInitialValue(url.getText());
        kb.show(url, new KeyboardInputCallback<String>() {
            @Override
            public void onOk(String value) {
                url.setText(value);
                node.saveUrl(value);
            }
        });
    }

    LocationsScreen areas() {
        return areas;
    }

    // -- setters the contribution calls from any thread ------------------------------------------

    void showUrl(final String value) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                url.setText(value);
            }
        });
    }

    void setStatus(final String text, final Kind kind) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                status.setText(text);
                status.setBackground(kind == Kind.OK ? OK_BG : kind == Kind.WARN ? WARN_BG : kind == Kind.ERR ? ERR_BG : STATUS_BG);
            }
        });
    }

    void setTarget(final String text) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                target.setText(text);
                target.setVisible(!text.isEmpty());
            }
        });
    }

    void setLive(final boolean live, final String framesPerSecond) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                dot.setForeground(live ? LIVE : DEAD);
                feed.live = live;
                feed.repaint();
                fps.setText(live ? fpsText(framesPerSecond) : "");
            }
        });
    }

    void setFrame(final BufferedImage image) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                feed.image = image;
                feed.repaint();
            }
        });
    }

    void setHover(final String text) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                feed.hover = text;
                feed.repaint();
            }
        });
    }

    void setMoveEnabled(final boolean polyscope, final boolean cockpit) {
        onEdt(new Runnable() {
            @Override
            public void run() {
                movePs.setEnabled(polyscope);
                moveCk.setEnabled(cockpit);
            }
        });
    }

    void clearMark() {
        onEdt(new Runnable() {
            @Override
            public void run() {
                feed.markX = -1;
                feed.repaint();
            }
        });
    }

    /** ``29.8 fps`` from the cockpit's X-Fps header; "" when it sent none or nonsense. */
    static String fpsText(String framesPerSecond) {
        if (framesPerSecond == null) return "";
        try {
            return String.format(java.util.Locale.ROOT, "%.1f fps", Double.parseDouble(framesPerSecond));
        } catch (NumberFormatException e) {
            return "";
        }
    }

    private static void onEdt(Runnable r) {
        if (SwingUtilities.isEventDispatchThread()) r.run();
        else SwingUtilities.invokeLater(r);
    }

    // -- widgets ---------------------------------------------------------------------------------

    private static JPanel row() {
        JPanel p = new JPanel(new FlowLayout(FlowLayout.LEFT, 8, 4));
        p.setOpaque(false);
        p.setAlignmentX(Component.LEFT_ALIGNMENT);
        return p;
    }

    private static JButton button(String text, Color bg, Color fg) {
        JButton b = new JButton(text);
        b.setFont(b.getFont().deriveFont(15f));
        b.setMargin(new Insets(8, 14, 8, 14));
        if (bg != null) {
            b.setBackground(bg);
            b.setForeground(fg);
            b.setOpaque(true);
        }
        return b;
    }

    private static JTextArea area(boolean mono) {
        JTextArea a = new JTextArea();
        a.setEditable(false);
        a.setLineWrap(true);
        a.setWrapStyleWord(true);
        a.setForeground(INK);
        a.setBackground(STATUS_BG);
        a.setBorder(BorderFactory.createEmptyBorder(6, 10, 6, 10));
        a.setAlignmentX(Component.LEFT_ALIGNMENT);
        a.setMaximumSize(new Dimension(Integer.MAX_VALUE, 110));
        if (mono) {
            a.setFont(new Font(Font.MONOSPACED, Font.PLAIN, 12));
            a.setOpaque(false);
            a.setVisible(false);
        }
        return a;
    }

    /** The colour feed, scaled to fit, with the tap mark and the hover readout. */
    private final class Feed extends JComponent {
        private static final long serialVersionUID = 1L;
        private static final int W = 640;
        private static final int H = 362; // 848×480 scaled to 640 wide
        transient volatile BufferedImage image;
        volatile String hover = "hover for depth · tap a point";
        volatile boolean live;
        int markX = -1;
        int markY = -1;

        Feed() {
            setPreferredSize(new Dimension(W, H));
            setMaximumSize(new Dimension(W, H));
            setMinimumSize(new Dimension(W / 2, H / 2));
            MouseAdapter m = new MouseAdapter() {
                @Override
                public void mouseMoved(MouseEvent e) {
                    int[] px = pixelOf(e);
                    if (px != null && node != null) node.onHover(px[0], px[1]);
                }

                @Override
                public void mouseClicked(MouseEvent e) {
                    int[] px = pixelOf(e);
                    if (px == null || node == null) return;
                    markX = e.getX();
                    markY = e.getY();
                    repaint();
                    node.onClick(px[0], px[1]);
                }
            };
            addMouseListener(m);
            addMouseMotionListener(m);
        }

        private Rectangle drawn() {
            BufferedImage img = image;
            if (img == null) return null;
            double s = Math.min(getWidth() / (double) img.getWidth(), getHeight() / (double) img.getHeight());
            int w = (int) Math.round(img.getWidth() * s);
            int h = (int) Math.round(img.getHeight() * s);
            return new Rectangle(0, 0, w, h);
        }

        private int[] pixelOf(MouseEvent e) {
            BufferedImage img = image;
            Rectangle r = drawn();
            if (img == null || r == null || r.width == 0 || !r.contains(e.getPoint())) return null;
            int x = (int) Math.round(e.getX() * (img.getWidth() / (double) r.width));
            int y = (int) Math.round(e.getY() * (img.getHeight() / (double) r.height));
            return new int[] {Math.max(0, Math.min(img.getWidth() - 1, x)), Math.max(0, Math.min(img.getHeight() - 1, y))};
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = (Graphics2D) g0.create();
            g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
            g.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);
            g.setColor(STAGE);
            g.fillRect(0, 0, getWidth(), getHeight());
            BufferedImage img = image;
            Rectangle r = drawn();
            if (img == null || !live) {
                LiveView.paintNoCamera(g, getWidth(), getHeight(),
                        "Check the camera computer's address above, and its camera's USB 3 cable.");
                g.dispose();
                return;
            }
            if (r != null) {
                g.drawImage(img, r.x, r.y, r.width, r.height, null);
                Logo.watermark(g, r.x + r.width, r.y + r.height);
            }
            if (markX >= 0) {
                g.setColor(new Color(0, 0, 0, 128));
                g.setStroke(new BasicStroke(4f));
                g.drawOval(markX - 11, markY - 11, 22, 22);
                g.setColor(MARK);
                g.setStroke(new BasicStroke(2f));
                g.drawOval(markX - 11, markY - 11, 22, 22);
                g.fillOval(markX - 2, markY - 2, 4, 4);
            }
            String h = hover;
            if (h != null && !h.isEmpty()) {
                g.setFont(getFont().deriveFont(12f));
                int tw = g.getFontMetrics().stringWidth(h);
                int y = (r != null ? r.height : getHeight()) - 8;
                g.setColor(new Color(15, 22, 32, 204));
                g.fillRoundRect(8, y - 18, tw + 16, 22, 8, 8);
                g.setColor(new Color(0xe8eef6));
                g.drawString(h, 16, y - 3);
            }
            g.dispose();
        }
    }
}
