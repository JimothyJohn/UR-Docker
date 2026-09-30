package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.toolbar.ToolbarContext;
import com.ur.urcap.api.contribution.toolbar.swing.SwingToolbarContribution;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import javax.swing.BorderFactory;
import javax.swing.Box;
import javax.swing.JComponent;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;

/**
 * The toolbar popup: the live picture from the camera computer, as large as the popup
 * allows, next to the mark, the live dot with the frame rate, the cockpit's address and a
 * status line. It polls only while the popup is open, and it reads the address the
 * Installation node saved — the same {@link PilotContribution} PolyScope holds for this
 * installation — so there is nothing to set up here.
 */
public class ToolbarContribution implements SwingToolbarContribution {
    private final ToolbarContext context;
    private final FeedPanel feed = new FeedPanel();
    private final JLabel dot = Ui.label("●", 16f, false, Ui.FAINT);
    private final JLabel fps = Ui.label("", 12f, false, Ui.MUTED);
    private final JLabel where = Ui.label("", 12f, false, Ui.MUTED);
    private final Ui.Note status = new Ui.Note();
    private FeedPoller poller;

    ToolbarContribution(ToolbarContext context) {
        this.context = context;
    }

    @Override
    public void buildUI(JPanel host) {
        // PolyScope's own panel refuses most setters (PilotView): lay it out, add one panel we own.
        host.setLayout(new BorderLayout());
        JPanel outer = new JPanel(new BorderLayout(12, 0));
        outer.setBackground(Ui.BG);
        outer.setBorder(BorderFactory.createEmptyBorder(8, 12, 8, 12));
        host.add(outer, BorderLayout.CENTER);
        outer.add(feed, BorderLayout.CENTER);

        JPanel side = Ui.column();
        side.setPreferredSize(new Dimension(300, 10));
        JPanel head = Ui.row(8);
        head.add(new Logo.Mark(28));
        head.add(Ui.label("Perceptronic", 20f, true, Ui.INK));
        head.add(dot);
        head.add(fps);
        side.add(head);
        side.add(Box.createVerticalStrut(6));
        side.add(Ui.left(Ui.label("the camera computer's live picture", 12f, false, Ui.MUTED)));
        side.add(Ui.left(where));
        side.add(Box.createVerticalStrut(8));
        side.add(Ui.left(status));
        side.add(Box.createVerticalStrut(8));
        side.add(Ui.left(Ui.label("Address, pick areas and reach: Installation → URCaps → Perceptronic.", 11.5f,
                false, Ui.FAINT)));
        outer.add(side, BorderLayout.EAST);
        status.set("opening…", Ui.Kind.INFO);
    }

    @Override
    public void openView() {
        Cockpit cockpit = new Cockpit(savedUrl());
        if (poller == null) poller = new FeedPoller(cockpit, listener);
        else poller.setCockpit(cockpit);
        where.setText(cockpit.base);
        status.set("connecting to " + cockpit.base + "…", Ui.Kind.INFO);
        poller.start("perceptronic-toolbar-feed");
    }

    @Override
    public void closeView() {
        if (poller != null) poller.stop();
        feed.setLive(false);
    }

    /** The address the Installation node saved for this installation ("" = the controller itself). */
    String savedUrl() {
        try {
            PilotContribution node = context.getAPIProvider().getApplicationAPI()
                    .getInstallationNode(PilotContribution.class);
            return node == null ? "" : node.savedUrl();
        } catch (RuntimeException e) {
            return "";
        }
    }

    private final FeedPoller.Listener listener = new FeedPoller.Listener() {
        @Override
        public void frame(final BufferedImage image, final String framesPerSecond) {
            onEdt(new Runnable() {
                @Override
                public void run() {
                    feed.setFrame(image);
                    dot.setForeground(Ui.OK);
                    fps.setText(PilotView.fpsText(framesPerSecond));
                }
            });
        }

        @Override
        public void live(final String base) {
            onEdt(new Runnable() {
                @Override
                public void run() {
                    status.set("live from " + base, Ui.Kind.OK);
                }
            });
        }

        @Override
        public void waiting(final String why) {
            onEdt(new Runnable() {
                @Override
                public void run() {
                    feed.setLive(false);
                    dot.setForeground(Ui.WARN);
                    fps.setText("");
                    status.set(why, Ui.Kind.WARN);
                }
            });
        }

        @Override
        public void failed(final String why) {
            onEdt(new Runnable() {
                @Override
                public void run() {
                    feed.setLive(false);
                    dot.setForeground(Ui.ERR);
                    fps.setText("");
                    status.set(why, Ui.Kind.ERR);
                }
            });
        }
    };

    private static void onEdt(Runnable r) {
        if (SwingUtilities.isEventDispatchThread()) r.run();
        else SwingUtilities.invokeLater(r);
    }

    /** The picture scaled to fit, the mark as a watermark; the test card when there is none. */
    @SuppressWarnings("serial")
    static final class FeedPanel extends JComponent {
        private transient volatile BufferedImage image;
        private volatile boolean live;

        FeedPanel() {
            setOpaque(false);
            setPreferredSize(new Dimension(640, 362));
        }

        void setFrame(BufferedImage img) {
            image = img;
            live = true;
            repaint();
        }

        void setLive(boolean on) {
            live = on;
            repaint();
        }

        @Override
        protected void paintComponent(Graphics g0) {
            Graphics2D g = Ui.smooth(g0);
            int w = getWidth(), h = getHeight();
            g.setColor(Ui.STAGE);
            g.fillRoundRect(0, 0, w, h, Ui.RADIUS, Ui.RADIUS);
            BufferedImage img = image;
            if (img == null || !live) {
                LiveView.paintNoCamera(g, w, h, "Check the camera computer and its USB 3 cable; the address is set"
                        + " in Installation → URCaps → Perceptronic.");
                g.dispose();
                return;
            }
            double k = Math.min((double) w / img.getWidth(), (double) h / img.getHeight());
            int iw = (int) Math.round(img.getWidth() * k), ih = (int) Math.round(img.getHeight() * k);
            int ox = (w - iw) / 2, oy = (h - ih) / 2;
            g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
            g.drawImage(img, ox, oy, iw, ih, null);
            Logo.watermark(g, ox + iw, oy + ih);
            g.dispose();
        }
    }
}
