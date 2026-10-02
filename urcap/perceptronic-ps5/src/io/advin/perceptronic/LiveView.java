package io.advin.perceptronic;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import java.util.List;
import javax.swing.JComponent;

/**
 * The live picture, as big as the screen allows — the camera's picture or, with the toggle
 * at its top right, the depth as a heatmap. Two things are drawn over it (0.8.0, Nick
 * 2026-10-01): every part that will be picked in <b>green with its number</b> in the pick
 * order, and every candidate that is nearly the part and will not be — a little off its size,
 * out of reach, no finger room — in <b>yellow with why</b>. Nothing else: not the jaws, not
 * what is nothing like the part.
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
final class LiveView extends JComponent {
    /** Told when the operator taps the Picture / Depth toggle. */
    interface ViewListener {
        void depthView(boolean on);
    }

    private volatile BufferedImage frame;
    private volatile Scene scene = Scene.empty();
    private volatile boolean live;
    private volatile boolean depth;
    private volatile String empty = "Check the camera computer and its USB 3 cable — the picture returns by itself.";
    private ViewListener listener;

    LiveView() {
        setOpaque(true);
        setPreferredSize(new Dimension(640, 400));
        addMouseListener(new java.awt.event.MouseAdapter() {
            @Override
            public void mouseReleased(java.awt.event.MouseEvent e) {
                if (!live || !Ui.ViewToggle.bounds(getWidth(), 0).contains(e.getPoint())) return;
                depth = e.getX() >= Ui.ViewToggle.bounds(getWidth(), 0).getCenterX();
                repaint();
                if (listener != null) listener.depthView(depth);
            }
        });
    }

    void setViewListener(ViewListener l) {
        listener = l;
    }

    boolean depthView() {
        return depth;
    }

    /** Show the toggle on Depth (or Picture) without a tap. */
    void setDepthView(boolean on) {
        depth = on;
        repaint();
    }

    void setFrame(BufferedImage image) {
        frame = image;
        repaint();
    }

    void setScene(Scene s) {
        scene = s == null ? Scene.empty() : s;
        repaint();
    }

    void setLive(boolean on) {
        live = on;
        repaint();
    }

    void setEmptyText(String text) {
        empty = text == null ? "" : text;
        repaint();
    }

    @Override
    protected void paintComponent(Graphics g0) {
        Graphics2D g = Ui.smooth(g0);
        int w = getWidth(), h = getHeight();
        g.setColor(Ui.STAGE);
        g.fillRoundRect(0, 0, w, h, Ui.RADIUS, Ui.RADIUS);
        BufferedImage img = frame;
        if (img == null || !live) {
            // never a blank or a stale picture: an unmistakable test card
            paintNoCamera(g, w, h, empty);
            g.dispose();
            return;
        }
        double k = Math.min((double) w / img.getWidth(), (double) h / img.getHeight());
        int iw = (int) Math.round(img.getWidth() * k), ih = (int) Math.round(img.getHeight() * k);
        int ox = (w - iw) / 2, oy = (h - ih) / 2;
        g.setRenderingHint(RenderingHints.KEY_INTERPOLATION, RenderingHints.VALUE_INTERPOLATION_BILINEAR);
        g.drawImage(img, ox, oy, iw, ih, null);
        Scene s = scene;
        // the scene's pixels are the colour picture's; the image drawn may be scaled (the depth is half-size)
        double sk = s.width > 0 ? (double) iw / s.width : k;
        for (Scene.Part p : s.nearMisses()) drawNearMiss(g, p, ox, oy, sk);
        for (Scene.Part p : s.parts) drawPart(g, p, ox, oy, sk);
        Ui.ViewToggle.paint(g, w, 0, depth);
        g.dispose();
    }

    /**
     * The test card shown whenever there is no live picture: grey bars, a hatch, and a plate
     * that says NO CAMERA CONNECTED — so nobody mistakes an empty or frozen view for a camera
     * that sees nothing. {@code detail}: one line under it (may be empty).
     */
    static void paintNoCamera(Graphics2D g, int w, int h, String detail) {
        g.setColor(new Color(0x1a2029));
        g.fillRect(0, 0, w, h);
        Color[] bars = {
            new Color(0x9aa0a6), new Color(0xa39b5e), new Color(0x5e9a9c), new Color(0x5e9a5e),
            new Color(0x9a5e9a), new Color(0x9a5e5e), new Color(0x5e5e9a),
        };
        int bh = h / 3;
        for (int i = 0; i < bars.length; i++) {
            g.setColor(bars[i].darker());
            g.fillRect(i * w / bars.length, 0, (i + 1) * w / bars.length - i * w / bars.length, bh);
        }
        g.setColor(new Color(255, 255, 255, 18));
        g.setStroke(new BasicStroke(1f));
        for (int x = -h; x < w; x += 22) g.drawLine(x, h, x + h, 0);
        int pw = Math.min(w - 40, 600);
        List<String> lines = Ui.wrap(detail == null ? "" : detail, g.getFontMetrics(Ui.font(13f, false)), pw - 40);
        int ph = Math.min(h - 16, 84 + 18 * lines.size());
        int px = (w - pw) / 2, py = (h - ph) / 2;
        g.setColor(new Color(0, 0, 0, 120));
        g.fillRoundRect(px + 3, py + 5, pw, ph, 18, 18);
        g.setColor(new Color(0x0d131a));
        g.fillRoundRect(px, py, pw, ph, 18, 18);
        g.setColor(Ui.ERR);
        g.setStroke(new BasicStroke(3f));
        g.drawRoundRect(px, py, pw, ph, 18, 18);
        g.setColor(Color.WHITE);
        g.setFont(Ui.font(Math.min(34f, pw / 15f), true));
        Ui.centre(g, "NO CAMERA CONNECTED", w / 2, py + 50);
        g.setFont(Ui.font(13f, false));
        g.setColor(new Color(0xc9d3de));
        int y = py + 84;
        for (String line : lines) {
            if (y > py + ph - 8) break;
            Ui.centre(g, line, w / 2, y);
            y += 18;
        }
    }

    private static Polygon poly(Scene.Part p, int ox, int oy, double k) {
        Polygon poly = new Polygon();
        for (int[] c : p.corners) poly.addPoint(ox + (int) Math.round(c[0] * k), oy + (int) Math.round(c[1] * k));
        return poly;
    }

    /** A part that will be picked: green, with its number in the pick order. */
    private static void drawPart(Graphics2D g, Scene.Part p, int ox, int oy, double k) {
        Polygon poly = poly(p, ox, oy, k);
        g.setColor(new Color(61, 220, 132, 60));
        g.fillPolygon(poly);
        g.setColor(Ui.PART);
        g.setStroke(new BasicStroke(2.4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.drawPolygon(poly);
        int x = ox + (int) Math.round(p.u * k), y = oy + (int) Math.round(p.v * k), r = 14;
        g.setColor(new Color(0x12824a));
        g.fillOval(x - r, y - r, 2 * r, 2 * r);
        g.setColor(Color.WHITE);
        g.setStroke(new BasicStroke(2f));
        g.drawOval(x - r, y - r, 2 * r, 2 * r);
        g.setFont(Ui.font(14f, true));
        Ui.centre(g, String.valueOf(p.order), x, y);
    }

    /** A candidate that will not be picked: yellow, with why. */
    private static void drawNearMiss(Graphics2D g, Scene.Part p, int ox, int oy, double k) {
        Polygon poly = poly(p, ox, oy, k);
        g.setColor(new Color(0, 0, 0, 110));
        g.setStroke(new BasicStroke(4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.drawPolygon(poly);
        g.setColor(Ui.JAW);
        g.setStroke(new BasicStroke(2f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1f, new float[] {7f, 5f}, 0f));
        g.drawPolygon(poly);
        if (p.why == null) return;
        g.setFont(Ui.font(12f, true));
        FontMetrics fm = g.getFontMetrics();
        int tw = fm.stringWidth(p.why) + 14, th = fm.getHeight() + 4;
        int x = ox + (int) Math.round(p.u * k) - tw / 2, y = oy + (int) Math.round(p.v * k) + 18;
        g.setColor(new Color(20, 26, 34, 215));
        g.fillRoundRect(x, y, tw, th, th, th);
        g.setColor(Ui.JAW);
        g.drawString(p.why, x + 7, y + 2 + fm.getAscent());
    }
}
