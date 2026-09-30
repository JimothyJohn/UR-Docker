package com.nickarmenta.perceptronic;

import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.Polygon;
import java.awt.RenderingHints;
import java.awt.image.BufferedImage;
import javax.swing.JComponent;

/**
 * The live picture, as big as the screen allows, with what the program will do drawn on it:
 * every part it will pick outlined in green with its pick-order number and the two jaws
 * where the fingers close (across the short side), everything else it saw dashed grey with
 * why not, and a heads-up line of what it is looking for.
 */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
final class LiveView extends JComponent {
    private volatile BufferedImage frame;
    private volatile Scene scene = Scene.empty();
    private volatile boolean live;
    private volatile String fps = "";
    private volatile String headline = "";
    private volatile String empty = "Check the camera computer and its USB 3 cable — the picture returns by itself.";

    LiveView() {
        setOpaque(true);
        setPreferredSize(new Dimension(640, 400));
    }

    void setFrame(BufferedImage image) {
        frame = image;
        repaint();
    }

    void setScene(Scene s) {
        scene = s == null ? Scene.empty() : s;
        repaint();
    }

    void setLive(boolean on, String framesPerSecond) {
        live = on;
        fps = framesPerSecond == null ? "" : framesPerSecond;
        repaint();
    }

    void setHeadline(String text) {
        headline = text == null ? "" : text;
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
        Logo.watermark(g, ox + iw, oy + ih);
        Scene s = scene;
        // the scene's pixels are the camera computer's frame; the image may be scaled
        double sk = s.width > 0 ? (double) iw / s.width : k;
        for (Scene.Part p : s.rejected) drawRejected(g, p, ox, oy, sk);
        for (Scene.Part p : s.parts) drawPart(g, p, ox, oy, sk);
        hud(g, s, w, h);
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
        int pw = Math.min(w - 40, 560), ph = 128;
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
        for (String line : Ui.wrap(detail == null ? "" : detail, g.getFontMetrics(), pw - 40)) {
            Ui.centre(g, line, w / 2, y);
            y += 18;
        }
    }

    private static Polygon poly(Scene.Part p, int ox, int oy, double k) {
        Polygon poly = new Polygon();
        for (int[] c : p.corners) poly.addPoint(ox + (int) Math.round(c[0] * k), oy + (int) Math.round(c[1] * k));
        return poly;
    }

    private static void drawRejected(Graphics2D g, Scene.Part p, int ox, int oy, double k) {
        Polygon poly = poly(p, ox, oy, k);
        g.setColor(new Color(255, 255, 255, 36));
        g.fillPolygon(poly);
        g.setColor(new Color(0xb9c3cf));
        g.setStroke(new BasicStroke(1.6f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND, 1f, new float[] {6f, 5f}, 0f));
        g.drawPolygon(poly);
        if (p.why != null) {
            int x = ox + (int) Math.round(p.u * k), y = oy + (int) Math.round(p.v * k);
            tag(g, p.why, x, y, new Color(20, 26, 34, 200), new Color(0xdfe5ec), 11.5f);
        }
    }

    private static void drawPart(Graphics2D g, Scene.Part p, int ox, int oy, double k) {
        Polygon poly = poly(p, ox, oy, k);
        g.setColor(new Color(61, 220, 132, 70));
        g.fillPolygon(poly);
        g.setColor(Ui.PART);
        g.setStroke(new BasicStroke(2.4f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.drawPolygon(poly);
        // the jaws: on the two long edges (corners 0-1 and 2-3), just outside, a third of their length
        double cx = 0, cy = 0;
        for (int i = 0; i < 4; i++) {
            cx += poly.xpoints[i] / 4.0;
            cy += poly.ypoints[i] / 4.0;
        }
        g.setColor(Ui.JAW);
        g.setStroke(new BasicStroke(5f, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        for (int[] e : new int[][] {{0, 1}, {2, 3}}) {
            double ax = poly.xpoints[e[0]], ay = poly.ypoints[e[0]], bx = poly.xpoints[e[1]], by = poly.ypoints[e[1]];
            double mx = (ax + bx) / 2, my = (ay + by) / 2;
            double nx = mx - cx, ny = my - cy, nl = Math.max(1e-6, Math.hypot(nx, ny));
            double off = 7;
            double dx = (bx - ax) * 0.18, dy = (by - ay) * 0.18;
            int x0 = (int) Math.round(mx + nx / nl * off - dx), y0 = (int) Math.round(my + ny / nl * off - dy);
            int x1 = (int) Math.round(mx + nx / nl * off + dx), y1 = (int) Math.round(my + ny / nl * off + dy);
            g.drawLine(x0, y0, x1, y1);
        }
        // the pick-order number
        int r = p.order == 1 ? 17 : 14;
        int x = (int) Math.round(cx), y = (int) Math.round(cy);
        g.setColor(new Color(0, 0, 0, 90));
        g.fillOval(x - r + 1, y - r + 2, 2 * r, 2 * r);
        g.setColor(p.order == 1 ? Ui.ACCENT : new Color(0x2b3a4a));
        g.fillOval(x - r, y - r, 2 * r, 2 * r);
        g.setColor(Color.WHITE);
        g.setStroke(new BasicStroke(2f));
        g.drawOval(x - r, y - r, 2 * r, 2 * r);
        g.setFont(Ui.font(p.order == 1 ? 17f : 14f, true));
        Ui.centre(g, String.valueOf(p.order), x, y);
    }

    private void hud(Graphics2D g, Scene s, int w, int h) {
        // top left: what it sees
        String what = s.parts.isEmpty() ? (s.reason != null && !s.rejected.isEmpty() ? s.reason : "no part in view")
                : s.parts.size() + (s.parts.size() == 1 ? " part" : " parts") + " · #1 is picked first";
        tag(g, what, 14, 14, new Color(13, 19, 26, 210), Color.WHITE, 14f, false);
        if (!headline.isEmpty()) tag(g, headline, 14, 46, new Color(13, 19, 26, 170), new Color(0xc9d3de), 12f, false);
        // top right: live
        String badge = live ? "LIVE" + (fps.isEmpty() ? "" : "  " + fps + " fps") : "NO FEED";
        g.setFont(Ui.font(12f, true));
        FontMetrics fm = g.getFontMetrics();
        int bw = fm.stringWidth(badge) + 34;
        g.setColor(new Color(13, 19, 26, 210));
        g.fillRoundRect(w - bw - 14, 14, bw, 26, 26, 26);
        g.setColor(live ? Ui.PART : Ui.ERR);
        g.fillOval(w - bw - 2, 23, 9, 9);
        g.setColor(Color.WHITE);
        g.drawString(badge, w - bw + 12, 14 + (26 + fm.getAscent() - fm.getDescent()) / 2);
        // bottom left: the surface
        if (s.surface != null) {
            String surf = "taught".equals(s.surface)
                    ? String.format(java.util.Locale.ROOT, "pick area taught · table %+.0f mm", s.offsetMm)
                    : "table found live";
            if (!s.baseFrame) surf += " · no robot pose: reach not checked";
            tag(g, surf, 14, h - 40, new Color(13, 19, 26, 170), new Color(0xc9d3de), 12f, false);
        }
    }

    private static void tag(Graphics2D g, String text, int cx, int cy, Color bg, Color fg, float size) {
        g.setFont(Ui.font(size, true));
        FontMetrics fm = g.getFontMetrics();
        int tw = fm.stringWidth(text) + 14, th = fm.getHeight() + 4;
        int x = cx - tw / 2, y = cy + 18;
        g.setColor(bg);
        g.fillRoundRect(x, y, tw, th, th, th);
        g.setColor(fg);
        g.drawString(text, x + 7, y + 2 + fm.getAscent());
    }

    private static void tag(Graphics2D g, String text, int x, int y, Color bg, Color fg, float size, boolean bold) {
        g.setFont(Ui.font(size, bold));
        if (!bold) g.setFont(g.getFont().deriveFont(Font.BOLD));
        FontMetrics fm = g.getFontMetrics();
        int tw = fm.stringWidth(text) + 20, th = fm.getHeight() + 8;
        g.setColor(bg);
        g.fillRoundRect(x, y, tw, th, th, th);
        g.setColor(fg);
        g.drawString(text, x + 10, y + 4 + fm.getAscent());
    }
}
