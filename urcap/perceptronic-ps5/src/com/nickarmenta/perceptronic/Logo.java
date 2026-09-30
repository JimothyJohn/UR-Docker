package com.nickarmenta.perceptronic;

import java.awt.AlphaComposite;
import java.awt.BasicStroke;
import java.awt.Color;
import java.awt.Dimension;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.geom.Arc2D;
import java.awt.geom.Path2D;
import java.awt.image.BufferedImage;
import javax.swing.Icon;
import javax.swing.ImageIcon;
import javax.swing.JComponent;

/**
 * The Perceptronic mark: a P whose bowl holds a lens. One drawing, three uses — the
 * toolbar button (a filled badge), the header of every screen, and a faint watermark on
 * every live picture. {@link #SVG} is the same glyph as ``urcap/perceptronic.svg`` (the
 * PolyScope X node's icon and the docs' logo); a test holds the two equal, and the
 * geometry below is that path's coordinates in a 64 × 64 box.
 */
final class Logo {
    private Logo() {
    }

    /** ``urcap/perceptronic.svg``, byte for byte. */
    static final String SVG = "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 64 64\" width=\"64\" height=\"64\""
            + " fill=\"none\" stroke=\"currentColor\" stroke-width=\"8\" stroke-linecap=\"round\""
            + " stroke-linejoin=\"round\">\n"
            + "  <path d=\"M22 54V10h14a14 14 0 0 1 0 28H22\"/>\n"
            + "  <circle cx=\"36\" cy=\"24\" r=\"5\" fill=\"currentColor\" stroke=\"none\"/>\n"
            + "</svg>\n";

    private static final int BOX = 64;
    private static final float STROKE = 8f;

    /** The P's outline in the 64-box: stem, top bar, the bowl (an arc about (36, 24), r 14). */
    private static Path2D glyph() {
        Path2D p = new Path2D.Double();
        p.moveTo(22, 54);
        p.lineTo(22, 10);
        p.lineTo(36, 10);
        p.append(new Arc2D.Double(22, 10, 28, 28, 90, -180, Arc2D.OPEN), true);
        p.lineTo(22, 38);
        return p;
    }

    /**
     * Paint the mark {@code size} px square at ({@code x}, {@code y}): the glyph in
     * {@code ink}, on a rounded badge of {@code badge} when that is not null.
     */
    static void paint(Graphics2D g0, int x, int y, int size, Color ink, Color badge) {
        Graphics2D g = Ui.smooth(g0);
        double k = size / (double) BOX;
        g.translate(x, y);
        g.scale(k, k);
        if (badge != null) {
            g.setColor(badge);
            g.fillRoundRect(0, 0, BOX, BOX, 18, 18);
        }
        g.setColor(ink);
        g.setStroke(new BasicStroke(STROKE, BasicStroke.CAP_ROUND, BasicStroke.JOIN_ROUND));
        g.draw(glyph());
        g.fillOval(31, 19, 10, 10);
        g.dispose();
    }

    /** The mark as an image: a badge in {@code badge} (null = transparent) with the glyph in {@code ink}. */
    static BufferedImage image(int size, Color ink, Color badge) {
        BufferedImage img = new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g = img.createGraphics();
        paint(g, 0, 0, size, ink, badge);
        g.dispose();
        return img;
    }

    /** The toolbar button: white P on the accent badge. */
    static Icon icon(int size) {
        return new ImageIcon(image(size, Color.WHITE, Ui.ACCENT));
    }

    /** A faint P in the bottom-right corner of a {@code w} × {@code h} picture. */
    static void watermark(Graphics2D g0, int w, int h) {
        int size = Math.max(18, Math.min(36, Math.min(w, h) / 9));
        Graphics2D g = (Graphics2D) g0.create();
        g.setComposite(AlphaComposite.getInstance(AlphaComposite.SRC_OVER, 0.38f));
        paint(g, w - size - 10, h - size - 10, size, Color.WHITE, null);
        g.dispose();
    }

    /** The mark, inline in a row. */
    // Swing components are never serialized here; javac's serial lint does not apply to them
    @SuppressWarnings("serial")
    static final class Mark extends JComponent {
        private final int size;

        Mark(int size) {
            this.size = size;
            setPreferredSize(new Dimension(size, size));
            setOpaque(false);
        }

        @Override
        protected void paintComponent(Graphics g) {
            Logo.paint((Graphics2D) g, 0, 0, size, Color.WHITE, Ui.ACCENT);
        }
    }
}
