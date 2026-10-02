package io.advin.perceptronic;

import java.awt.image.BufferedImage;
import java.io.IOException;

/**
 * The live picture, long-polled from the cockpit's ``GET /api/color.png`` by sequence
 * number on a daemon thread — one loop for every place that shows it (the Installation
 * node, the toolbar popup). No UR API, so it runs and is tested on any JDK. Every callback
 * comes on the poller's thread; the listener marshals onto Swing itself.
 */
final class FeedPoller implements Runnable {
    interface Listener {
        /** A new frame: the feed is live ({@code fps} as the cockpit reports it, may be null). */
        void frame(BufferedImage image, String fps);

        /** The first frame after the cockpit came (back): {@code base} is its URL. */
        void live(String base);

        /** The cockpit answers but has no frame yet (HTTP 503: camera opening?). */
        void waiting(String why);

        /** No usable answer — {@code why} says what to do about it. */
        void failed(String why);
    }

    static final int POLL_TIMEOUT_MS = 1500;
    private static final long RETRY_MS = 1500;
    private static final long WAIT_MS = 500;

    private final Listener listener;
    private volatile Cockpit cockpit;
    private volatile long seq;
    private volatile boolean depth;
    private volatile Thread thread;

    FeedPoller(Cockpit cockpit, Listener listener) {
        this.cockpit = cockpit;
        this.listener = listener;
    }

    Cockpit cockpit() {
        return cockpit;
    }

    /** Poll another cockpit from now on (from the first frame again). */
    void setCockpit(Cockpit c) {
        cockpit = c;
        seq = 0;
    }

    /** Poll the depth heatmap ({@code true}) or the colour picture from now on. */
    void setDepthView(boolean on) {
        depth = on;
    }

    boolean depthView() {
        return depth;
    }

    /** Start polling on a daemon thread called {@code name}; a running poller is left alone. */
    synchronized void start(String name) {
        Thread t = thread;
        if (t != null && t.isAlive()) return;
        t = new Thread(this, name);
        t.setDaemon(true);
        thread = t;
        t.start();
    }

    /** Stop after the request in flight (at most {@link #POLL_TIMEOUT_MS} + connect). */
    synchronized void stop() {
        Thread t = thread;
        thread = null;
        if (t != null) t.interrupt();
    }

    boolean running() {
        Thread t = thread;
        return t != null && t.isAlive();
    }

    @Override
    public void run() {
        boolean announced = false;
        while (thread == Thread.currentThread() && !Thread.currentThread().isInterrupted()) {
            Cockpit c = cockpit;
            try {
                boolean heat = depth;
                Cockpit.Frame f = c.framePng(heat, seq, POLL_TIMEOUT_MS);
                if (f.status == 503) {
                    listener.waiting(Cockpit.noPicture(c.base, null));
                    sleep(WAIT_MS);
                    continue;
                }
                if (f.status == 404 && heat) {
                    depth = false; // a camera computer older than 0.7.0 has no heatmap: the picture instead
                    Log.detail(c.base + " answered 404 on /api/depth.png: it predates the depth view; update it");
                    continue;
                }
                if (f.status != 200) {
                    announced = false;
                    Log.detail(c.base + " answered HTTP " + f.status + " on /api/color.png");
                    listener.failed("The camera computer answers, but its software is "
                            + (f.status == 404 ? "older than this URCap" : "not answering as expected (HTTP " + f.status
                            + ")") + ".\n• Update the camera computer's software and restart it");
                    sleep(RETRY_MS);
                    continue;
                }
                seq = f.seq;
                listener.frame(f.image, f.fps);
                if (!announced) {
                    listener.live(c.base);
                    announced = true;
                }
            } catch (IOException e) {
                announced = false;
                listener.failed(Cockpit.explain(e, c.base));
                sleep(RETRY_MS);
            } catch (RuntimeException e) {
                announced = false;
                listener.failed(Cockpit.explain(e, c.base));
                sleep(RETRY_MS);
            }
        }
    }

    private static void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
