package io.advin.perceptronic;

import java.util.ArrayList;
import java.util.List;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 * Where the long story goes. The screens tell the operator what to check in a few plain lines;
 * the exception, the URL and the command that would fix it are written here — the JVM's own
 * logger (name {@code perceptronic}), and the last {@link #KEEP} lines kept in memory for a
 * support screen or a test. No UR API.
 */
final class Log {
    static final int KEEP = 50;
    private static final Logger LOGGER = Logger.getLogger("perceptronic");
    private static final List<String> RECENT = new ArrayList<String>();
    private static String last = "";

    private Log() {
    }

    /** One line of detail. The same line twice in a row (a poll that keeps failing) is written once. */
    static synchronized void detail(String text) {
        if (text == null || text.equals(last)) return;
        last = text;
        RECENT.add(text);
        while (RECENT.size() > KEEP) RECENT.remove(0);
        LOGGER.log(Level.WARNING, "Perceptronic: {0}", text);
    }

    static synchronized List<String> recent() {
        return new ArrayList<String>(RECENT);
    }
}
