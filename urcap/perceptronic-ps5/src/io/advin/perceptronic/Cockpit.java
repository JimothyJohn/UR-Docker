package io.advin.perceptronic;

import java.awt.image.BufferedImage;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ConnectException;
import java.net.HttpURLConnection;
import java.net.NoRouteToHostException;
import java.net.SocketTimeoutException;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.net.UnknownHostException;
import java.nio.charset.Charset;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.imageio.ImageIO;

/**
 * The RealSense cockpit's HTTP API (``perceptronics gui``), from the controller: the same
 * routes the PolyScope X node calls from the browser — ``GET /api/color.png`` long-polled
 * by sequence number, ``GET /api/point``, ``POST /api/segment`` → ``POST
 * /api/robot/locate``, ``/api/robot/{move,bring_up,stop}``. No UR API here, so it runs
 * (and is tested) on any JDK.
 */
final class Cockpit {
    static final int DEFAULT_PORT = 7621;
    private static final Charset UTF8 = Charset.forName("UTF-8");
    private static final Pattern PORT_ONLY = Pattern.compile("^:?(\\d{1,5})$");
    private static final Pattern HAS_SCHEME = Pattern.compile("^[a-zA-Z][a-zA-Z0-9+.-]*://.*");

    final String base;

    Cockpit(String saved) {
        this.base = base(saved);
    }

    /**
     * The saved field as an absolute base URL — the PolyScope X node's rules, with "this
     * host" being the controller: empty or ":7621" / "7621" → http://127.0.0.1:7621,
     * "host[:port]" → http://host[:port], a bare host gets :7621, a URL is kept.
     */
    static String base(String saved) {
        String raw = saved == null ? "" : saved.trim();
        while (raw.endsWith("/")) raw = raw.substring(0, raw.length() - 1);
        if (raw.isEmpty()) return "http://127.0.0.1:" + DEFAULT_PORT;
        Matcher port = PORT_ONLY.matcher(raw);
        if (port.matches()) return "http://127.0.0.1:" + port.group(1);
        if (HAS_SCHEME.matcher(raw).matches()) return raw;
        String withScheme = "http://" + raw;
        try {
            URI u = new URI(withScheme);
            String path = u.getPath();
            if (u.getHost() != null && u.getPort() < 0 && (path == null || path.isEmpty())) {
                return withScheme + ":" + DEFAULT_PORT;
            }
        } catch (URISyntaxException e) {
            // explain() names the bad value when the request fails
        }
        return withScheme;
    }

    /** One colour frame from ``GET /api/color.png`` (``status`` 503 = no frame yet). */
    static final class Frame {
        final int status;
        final long seq;
        final String fps;
        final BufferedImage image;

        Frame(int status, long seq, String fps, BufferedImage image) {
            this.status = status;
            this.seq = seq;
            this.fps = fps;
            this.image = image;
        }
    }

    Frame colorPng(long after, int timeoutMs) throws IOException {
        return framePng(false, after, timeoutMs);
    }

    /** The next picture: the colour image, or ({@code depth}) the depth as a heatmap — ``GET /api/depth.png``. */
    Frame framePng(boolean depth, long after, int timeoutMs) throws IOException {
        HttpURLConnection c = open("/api/" + (depth ? "depth" : "color") + ".png?after=" + after + "&timeout_ms="
                + timeoutMs, "GET", timeoutMs + 5000);
        try {
            int status = c.getResponseCode();
            if (status != 200) {
                drain(c);
                return new Frame(status, after, null, null);
            }
            long seq = after;
            String h = c.getHeaderField("X-Seq");
            if (h != null) {
                try {
                    seq = Long.parseLong(h.trim());
                } catch (NumberFormatException e) {
                    // keep the previous sequence
                }
            }
            InputStream in = c.getInputStream();
            try {
                BufferedImage img = ImageIO.read(in);
                if (img == null) throw new IOException("the cockpit sent a colour frame that is not an image");
                return new Frame(status, seq, c.getHeaderField("X-Fps"), img);
            } finally {
                in.close();
            }
        } finally {
            c.disconnect();
        }
    }

    Map<String, Object> get(String path, int timeoutMs) throws IOException {
        return call("GET", path, null, timeoutMs);
    }

    Map<String, Object> post(String path, Map<String, Object> body, int timeoutMs) throws IOException {
        return call("POST", path, body, timeoutMs);
    }

    private Map<String, Object> call(String method, String path, Map<String, Object> body, int timeoutMs)
            throws IOException {
        HttpURLConnection c = open(path, method, timeoutMs);
        try {
            if (body != null) {
                byte[] bytes = Json.write(body).getBytes(UTF8);
                c.setDoOutput(true);
                c.setRequestProperty("Content-Type", "application/json");
                c.setFixedLengthStreamingMode(bytes.length);
                OutputStream out = c.getOutputStream();
                try {
                    out.write(bytes);
                } finally {
                    out.close();
                }
            }
            int status = c.getResponseCode();
            String text = readAll(status >= 400 ? c.getErrorStream() : c.getInputStream());
            Map<String, Object> out;
            try {
                out = Json.parseObject(text);
            } catch (IllegalArgumentException e) {
                out = new LinkedHashMap<String, Object>();
                out.put("ok", Boolean.FALSE);
                out.put("error", "HTTP " + status + " from " + base + path);
            }
            if (!out.containsKey("ok")) out.put("ok", Boolean.valueOf(status < 400));
            return out;
        } finally {
            c.disconnect();
        }
    }

    private HttpURLConnection open(String path, String method, int timeoutMs) throws IOException {
        URL url = new URL(base + path);
        HttpURLConnection c = (HttpURLConnection) url.openConnection();
        c.setRequestMethod(method);
        c.setConnectTimeout(3000);
        c.setReadTimeout(Math.max(1000, timeoutMs));
        c.setUseCaches(false);
        return c;
    }

    private static void drain(HttpURLConnection c) {
        try {
            InputStream in = c.getErrorStream();
            if (in != null) in.close();
        } catch (IOException e) {
            // nothing to salvage
        }
    }

    private static String readAll(InputStream in) throws IOException {
        if (in == null) return "";
        try {
            ByteArrayOutputStream buf = new ByteArrayOutputStream();
            byte[] chunk = new byte[8192];
            int n;
            while ((n = in.read(chunk)) > 0) {
                buf.write(chunk, 0, n);
                if (buf.size() > 4 * 1024 * 1024) throw new IOException("reply larger than 4 MB");
            }
            return new String(buf.toByteArray(), UTF8);
        } finally {
            in.close();
        }
    }

    /** What went wrong, for the operator: one plain line and the things to check, in order. */
    static final class Advice {
        final String summary;
        final String[] checks;
        final String detail; // for the log: the exception, the URL, the command

        Advice(String summary, String[] checks, String detail) {
            this.summary = summary;
            this.checks = checks;
            this.detail = detail;
        }

        /** The summary, then one check per line. */
        String text() {
            StringBuilder b = new StringBuilder(summary);
            for (String c : checks) b.append("\n• ").append(c);
            return b.toString();
        }
    }

    static final String CHECK_CABLES = "Cables: the camera computer is powered and its network cable is plugged in at"
            + " both ends (link lights on)";
    static final String CHECK_FIREWALL = "Firewall: the camera computer must let the robot in on TCP ports "
            + DEFAULT_PORT + " and " + PickScript.DEFAULT_PICK_PORT;
    static final String CHECK_USB = "Camera cable: the camera's USB cable seated at both ends, in a blue (USB 3) port"
            + " — unplug it and plug it back in";

    private static String host(String base) {
        try {
            String h = new URI(base).getHost();
            return h == null ? base : h;
        } catch (URISyntaxException e) {
            return base;
        }
    }

    private static String checkAddress(String base) {
        return "IP address: is " + host(base) + " the camera computer's, and on the same network as the robot"
                + " (the robot's own is under Settings → System → Network)?";
    }

    /**
     * Why a request to the camera computer failed, as the operator should read it: what
     * happened in one line, then what to check — cables, the IP address, the firewall — most
     * likely first. The exception itself goes in {@link Advice#detail}, for the log.
     */
    static Advice advise(Exception e, String base) {
        String why = e.getClass().getSimpleName() + (e.getMessage() == null ? "" : ": " + e.getMessage());
        String detail = base + " — " + why + " — the camera computer runs the cockpit with:  perceptronics --cell"
                + " <cell> gui --bind 0.0.0.0  (service: perceptronics-cockpit)";
        boolean self = base.contains("127.0.0.1") || base.contains("localhost");
        if (e instanceof java.net.MalformedURLException || e instanceof IllegalArgumentException) {
            return new Advice("\"" + base + "\" is not an address.",
                    new String[] {"Enter it as  http://<camera computer's IP>:" + DEFAULT_PORT}, detail);
        }
        if (e instanceof UnknownHostException) {
            return new Advice("The robot cannot find a computer called " + host(base) + ".",
                    new String[] {"IP address: enter the camera computer's IP address instead of its name",
                        CHECK_CABLES}, detail);
        }
        if (e instanceof ConnectException) {
            return new Advice(self ? "Nothing on the robot controller itself answers at " + base + "."
                    : "A computer answers at " + host(base) + ", but not the camera program.", new String[] {
                        self ? "IP address: 127.0.0.1 is this robot, not the camera computer — enter that computer's"
                                + " IP address" : checkAddress(base),
                        "Camera program: is it running on the camera computer? Restart that computer if unsure",
                        CHECK_FIREWALL,
                    }, detail + " — connection refused: nothing listens on that port, or the cockpit is bound to"
                            + " loopback only");
        }
        if (e instanceof SocketTimeoutException || e instanceof NoRouteToHostException) {
            return new Advice("No answer from the camera computer at " + host(base) + ".",
                    new String[] {CHECK_CABLES, checkAddress(base), CHECK_FIREWALL}, detail);
        }
        return new Advice("The camera computer at " + host(base) + " cannot be reached.",
                new String[] {CHECK_CABLES, checkAddress(base), CHECK_FIREWALL}, detail);
    }

    /** {@link #advise} as text for a status line; the detail is written to the log. */
    static String explain(Exception e, String base) {
        Advice a = advise(e, base);
        Log.detail(a.detail);
        return a.text();
    }

    /** The camera computer answers but has no picture (HTTP 503): the camera, not the network. */
    static String noPicture(String base, String lastError) {
        Log.detail(base + " — the cockpit is up but has no frame" + (lastError == null ? "" : ": " + lastError));
        return "The camera computer is on, but its camera gives no picture.\n• " + CHECK_USB
                + "\n• The picture comes back by itself a few seconds after the camera does";
    }

    /** ``xs`` as six finite doubles, or null (a JSON list of numbers from the cockpit). */
    static double[] six(Object xs) {
        if (!(xs instanceof List)) return null;
        List<?> l = (List<?>) xs;
        if (l.size() != 6) return null;
        double[] out = new double[6];
        for (int k = 0; k < 6; k++) {
            Object v = l.get(k);
            if (!(v instanceof Number)) return null;
            out[k] = ((Number) v).doubleValue();
            if (Double.isNaN(out[k]) || Double.isInfinite(out[k])) return null;
        }
        return out;
    }

    /** The located-target text under the feed — the PolyScope X node's wording. */
    static String targetText(Map<String, Object> loc) {
        Object reachable = loc.get("reachable");
        String reach = Boolean.FALSE.equals(reachable) ? "OUT OF REACH"
                : Boolean.TRUE.equals(reachable) ? "reachable" : "reach unknown";
        StringBuilder b = new StringBuilder();
        b.append("object  base ").append(fmtVec(loc.get("point_base_m"))).append(" m  (")
                .append(fmt(loc.get("point_distance_m"), 2)).append(" m from the base)\n");
        if ("fingertip".equals(loc.get("reference"))) {
            b.append("fingertips ").append(fmtVec(loc.get("approach_pose"))).append("  ")
                    .append(fmt(loc.get("standoff_m"), 3)).append(" m above the object (tool ")
                    .append(fmt(loc.get("tip_m"), 3)).append(" m)\n");
        } else if ("flange".equals(loc.get("reference")) && loc.get("flange_target_pose") instanceof List) {
            b.append("flange   ").append(fmtVec(loc.get("flange_target_pose"))).append("  standoff ")
                    .append(fmt(loc.get("standoff_m"), 2)).append(" m above the object\n");
        } else {
            b.append("approach ").append(fmtVec(loc.get("approach_pose"))).append("  standoff ")
                    .append(fmt(loc.get("standoff_m"), 2)).append(" m\n");
        }
        b.append(reach);
        if ("controller_ik".equals(loc.get("reach_check"))) {
            b.append("  (the controller's inverse kinematics)");
        } else if (loc.get("max_reach_m") instanceof Number) {
            b.append("  (").append(fmt(loc.get("max_reach_m"), 2)).append(" m datasheet radius");
            if (loc.get("model") != null) b.append(", ").append(loc.get("model"));
            b.append(" — no IK answer)");
        }
        if (six(loc.get("joint_target")) == null && six(loc.get("polyscope_pose")) == null
                && loc.get("polyscope_pose_note") != null) {
            b.append("\nMove (PolyScope) unavailable: ").append(loc.get("polyscope_pose_note"));
        }
        return b.toString();
    }

    static String fmt(Object v, int digits) {
        if (!(v instanceof Number)) return "?";
        return String.format(java.util.Locale.ROOT, "%." + digits + "f", ((Number) v).doubleValue());
    }

    static String fmtVec(Object xs) {
        if (!(xs instanceof List)) return "?";
        StringBuilder b = new StringBuilder();
        for (Object v : (List<?>) xs) {
            if (b.length() > 0) b.append(", ");
            b.append(fmt(v, 3));
        }
        return b.toString();
    }
}
