package io.advin.perceptronic;

/**
 * UR pose math with no UR API — rotation vectors, {@code pose_trans}, {@code pose_inv} — and
 * the pick area a Pick location is taught as: three points touched with the fingertips
 * (the area's corner, a point along its X edge, a point on its far side), the same
 * construction as {@code perceptronics.volume.Surface.from_points}.
 */
final class PoseMath {
    private PoseMath() {
    }

    /** {@code [x, y, z, rx, ry, rz]} → a 4×4 row-major transform. */
    static double[][] matrix(double[] p) {
        double[][] r = rot(p[3], p[4], p[5]);
        return new double[][] {
            {r[0][0], r[0][1], r[0][2], p[0]},
            {r[1][0], r[1][1], r[1][2], p[1]},
            {r[2][0], r[2][1], r[2][2], p[2]},
            {0, 0, 0, 1},
        };
    }

    static double[] pose(double[][] m) {
        double[] rv = rotvec(m);
        return new double[] {m[0][3], m[1][3], m[2][3], rv[0], rv[1], rv[2]};
    }

    /** URScript's {@code pose_trans(a, b)}: b expressed in a's frame, in a's parent. */
    static double[] trans(double[] a, double[] b) {
        return pose(mul(matrix(a), matrix(b)));
    }

    /** URScript's {@code pose_inv}. */
    static double[] inv(double[] a) {
        double[][] m = matrix(a);
        double[][] o = new double[4][4];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) o[i][j] = m[j][i];
        }
        for (int i = 0; i < 3; i++) o[i][3] = -(o[i][0] * m[0][3] + o[i][1] * m[1][3] + o[i][2] * m[2][3]);
        o[3][3] = 1;
        return pose(o);
    }

    /**
     * The fingertips' position when PolyScope reports the active TCP at {@code tcpPose} with
     * {@code tcpOffset} (PositionParameters' pose and TCP offset): flange = tcp ∘ offset⁻¹,
     * fingertips {@code tipM} along the flange's +Z.
     */
    static double[] fingertip(double[] tcpPose, double[] tcpOffset, double tipM) {
        return tipOf(trans(tcpPose, inv(tcpOffset)), tipM);
    }

    /** The fingertips' position, {@code tipM} along the flange's +Z. */
    static double[] tipOf(double[] flange, double tipM) {
        double[] tip = trans(flange, new double[] {0, 0, tipM, 0, 0, 0});
        return new double[] {tip[0], tip[1], tip[2]};
    }

    // -- the arm's nominal geometry ----------------------------------------------------------

    /**
     * UR's published nominal DH table {d[6], a[6]} (m) of each e-Series arm by PolyScope's robot
     * type name — the same rows as {@code perceptronics.armfk.DH} (the UR3e row checked on the
     * cell at 0.84 mm); alpha is {π/2, 0, 0, π/2, −π/2, 0} for all. Null when not in the table.
     */
    static double[][] dh(String robotType) {
        String t = robotType == null ? "" : robotType.toUpperCase(java.util.Locale.ROOT).replaceAll("[^A-Z0-9]", "");
        if (t.endsWith("E")) t = t.substring(0, t.length() - 1);
        if ("UR3".equals(t)) {
            return new double[][] {{0.15185, 0, 0, 0.13105, 0.08535, 0.0921}, {0, -0.24355, -0.2132, 0, 0, 0}};
        }
        if ("UR5".equals(t)) {
            return new double[][] {{0.1625, 0, 0, 0.1333, 0.0997, 0.0996}, {0, -0.425, -0.3922, 0, 0, 0}};
        }
        if ("UR10".equals(t)) {
            return new double[][] {{0.1807, 0, 0, 0.17415, 0.11985, 0.11655}, {0, -0.6127, -0.57155, 0, 0, 0}};
        }
        if ("UR16".equals(t)) {
            return new double[][] {{0.1807, 0, 0, 0.17415, 0.11985, 0.11655}, {0, -0.4784, -0.36, 0, 0, 0}};
        }
        return null;
    }

    private static final double[] ALPHA = {Math.PI / 2, 0, 0, Math.PI / 2, -Math.PI / 2, 0};

    /**
     * The flange pose (base frame) at joints {@code q} (rad) through the nominal DH table — for
     * a PolyScope too old to report the TCP offset with a taught position. Null when the arm
     * isn't in the table or {@code q} isn't six finite numbers.
     */
    static double[] flange(String robotType, double[] q) {
        double[][] t = dh(robotType);
        if (t == null || q == null || q.length != 6) return null;
        for (double x : q) {
            if (Double.isNaN(x) || Double.isInfinite(x)) return null;
        }
        double[][] m = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
        for (int i = 0; i < 6; i++) {
            double ct = Math.cos(q[i]), st = Math.sin(q[i]), ca = Math.cos(ALPHA[i]), sa = Math.sin(ALPHA[i]);
            double d = t[0][i], a = t[1][i];
            double[][] link = {
                {ct, -st * ca, st * sa, a * ct},
                {st, ct * ca, -ct * sa, a * st},
                {0, sa, ca, d},
                {0, 0, 0, 1},
            };
            m = mul(m, link);
        }
        return pose(m);
    }

    /**
     * The plane through three touched points as {@code [pose(6), sizeX, sizeY]} (m): origin at
     * the corner, X toward the second point, Z (the normal) up, the area from the corner to
     * the other two. Null when the points are collinear or coincide.
     */
    static double[] plane(double[] corner, double[] alongX, double[] farSide) {
        double[] x = sub(alongX, corner);
        double lx = norm(x);
        if (lx < 1e-4) return null;
        x = scale(x, 1 / lx);
        double[] n = cross(x, sub(farSide, corner));
        double ln = norm(n);
        if (ln < 1e-6) return null;
        n = scale(n, 1 / ln);
        if (n[2] < 0) n = scale(n, -1);
        double[] y = cross(n, x);
        double sx = dot(sub(alongX, corner), x);
        double sy = dot(sub(farSide, corner), y);
        double[][] m = {
            {x[0], y[0], n[0], corner[0]},
            {x[1], y[1], n[1], corner[1]},
            {x[2], y[2], n[2], corner[2]},
            {0, 0, 0, 1},
        };
        double[] p = pose(m);
        return new double[] {p[0], p[1], p[2], p[3], p[4], p[5], sx, sy};
    }

    /** The plane's tilt from base XY, degrees (the table is level: tilt is teaching error). */
    static double tiltDeg(double[] planePose) {
        double nz = rot(planePose[3], planePose[4], planePose[5])[2][2];
        return Math.toDegrees(Math.acos(Math.max(-1, Math.min(1, Math.abs(nz)))));
    }

    // -- rotation vectors -------------------------------------------------------------------

    static double[][] rot(double rx, double ry, double rz) {
        double th = Math.sqrt(rx * rx + ry * ry + rz * rz);
        if (th < 1e-12) return new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
        double kx = rx / th, ky = ry / th, kz = rz / th;
        double c = Math.cos(th), s = Math.sin(th), v = 1 - c;
        return new double[][] {
            {kx * kx * v + c, kx * ky * v - kz * s, kx * kz * v + ky * s},
            {kx * ky * v + kz * s, ky * ky * v + c, ky * kz * v - kx * s},
            {kx * kz * v - ky * s, ky * kz * v + kx * s, kz * kz * v + c},
        };
    }

    static double[] rotvec(double[][] m) {
        double tr = m[0][0] + m[1][1] + m[2][2];
        double c = Math.max(-1, Math.min(1, (tr - 1) / 2));
        double th = Math.acos(c);
        if (th < 1e-9) return new double[] {0, 0, 0};
        if (Math.PI - th < 1e-4) {
            // near π the antisymmetric part vanishes: take the axis from the diagonal
            double xx = Math.sqrt(Math.max(0, (m[0][0] + 1) / 2));
            double yy = Math.sqrt(Math.max(0, (m[1][1] + 1) / 2));
            double zz = Math.sqrt(Math.max(0, (m[2][2] + 1) / 2));
            if (xx >= yy && xx >= zz) {
                yy = (m[0][1] + m[1][0]) / (4 * xx);
                zz = (m[0][2] + m[2][0]) / (4 * xx);
            } else if (yy >= zz) {
                xx = (m[0][1] + m[1][0]) / (4 * yy);
                zz = (m[1][2] + m[2][1]) / (4 * yy);
            } else {
                xx = (m[0][2] + m[2][0]) / (4 * zz);
                yy = (m[1][2] + m[2][1]) / (4 * zz);
            }
            double n = Math.sqrt(xx * xx + yy * yy + zz * zz);
            return new double[] {th * xx / n, th * yy / n, th * zz / n};
        }
        double s = 2 * Math.sin(th);
        return new double[] {
            th * (m[2][1] - m[1][2]) / s, th * (m[0][2] - m[2][0]) / s, th * (m[1][0] - m[0][1]) / s,
        };
    }

    private static double[][] mul(double[][] a, double[][] b) {
        double[][] o = new double[4][4];
        for (int i = 0; i < 4; i++) {
            for (int j = 0; j < 4; j++) {
                double s = 0;
                for (int k = 0; k < 4; k++) s += a[i][k] * b[k][j];
                o[i][j] = s;
            }
        }
        return o;
    }

    static double[] sub(double[] a, double[] b) {
        return new double[] {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
    }

    static double dot(double[] a, double[] b) {
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    }

    static double[] cross(double[] a, double[] b) {
        return new double[] {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    }

    static double norm(double[] a) {
        return Math.sqrt(dot(a, a));
    }

    static double[] scale(double[] a, double k) {
        return new double[] {a[0] * k, a[1] * k, a[2] * k};
    }
}
