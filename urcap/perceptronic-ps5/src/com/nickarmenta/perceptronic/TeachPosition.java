package com.nickarmenta.perceptronic;

import com.ur.urcap.api.domain.userinteraction.UserInteraction;
import com.ur.urcap.api.domain.value.Pose;
import com.ur.urcap.api.domain.value.jointposition.JointPosition;
import com.ur.urcap.api.domain.value.jointposition.JointPositions;
import com.ur.urcap.api.domain.value.simple.Angle;

/**
 * PolyScope's own move screen: the operator puts the arm somewhere and taps OK; the node gets
 * the joints and the <b>flange</b> pose there — on every PolyScope 5 this URCap supports.
 *
 * <p>PolyScope 5.8+ ({@code RobotPositionCallback2}) says which TCP offset
 * its pose is under, so the flange is exact: {@link TeachPosition2}, compiled against the 5.8
 * API ({@code compat.since.5.8} in bundle.properties) and only ever loaded by name, after
 * {@link #modern()} found that class on this PolyScope. Older PolyScopes' callback gives the
 * pose under the active TCP with no offset, so the flange comes from the joints through the
 * arm's nominal geometry ({@link PoseMath#flange}); an arm the table doesn't know gets none.
 */
final class TeachPosition {
    private TeachPosition() {
    }

    /** What the operator taught. {@code flange} is null when this PolyScope can't say where it is. */
    interface Done {
        void taught(JointPositions joints, double[] flange);
    }

    /** One way of asking PolyScope for the position. */
    interface Teacher {
        void teach(UserInteraction ui, Done done);
    }

    /** PolyScope 5.8+: the callback that carries the TCP offset. */
    static final String CALLBACK2 = "com.ur.urcap.api.domain.userinteraction.RobotPositionCallback2";
    static final String MODERN = "com.nickarmenta.perceptronic.TeachPosition2";

    /** The best teacher this PolyScope runs; {@code robotType} names the arm for the old path. */
    static Teacher teacher(String robotType) {
        Teacher t = modern();
        return t != null ? t : new Legacy(robotType);
    }

    /** {@link TeachPosition2} when this PolyScope has {@link #CALLBACK2}, else null. */
    static Teacher modern() {
        return load(CALLBACK2, MODERN, TeachPosition.class.getClassLoader());
    }

    /**
     * {@code impl}, instantiated, if {@code feature} loads through {@code loader} — the bundle's
     * class loader, which sees exactly the URCap API this PolyScope exports. Any failure (the
     * class is absent, or {@code impl} can't link against this API) means: not available.
     */
    static Teacher load(String feature, String impl, ClassLoader loader) {
        try {
            Class.forName(feature, false, loader);
            return (Teacher) Class.forName(impl, true, loader).getDeclaredConstructor().newInstance();
        } catch (ReflectiveOperationException | LinkageError | ClassCastException e) {
            return null;
        }
    }

    /** Radians of every joint. */
    static double[] radians(JointPositions q) {
        JointPosition[] all = q.getAllJointPositions();
        double[] out = new double[all.length];
        for (int i = 0; i < all.length; i++) out[i] = all[i].getPosition(Angle.Unit.RAD);
        return out;
    }

    /** PolyScope before 5.8: the original callback, the flange from the joints. */
    static final class Legacy implements Teacher {
        private final String robotType;

        Legacy(String robotType) {
            this.robotType = robotType;
        }

        @Override
        @SuppressWarnings("deprecation") // deprecated once RobotPositionCallback2 came; the only one before
        public void teach(UserInteraction ui, final Done done) {
            // named in full: Java 8 warns on the import of a deprecated class, which no annotation reaches
            ui.getUserDefinedRobotPosition(new com.ur.urcap.api.domain.userinteraction.RobotPositionCallback() {
                @Override
                public void onOk(Pose pose, JointPositions q) {
                    done.taught(q, PoseMath.flange(robotType, radians(q)));
                }
            });
        }
    }
}
