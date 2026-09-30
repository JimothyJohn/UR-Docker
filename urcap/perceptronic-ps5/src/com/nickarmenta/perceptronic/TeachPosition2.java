package com.nickarmenta.perceptronic;

import com.ur.urcap.api.domain.userinteraction.RobotPositionCallback2;
import com.ur.urcap.api.domain.userinteraction.UserInteraction;
import com.ur.urcap.api.domain.value.robotposition.PositionParameters;
import com.ur.urcap.api.domain.value.simple.Angle;
import com.ur.urcap.api.domain.value.simple.Length;

/**
 * PolyScope 5.8+: the taught position with the TCP offset its pose is under,
 * so the flange is exact — flange = tcp ∘ offset⁻¹. Compiled against the 5.8 API
 * ({@code compat.since.5.8}); nothing refers to this class but {@link TeachPosition#modern()},
 * by name, so an older PolyScope never loads it.
 */
final class TeachPosition2 implements TeachPosition.Teacher {
    @Override
    public void teach(UserInteraction ui, final TeachPosition.Done done) {
        ui.getUserDefinedRobotPosition(new RobotPositionCallback2() {
            @Override
            public void onOk(PositionParameters position) {
                double[] tcp = position.getPose().toArray(Length.Unit.M, Angle.Unit.RAD);
                double[] off = position.getTCPOffset().toArray(Length.Unit.M, Angle.Unit.RAD);
                done.taught(position.getJointPositions(), PoseMath.trans(tcp, PoseMath.inv(off)));
            }
        });
    }
}
