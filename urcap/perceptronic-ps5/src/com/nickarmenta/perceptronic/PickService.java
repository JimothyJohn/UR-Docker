package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.ContributionConfiguration;
import com.ur.urcap.api.contribution.program.CreationContext;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;
import com.ur.urcap.api.domain.data.DataModel;
import java.util.Locale;

/**
 * Program tab → URCaps → Perceptronic Pick: look from the picture points, find the part by its
 * size, pick the next one in order. Its children are the routine after the pick (or, with
 * the gripper set to "my own nodes", the gripper's Close, run at the grip).
 */
public class PickService implements SwingProgramNodeService<PickContribution, PickView> {
    @Override
    public String getId() {
        return "PerceptronicPick";
    }

    @Override
    public void configureContribution(ContributionConfiguration configuration) {
        configuration.setChildrenAllowed(true);
        configuration.setUserInsertable(true);
    }

    @Override
    public String getTitle(Locale locale) {
        return "Perceptronic Pick";
    }

    @Override
    public PickView createView(ViewAPIProvider apiProvider) {
        return new PickView(apiProvider);
    }

    @Override
    public PickContribution createNode(
            ProgramAPIProvider apiProvider, PickView view, DataModel model, CreationContext context) {
        return new PickContribution(apiProvider, view, model, context);
    }
}
