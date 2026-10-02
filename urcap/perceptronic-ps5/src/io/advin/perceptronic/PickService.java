package io.advin.perceptronic;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.ContributionConfiguration;
import com.ur.urcap.api.contribution.program.CreationContext;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;
import com.ur.urcap.api.domain.data.DataModel;
import java.util.Locale;

/**
 * Program tab → URCaps → 3D Pick: one move sequence, from the survey at the picture points to
 * the gripper clamped on the next part in order. It has no children (0.7.0): what happens to
 * the part is the program's next nodes. The service id keeps its 0.6.0 name so a saved
 * program still finds its node.
 */
public class PickService implements SwingProgramNodeService<PickContribution, PickView> {
    @Override
    public String getId() {
        return "PerceptronicPick";
    }

    @Override
    public void configureContribution(ContributionConfiguration configuration) {
        configuration.setChildrenAllowed(false);
        configuration.setUserInsertable(true);
    }

    @Override
    public String getTitle(Locale locale) {
        return "3D Pick";
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
