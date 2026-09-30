package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.ContributionConfiguration;
import com.ur.urcap.api.contribution.program.CreationContext;
import com.ur.urcap.api.contribution.program.ProgramAPIProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;
import com.ur.urcap.api.domain.data.DataModel;
import java.util.Locale;

/**
 * "After picture N" — the routine a Perceptronic Pick node runs after a pick that came from its
 * picture point N (Options → "A routine per picture point"). The Pick node inserts one per
 * point; the operator never inserts it by hand.
 */
public class PickRoutineService implements SwingProgramNodeService<PickRoutineContribution, PickRoutineView> {
    @Override
    public String getId() {
        return "PerceptronicPickRoutine";
    }

    @Override
    public void configureContribution(ContributionConfiguration configuration) {
        configuration.setChildrenAllowed(true);
        configuration.setUserInsertable(false);
    }

    @Override
    public String getTitle(Locale locale) {
        return "After picture";
    }

    @Override
    public PickRoutineView createView(ViewAPIProvider apiProvider) {
        return new PickRoutineView();
    }

    @Override
    public PickRoutineContribution createNode(
            ProgramAPIProvider apiProvider, PickRoutineView view, DataModel model, CreationContext context) {
        return new PickRoutineContribution(model);
    }
}
