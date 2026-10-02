package io.advin.perceptronic;

import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.installation.ContributionConfiguration;
import com.ur.urcap.api.contribution.installation.CreationContext;
import com.ur.urcap.api.contribution.installation.InstallationAPIProvider;
import com.ur.urcap.api.contribution.installation.swing.SwingInstallationNodeService;
import com.ur.urcap.api.domain.data.DataModel;
import java.util.Locale;

/** Installation tab → URCaps → Perceptronic (PolyScope X: Application → Perceptronic). */
public class PilotService implements SwingInstallationNodeService<PilotContribution, PilotView> {
    @Override
    public void configureContribution(ContributionConfiguration configuration) {
        // defaults are fine: one node per installation
    }

    @Override
    public String getTitle(Locale locale) {
        return "Perceptronic";
    }

    @Override
    public PilotView createView(ViewAPIProvider apiProvider) {
        return new PilotView(apiProvider);
    }

    @Override
    public PilotContribution createInstallationNode(
            InstallationAPIProvider apiProvider, PilotView view, DataModel model, CreationContext context) {
        return new PilotContribution(apiProvider, view, model);
    }
}
