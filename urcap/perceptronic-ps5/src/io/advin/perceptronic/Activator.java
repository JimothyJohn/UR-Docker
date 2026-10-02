package io.advin.perceptronic;

import com.ur.urcap.api.contribution.installation.swing.SwingInstallationNodeService;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeService;
import com.ur.urcap.api.contribution.toolbar.swing.SwingToolbarService;
import org.osgi.framework.BundleActivator;
import org.osgi.framework.BundleContext;

/**
 * Registers the Perceptronic installation node, the 3D Pick program node and the toolbar
 * button (the live picture in a popup) with PolyScope 5.
 */
public class Activator implements BundleActivator {
    @Override
    public void start(BundleContext context) {
        context.registerService(SwingInstallationNodeService.class, new PilotService(), null);
        context.registerService(SwingProgramNodeService.class, new PickService(), null);
        context.registerService(SwingToolbarService.class, new ToolbarService(), null);
    }

    @Override
    public void stop(BundleContext context) {
        // nothing held outside the nodes, which PolyScope closes itself
    }
}
