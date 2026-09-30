package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.toolbar.ToolbarConfiguration;
import com.ur.urcap.api.contribution.toolbar.ToolbarContext;
import com.ur.urcap.api.contribution.toolbar.swing.SwingToolbarContribution;
import com.ur.urcap.api.contribution.toolbar.swing.SwingToolbarService;
import javax.swing.Icon;

/**
 * The P button in PolyScope's header — the way Robotiq's URCap puts its gripper there —
 * that drops the live picture down over whatever screen is open. In the URCap API since
 * 1.7.0 (PolyScope 5.4), so it needs no compatibility guard.
 */
public class ToolbarService implements SwingToolbarService {
    /** PolyScope's header buttons are about this tall. */
    static final int ICON_PX = 30;
    /** The popup's height (PolyScope fixes its width): the feed at the pendant's proportions plus a line. */
    static final int HEIGHT_PX = 420;

    @Override
    public Icon getIcon() {
        return Logo.icon(ICON_PX);
    }

    @Override
    public void configureContribution(ToolbarConfiguration configuration) {
        configuration.setToolbarHeight(HEIGHT_PX);
    }

    @Override
    public SwingToolbarContribution createToolbar(ToolbarContext context) {
        return new ToolbarContribution(context);
    }
}
