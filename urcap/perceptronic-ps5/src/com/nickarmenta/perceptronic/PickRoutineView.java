package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.ContributionProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeView;
import java.awt.BorderLayout;
import javax.swing.BorderFactory;
import javax.swing.JPanel;

/** A line of explanation: the routine is the nodes inside it. */
// Swing components are never serialized here; javac's serial lint does not apply to them
@SuppressWarnings("serial")
public class PickRoutineView implements SwingProgramNodeView<PickRoutineContribution> {
    @Override
    public void buildUI(JPanel host, ContributionProvider<PickRoutineContribution> provider) {
        host.setLayout(new BorderLayout());
        JPanel p = Ui.column();
        p.setBorder(BorderFactory.createEmptyBorder(16, 16, 16, 16));
        p.add(Ui.left(Ui.label("After a pick from this picture point", 18f, true, Ui.INK)));
        p.add(Ui.left(Ui.label("Insert what happens to the part — where to place it — inside this node. It runs only"
                + " when the part came from this picture point.", 13f, false, Ui.MUTED)));
        host.add(p, BorderLayout.NORTH);
    }
}
