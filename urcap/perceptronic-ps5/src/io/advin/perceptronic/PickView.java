package io.advin.perceptronic;

import com.ur.urcap.api.contribution.ContributionProvider;
import com.ur.urcap.api.contribution.ViewAPIProvider;
import com.ur.urcap.api.contribution.program.swing.SwingProgramNodeView;
import java.awt.BorderLayout;
import javax.swing.JLabel;
import javax.swing.JPanel;

/**
 * The 3D Pick node's screen: {@link PickScreen} (the live picture, the picture points, the
 * pick order, the Options view) in PolyScope's panel. One
 * view serves every Pick node in the program, so every action goes to
 * {@code provider.get()} — the node that is open.
 */
public class PickView implements SwingProgramNodeView<PickContribution> {
    private final ViewAPIProvider api;
    private ContributionProvider<PickContribution> provider;
    private PickScreen screen;

    PickView(ViewAPIProvider api) {
        this.api = api;
    }

    @Override
    public void buildUI(JPanel host, ContributionProvider<PickContribution> contributionProvider) {
        this.provider = contributionProvider;
        // PolyScope's own panel refuses most setters (setBorder throws): lay it out, add one panel we own
        host.setLayout(new BorderLayout());
        screen = new PickScreen(new Forward());
        host.add(screen, BorderLayout.CENTER);
    }

    PickScreen screen() {
        return screen;
    }

    /** Every tap goes to the node that is open now. */
    private final class Forward implements PickScreen.Actions {
        private PickContribution node() {
            return provider.get();
        }

        @Override
        public void addPoint() {
            node().addPoint();
        }

        @Override
        public void goTo(int i) {
            node().goTo(i);
        }

        @Override
        public void retake(int i) {
            node().retake(i);
        }

        @Override
        public void remove(int i) {
            node().remove(i);
        }

        @Override
        public void select(int i) {
            node().select(i);
        }

        @Override
        public void cycleArea(int i) {
            node().cycleArea(i);
        }

        @Override
        public void setOrder(String first, String rows) {
            node().setOrder(first, rows);
        }

        @Override
        public void setNumber(String key, double value) {
            node().setNumber(key, value);
        }

        @Override
        public void askNumber(String key, JLabel anchor) {
            node().askNumber(key, anchor);
        }

        @Override
        public void setShape(String shape) {
            node().setShape(shape);
        }

        @Override
        public void setDepthView(boolean on) {
            node().setDepthView(on);
        }

        @Override
        public void setFlag(String key, boolean on) {
            node().setFlag(key, on);
        }

        @Override
        public void checkApproach() {
            node().checkApproach();
        }

        @Override
        public void resetDefaults() {
            node().resetDefaults();
        }
    }
}
