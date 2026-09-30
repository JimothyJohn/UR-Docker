package com.nickarmenta.perceptronic;

import com.ur.urcap.api.contribution.ProgramNodeContribution;
import com.ur.urcap.api.domain.data.DataModel;
import com.ur.urcap.api.domain.script.ScriptWriter;
import com.ur.urcap.api.domain.variable.Variable;

/** "After picture N": its children run only when the pick came from picture point N. */
public class PickRoutineContribution implements ProgramNodeContribution {
    static final String KEY_POINT = "point";
    static final String KEY_VARIABLE = "locVariable";

    private final DataModel model;

    PickRoutineContribution(DataModel model) {
        this.model = model;
    }

    /** Called once by the Pick node that inserts it. */
    void init(int point, Variable locVariable) {
        model.set(KEY_POINT, point);
        if (locVariable != null) model.set(KEY_VARIABLE, locVariable);
    }

    int point() {
        return model.get(KEY_POINT, 0);
    }

    @Override
    public void openView() {
    }

    @Override
    public void closeView() {
    }

    @Override
    public String getTitle() {
        return "After picture " + point();
    }

    @Override
    public boolean isDefined() {
        return point() >= 1;
    }

    @Override
    public void generateScript(ScriptWriter writer) {
        Variable v = model.get(KEY_VARIABLE, (Variable) null);
        String name = v != null ? writer.getResolvedVariableName(v) : PickContribution.LOC_NAME;
        writer.appendLine("if " + name + " == " + point() + ":");
        writer.writeChildren();
        writer.appendLine("end");
    }
}
