package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.Arrays;
import java.util.List;

/**
 * Action that runs multiple actions simultaneously.
 * Completes when all child actions have completed.
 */
public class ParallelAction implements AutonomousAction {
    private final List<AutonomousAction> actions;
    private final boolean[] completed;

    public ParallelAction(AutonomousAction... actions) {
        this.actions = Arrays.asList(actions);
        this.completed = new boolean[this.actions.size()];
    }

    @Override
    public void initialize(Robot bot) {
        Arrays.fill(completed, false);
        for (AutonomousAction action : actions) {
            action.initialize(bot);
        }
    }

    @Override
    public boolean execute(Robot bot) {
        boolean allComplete = true;

        for (int i = 0; i < actions.size(); i++) {
            if (!completed[i]) {
                completed[i] = actions.get(i).execute(bot);
                if (completed[i]) {
                    actions.get(i).end(bot, false);
                }
            }
            allComplete &= completed[i];
        }

        return allComplete;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        if (interrupted) {
            for (int i = 0; i < actions.size(); i++) {
                if (!completed[i]) {
                    actions.get(i).end(bot, true);
                }
            }
        }
    }

    @Override
    public String getName() {
        return "Parallel(" + actions.size() + " actions)";
    }
}
