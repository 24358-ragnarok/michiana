package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.Arrays;
import java.util.List;

/**
 * An action that executes multiple child actions simultaneously.
 * <p>
 * The parallel action completes only when ALL of its child actions have
 * finished.
 * This is useful for performing multiple independent tasks at once, such as
 * driving
 * while moving an intake or lift.
 */
public class ParallelAction implements AutonomousAction {
    private final List<AutonomousAction> actions;
    private final boolean[] completed;

    /**
     * Creates a new ParallelAction.
     *
     * @param actions The list of actions to run concurrently.
     */
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
                // Execute the child action if it hasn't finished yet
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
            // Stop any actions that were still running
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
