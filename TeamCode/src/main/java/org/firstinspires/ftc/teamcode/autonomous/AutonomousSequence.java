package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Manages a sequence of AutonomousActions.
 * Provides a clean, declarative way to define autonomous routines.
 * <p>
 * Features:
 * - Sequential execution of actions
 * - Automatic state management
 * - Progress tracking for telemetry
 * - Graceful handling of action interruption
 */
public class AutonomousSequence {

    public final Timer sequenceTimer;
    private final List<AutonomousAction> actions;
    private final Timer actionTimer;
    private int currentActionIndex;
    private boolean initialized;

    /**
     * Creates a new autonomous sequence.
     */
    public AutonomousSequence() {
        this.actions = new ArrayList<>();
        this.sequenceTimer = new Timer();
        this.actionTimer = new Timer();
        this.currentActionIndex = 0;
        this.initialized = false;
    }

    /**
     * Adds an action to the sequence.
     *
     * @param action The action to add
     * @return this (for method chaining)
     */
    public AutonomousSequence addAction(AutonomousAction action) {
        actions.add(action);
        return this;
    }

    /**
     * Adds multiple actions to the sequence.
     *
     * @param actionsToAdd The actions to add
     * @return this (for method chaining)
     */
    public AutonomousSequence addActions(AutonomousAction... actionsToAdd) {
        actions.addAll(Arrays.asList(actionsToAdd));
        return this;
    }

    /**
     * Starts the sequence. Must be called before update().
     *
     * @param bot The mechanism manager
     */
    public void start(Robot bot) {
        currentActionIndex = 0;
        initialized = false;

        if (!actions.isEmpty()) {
            sequenceTimer.resetTimer();
            actionTimer.resetTimer();
            actions.get(0).initialize(bot);
            initialized = true;
        }
    }

    /**
     * Updates the current action in the sequence.
     * Call this repeatedly in your autonomous loop.
     *
     * @param bot The botmanager
     */
    public void update(Robot bot) {
        // Store elapsed time so actions can access it
        if (isComplete()) {
            return; // No more actions to run
        }

        AutonomousAction currentAction = actions.get(currentActionIndex);

        // Execute the current action
        boolean actionComplete = currentAction.execute(bot);

        // Check for timeout (per-action or global fallback)
        double actionTimeout = currentAction.getTimeoutSeconds();
        boolean actionTimedOut = actionTimeout > 0.0
                && actionTimer.getElapsedTimeSeconds() > actionTimeout;


        boolean robotIsStuck = bot.dt.follower.isRobotStuck();
        boolean shouldEnd = actionComplete || actionTimedOut || robotIsStuck;

        if (shouldEnd) {
            // End the current action
            boolean interrupted = actionTimedOut || robotIsStuck;
            currentAction.end(bot, interrupted);

            // Move to the next action
            currentActionIndex++;
            sequenceTimer.resetTimer();
            actionTimer.resetTimer();

            // Initialize the next action if it exists
            if (currentActionIndex < actions.size()) {
                actions.get(currentActionIndex).initialize(bot);
            }
        }
    }

    /**
     * Stops the sequence immediately, ending the current action.
     *
     * @param bot The botmanager
     */
    public void stop(Robot bot) {
        if (!isComplete() && initialized) {
            actions.get(currentActionIndex).end(bot, true);
        }
    }

    /**
     * @return true if all actions in the sequence have completed
     */
    public boolean isComplete() {
        return currentActionIndex >= actions.size();
    }

    /**
     * @return the index of the current action (0-based)
     */
    public int getCurrentActionIndex() {
        return currentActionIndex;
    }

    /**
     * @return the total number of actions in the sequence
     */
    public int getTotalActions() {
        return actions.size();
    }

    /**
     * @return the name of the current action, or "Complete" if done
     */
    public String getCurrentActionName() {
        if (isComplete()) {
            return "Complete";
        }
        return actions.get(currentActionIndex).getName();
    }

    /**
     * @return progress as a percentage (0-100)
     */
    public double getProgressPercent() {
        if (actions.isEmpty()) {
            return 100.0;
        }
        return (currentActionIndex * 100.0) / actions.size();
    }
}
