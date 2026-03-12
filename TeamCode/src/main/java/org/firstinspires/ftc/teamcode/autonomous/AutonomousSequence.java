package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Manages a linear sequence of {@link AutonomousAction}s.
 * <p>
 * This class handles the execution flow of autonomous routines, including:
 * <ul>
 *   <li>Sequential execution of actions.</li>
 *   <li>Automatic state transitions.</li>
 *   <li>Timeout handling for individual actions.</li>
 *   <li>Progress tracking for telemetry.</li>
 * </ul>
 */
public class AutonomousSequence {

    /**
     * Timer for the entire sequence.
     */
    public final Timer sequenceTimer;

    private final List<AutonomousAction> actions;
    private final Timer actionTimer;
    private int currentActionIndex;
    private boolean initialized;

    /**
     * Creates a new, empty autonomous sequence.
     */
    public AutonomousSequence() {
        this.actions = new ArrayList<>();
        this.sequenceTimer = new Timer();
        this.actionTimer = new Timer();
        this.currentActionIndex = 0;
        this.initialized = false;
    }

    /**
     * Adds a single action to the end of the sequence.
     *
     * @param action The action to add.
     * @return This sequence instance (for method chaining).
     */
    public AutonomousSequence addAction(AutonomousAction action) {
        actions.add(action);
        return this;
    }

    /**
     * Adds multiple actions to the end of the sequence.
     *
     * @param actionsToAdd The actions to add.
     * @return This sequence instance (for method chaining).
     */
    public AutonomousSequence addActions(AutonomousAction... actionsToAdd) {
        actions.addAll(Arrays.asList(actionsToAdd));
        return this;
    }

    /**
     * Starts the sequence from the beginning.
     * <p>
     * Resets timers and initializes the first action. This must be called before {@link #update(Robot)}.
     *
     * @param bot The robot instance.
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
     * <p>
     * This method should be called repeatedly in the autonomous loop. It handles:
     * <ul>
     *   <li>Executing the current action.</li>
     *   <li>Checking for completion or timeouts.</li>
     *   <li>Transitioning to the next action.</li>
     * </ul>
     *
     * @param bot The robot instance.
     */
    public void update(Robot bot) {
        if (isComplete()) {
            return; // Sequence is finished
        }

        AutonomousAction currentAction = actions.get(currentActionIndex);

        // Execute the current action logic
        boolean actionComplete = currentAction.execute(bot);

        // Check for timeout
        double actionTimeout = currentAction.getTimeoutSeconds();
        boolean actionTimedOut = actionTimeout > 0.0
                && actionTimer.getElapsedTimeSeconds() > actionTimeout;

        // Check if the robot is stuck (if supported by the drivetrain)
        boolean robotIsStuck = bot.dt.follower.isRobotStuck();

        boolean shouldEnd = actionComplete || actionTimedOut || robotIsStuck;

        if (shouldEnd) {
            // Determine if the end was normal or interrupted
            boolean interrupted = actionTimedOut || robotIsStuck;
            currentAction.end(bot, interrupted);

            // Advance to the next action
            currentActionIndex++;
            sequenceTimer.resetTimer();
            actionTimer.resetTimer();

            // Initialize the next action if available
            if (currentActionIndex < actions.size()) {
                actions.get(currentActionIndex).initialize(bot);
            }
        }
    }

    /**
     * Stops the sequence immediately.
     * <p>
     * Ends the currently running action with the interrupted flag set to true.
     *
     * @param bot The robot instance.
     */
    public void stop(Robot bot) {
        if (!isComplete() && initialized) {
            actions.get(currentActionIndex).end(bot, true);
        }
    }

    /**
     * Checks if the entire sequence has completed.
     *
     * @return true if all actions have finished, false otherwise.
     */
    public boolean isComplete() {
        return currentActionIndex >= actions.size();
    }

    /**
     * Gets the index of the currently executing action.
     *
     * @return The 0-based index of the current action.
     */
    public int getCurrentActionIndex() {
        return currentActionIndex;
    }

    /**
     * Gets the total number of actions in the sequence.
     *
     * @return The total action count.
     */
    public int getTotalActions() {
        return actions.size();
    }

    /**
     * Gets the name of the currently executing action.
     *
     * @return The name of the current action, or "Complete" if the sequence is finished.
     */
    public String getCurrentActionName() {
        if (isComplete()) {
            return "Complete";
        }
        return actions.get(currentActionIndex).getName();
    }

    /**
     * Calculates the progress of the sequence as a percentage.
     *
     * @return The progress percentage (0.0 to 100.0).
     */
    public double getProgressPercent() {
        if (actions.isEmpty()) {
            return 100.0;
        }
        return (currentActionIndex * 100.0) / actions.size();
    }
}
