package org.firstinspires.ftc.teamcode.autonomous

import com.pedropathing.util.Timer
import org.firstinspires.ftc.teamcode.hardware.Robot
import java.util.ArrayList
import java.util.Arrays

/**
 * Manages a linear sequence of [AutonomousAction]s.
 *
 * This class handles the execution flow of autonomous routines, including:
 *
 *   - Sequential execution of actions.
 *   - Automatic state transitions.
 *   - Timeout handling for individual actions.
 *   - Progress tracking for telemetry.
 */
class AutonomousSequence {

    /**
     * Timer for the entire sequence.
     */
    val sequenceTimer: Timer = Timer()

    private val actions: MutableList<AutonomousAction> = ArrayList()
    private val actionTimer: Timer = Timer()
    
    /**
     * Gets the index of the currently executing action.
     *
     * @return The 0-based index of the current action.
     */
    var currentActionIndex: Int = 0
        private set
        
    private var initialized: Boolean = false

    /**
     * Adds a single action to the end of the sequence.
     *
     * @param action The action to add.
     * @return This sequence instance (for method chaining).
     */
    fun addAction(action: AutonomousAction): AutonomousSequence {
        actions.add(action)
        return this
    }

    /**
     * Adds multiple actions to the end of the sequence.
     *
     * @param actionsToAdd The actions to add.
     * @return This sequence instance (for method chaining).
     */
    fun addActions(vararg actionsToAdd: AutonomousAction): AutonomousSequence {
        actions.addAll(Arrays.asList(*actionsToAdd))
        return this
    }

    /**
     * Starts the sequence from the beginning.
     *
     * Resets timers and initializes the first action. This must be called before [update].
     *
     * @param bot The robot instance.
     */
    fun start(bot: Robot) {
        currentActionIndex = 0
        initialized = false

        if (actions.isNotEmpty()) {
            sequenceTimer.resetTimer()
            actionTimer.resetTimer()
            actions[0].initialize(bot)
            initialized = true
        }
    }

    /**
     * Updates the current action in the sequence.
     *
     * This method should be called repeatedly in the autonomous loop. It handles:
     *
     *   - Executing the current action.
     *   - Checking for completion or timeouts.
     *   - Transitioning to the next action.
     *
     * @param bot The robot instance.
     */
    fun update(bot: Robot) {
        if (isComplete) {
            return // Sequence is finished
        }

        val currentAction = actions[currentActionIndex]

        // Execute the current action logic
        val actionComplete = currentAction.execute(bot)

        // Check for timeout
        val actionTimeout = currentAction.timeoutSeconds
        val actionTimedOut = actionTimeout > 0.0 && actionTimer.elapsedTimeSeconds > actionTimeout

        // Check if the robot is stuck (if supported by the drivetrain)
        val robotIsStuck = bot.dt.follower.isRobotStuck

        val shouldEnd = actionComplete || actionTimedOut || robotIsStuck

        if (shouldEnd) {
            // Determine if the end was normal or interrupted
            val interrupted = actionTimedOut || robotIsStuck
            currentAction.end(bot, interrupted)

            // Advance to the next action
            currentActionIndex++
            sequenceTimer.resetTimer()
            actionTimer.resetTimer()

            // Initialize the next action if available
            if (currentActionIndex < actions.size) {
                actions[currentActionIndex].initialize(bot)
            }
        }
    }

    /**
     * Stops the sequence immediately.
     *
     * Ends the currently running action with the interrupted flag set to true.
     *
     * @param bot The robot instance.
     */
    fun stop(bot: Robot) {
        if (!isComplete && initialized) {
            actions[currentActionIndex].end(bot, true)
        }
    }

    /**
     * Checks if the entire sequence has completed.
     *
     * @return true if all actions have finished, false otherwise.
     */
    val isComplete: Boolean
        get() = currentActionIndex >= actions.size

    /**
     * Gets the total number of actions in the sequence.
     *
     * @return The total action count.
     */
    val totalActions: Int
        get() = actions.size

    /**
     * Gets the name of the currently executing action.
     *
     * @return The name of the current action, or "Complete" if the sequence is finished.
     */
    val currentActionName: String
        get() {
            if (isComplete) {
                return "Complete"
            }
            return actions[currentActionIndex].name
        }

    /**
     * Calculates the progress of the sequence as a percentage.
     *
     * @return The progress percentage (0.0 to 100.0).
     */
    val progressPercent: Double
        get() {
            if (actions.isEmpty()) {
                return 100.0
            }
            return (currentActionIndex * 100.0) / actions.size
        }
}
