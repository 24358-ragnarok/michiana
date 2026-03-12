package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Represents a single action in an autonomous sequence.
 * Actions are composable, reusable units that encapsulate robot behaviors.
 * <p>
 * This interface follows the Command pattern, allowing for:
 * - Easy testing of individual actions
 * - Clear separation of concerns
 * - Reusability across different autonomous modes
 */
public interface AutonomousAction {

    /**
     * Called once when the action starts.
     * Use this to initialize motors, set initial states, start timers, etc.
     */
    void initialize(Robot bot);

    /**
     * Called repeatedly while the action is running.
     * Use this to update bot, check sensors, etc.
     *
     * @return true if the action is complete, false if it should continue
     */
    boolean execute(Robot bot);

    /**
     * Called once when the action ends (either normally or when interrupted).
     * Use this for cleanup: stop motors, reset states, etc.
     *
     * @param bot         The robot's mechanism manager
     * @param interrupted true if the action was interrupted, false if it completed
     *                    normally
     */
    void end(Robot bot, boolean interrupted);

    /**
     * @return A human-readable name for this action (used for telemetry/debugging)
     */
    default String getName() {
        return this.getClass().getSimpleName();
    }

    /**
     * Returns the timeout for this action in seconds.
     * If 0, no timeout is applied (action runs until it completes naturally).
     * If > 0, the action will be interrupted after this duration.
     * <p>
     *
     * @return Timeout in seconds (0 = no timeout)
     */
    default double getTimeoutSeconds() {
        return 0;
    }
}
