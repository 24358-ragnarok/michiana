package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * Represents a single atomic action in an autonomous sequence.
 * <p>
 * This interface follows the Command pattern, allowing for composable and
 * reusable robot behaviors.
 * Actions can be anything from moving the robot, operating a mechanism, or
 * waiting for a condition.
 */
public interface AutonomousAction {

    /**
	 * Called once when the action is first started.
	 * <p>
	 * Use this method to initialize motors, set initial states, reset timers, or
	 * configure
	 * sensors required for the action.
	 *
	 * @param bot The robot instance.
     */
    void initialize(Robot bot);

    /**
     * Called repeatedly while the action is running.
	 * <p>
	 * This method contains the main logic of the action. It should return true when
	 * the action
	 * has completed its task, and false otherwise.
     *
	 * @param bot The robot instance.
	 * @return true if the action is complete, false if it should continue running.
     */
    boolean execute(Robot bot);

    /**
	 * Called once when the action ends, either because it completed normally or was
	 * interrupted.
	 * <p>
	 * Use this method to clean up resources, stop motors, or reset states.
     *
	 * @param bot         The robot instance.
	 * @param interrupted true if the action was interrupted (e.g., by a timeout or
	 *                    stop command),
	 *                    false if it completed normally.
     */
    void end(Robot bot, boolean interrupted);

    /**
	 * Gets a human-readable name for this action.
	 * <p>
	 * Used primarily for telemetry and debugging purposes.
	 *
	 * @return The name of the action.
     */
    default String getName() {
        return this.getClass().getSimpleName();
    }

    /**
	 * Gets the timeout for this action in seconds.
     * <p>
	 * If the action runs longer than this duration, it will be automatically
	 * interrupted.
	 * A value of 0 indicates no timeout (the action runs until it completes
	 * naturally).
     *
	 * @return Timeout in seconds (0 = no timeout).
     */
    default double getTimeoutSeconds() {
        return 0;
    }
}
