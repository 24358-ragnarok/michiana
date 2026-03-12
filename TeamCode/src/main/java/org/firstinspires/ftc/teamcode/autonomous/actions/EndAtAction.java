package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * An action that holds the robot at a specific pose indefinitely.
 * <p>
 * This is typically used as the final action in an autonomous sequence to ensure the robot
 * stays parked and resists external forces (like gravity or collisions) until the end of the mode.
 */
public class EndAtAction implements AutonomousAction {
    private final Pose pose;
    private final String name;

	/**
	 * Creates a new EndAtAction.
	 *
	 * @param pose The pose to hold.
	 * @param name A descriptive name for the action.
	 */
    public EndAtAction(Pose pose, String name) {
        this.pose = pose;
        this.name = name;
	}

	/**
	 * Creates a new EndAtAction with a default name.
	 *
	 * @param pose The pose to hold.
	 */
    public EndAtAction(Pose pose) {
        this(pose, "EndAt");
    }

    @Override
    public void initialize(Robot bot) {
        bot.dt.follower.holdPoint(pose);
    }

    @Override
    public boolean execute(Robot bot) {
		// Never complete naturally; must be stopped by the sequence ending
        return false;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
		// No cleanup needed; follower will stop when OpMode stops
    }

    @Override
    public String getName() {
        return name;
    }
}
