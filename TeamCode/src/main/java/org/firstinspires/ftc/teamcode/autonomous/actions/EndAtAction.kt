package org.firstinspires.ftc.teamcode.autonomous.actions

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction
import org.firstinspires.ftc.teamcode.hardware.Robot

/**
 * An action that holds the robot at a specific pose indefinitely.
 *
 * This is typically used as the final action in an autonomous sequence to ensure the robot
 * stays parked and resists external forces (like gravity or collisions) until the end of the mode.
 */
class EndAtAction(private val pose: Pose, override val name: String) : AutonomousAction {

    /**
     * Creates a new EndAtAction with a default name.
     *
     * @param pose The pose to hold.
     */
    constructor(pose: Pose) : this(pose, "EndAt")

    override fun initialize(bot: Robot) {
        bot.dt.follower.holdPoint(pose)
    }

    override fun execute(bot: Robot): Boolean {
        // Never complete naturally; must be stopped by the sequence ending
        return false
    }

    override fun end(bot: Robot, interrupted: Boolean) {
        // No cleanup needed; follower will stop when OpMode stops
    }
}
