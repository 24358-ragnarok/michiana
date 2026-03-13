package org.firstinspires.ftc.teamcode.autonomous.actions

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction
import org.firstinspires.ftc.teamcode.config.MatchState
import org.firstinspires.ftc.teamcode.hardware.Robot

/**
 * An initialization action that sets the robot's starting pose.
 *
 * This action should be the first one in any autonomous sequence. It tells the follower
 * where the robot is starting on the field and holds that position to prevent drift
 * before the first movement command.
 */
class StartAtAction(
    private val targetPose: Pose,
    override val name: String,
    private val isBlue: Boolean
) : AutonomousAction {

    /**
     * Creates a new StartAtAction using global match state.
     *
     * @param targetPose The starting pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    constructor(targetPose: Pose, name: String) : this(targetPose, name, MatchState.isBlue)

    /**
     * Creates a new StartAtAction with a default name.
     *
     * @param targetPose The starting pose in BLUE alliance coordinates.
     */
    constructor(targetPose: Pose) : this(targetPose, "StartAt")

    override fun initialize(bot: Robot) {
        val actualPose = if (isBlue) targetPose else targetPose.mirror()

        // Initialize the follower's pose estimate
        bot.dt.follower.setStartingPose(actualPose)
        // Actively hold this position
        bot.dt.follower.holdPoint(actualPose)
    }

    override fun execute(bot: Robot): Boolean {
        // Initialization is instantaneous
        return true
    }

    override fun end(bot: Robot, interrupted: Boolean) {
        // Continue holding position until the next action takes over
    }
}
