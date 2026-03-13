package org.firstinspires.ftc.teamcode.autonomous.actions

import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction
import org.firstinspires.ftc.teamcode.config.MatchState
import org.firstinspires.ftc.teamcode.hardware.Robot

/**
 * Abstract base class for all path-following autonomous actions.
 *
 * This class handles the common logic for:
 *
 *   - Alliance-specific mirroring of target poses.
 *   - Dynamic path generation from the robot's current position.
 *   - Executing the path following using the robot's drivetrain.
 *
 * Subclasses must implement [buildPath] to define the specific
 * path geometry (e.g., linear, curved, splined).
 */
abstract class PathAction(
    protected val targetPose: Pose,
    override val name: String,
    protected val isBlue: Boolean
) : AutonomousAction {

    private var generatedPath: PathChain? = null

    /**
     * Creates a new PathAction using the global match state for alliance color.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    constructor(targetPose: Pose, name: String) : this(targetPose, name, MatchState.isBlue)

    /**
     * Initializes the path action.
     *
     * Calculates the actual target pose based on alliance color, builds the path from the
     * robot's current position, and commands the follower to start driving.
     *
     * @param bot The robot instance.
     */
    override fun initialize(bot: Robot) {
        val currentPose = bot.dt.follower.pose

        val actualTarget = if (isBlue) targetPose else targetPose.mirror()

        generatedPath = buildPath(bot, currentPose, actualTarget)

        followPath(bot, generatedPath!!)
    }

    /**
     * Executes the path following logic.
     *
     * @param bot The robot instance.
     * @return true if the follower has finished the path, false otherwise.
     */
    override fun execute(bot: Robot): Boolean {
        return !bot.dt.follower.isBusy
    }

    /**
     * Ends the path action.
     *
     * If interrupted, it forces the follower to stop immediately.
     *
     * @param bot         The robot instance.
     * @param interrupted True if the action was interrupted.
     */
    override fun end(bot: Robot, interrupted: Boolean) {
        if (interrupted) {
            bot.dt.follower.breakFollowing()
        }
    }

    /**
     * Constructs the specific path geometry from the start pose to the end pose.
     *
     * @param bot       The robot instance.
     * @param startPose The current pose of the robot.
     * @param endPose   The target pose (already mirrored if necessary).
     * @return The constructed [PathChain].
     */
    protected abstract fun buildPath(bot: Robot, startPose: Pose, endPose: Pose): PathChain

    /**
     * Commands the robot to follow the generated path.
     *
     * Can be overridden by subclasses to apply specific following parameters.
     *
     * @param bot  The robot instance.
     * @param path The path to follow.
     */
    protected open fun followPath(bot: Robot, path: PathChain) {
        bot.dt.follower.followPath(path, true)
    }

    /**
     * Gets the final target pose, accounting for alliance mirroring.
     *
     * @return The actual target pose on the field.
     */
    val finalTargetPose: Pose
        get() = if (isBlue) targetPose else targetPose.mirror()
}
