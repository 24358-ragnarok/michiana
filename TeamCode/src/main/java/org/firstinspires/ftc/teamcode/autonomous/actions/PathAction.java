package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * Abstract base class for all path-following autonomous actions.
 * <p>
 * This class handles the common logic for:
 * <ul>
 *   <li>Alliance-specific mirroring of target poses.</li>
 *   <li>Dynamic path generation from the robot's current position.</li>
 *   <li>Executing the path following using the robot's drivetrain.</li>
 * </ul>
 * Subclasses must implement {@link #buildPath(Robot, Pose, Pose)} to define the specific
 * path geometry (e.g., linear, curved, splined).
 */
public abstract class PathAction implements AutonomousAction {
    protected final Pose targetPose;
    protected final String name;
    protected final boolean isBlue;

    private PathChain generatedPath;

    /**
     * Creates a new PathAction.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     * @param isBlue     True if the robot is on the BLUE alliance, false for RED.
     */
    public PathAction(Pose targetPose, String name, boolean isBlue) {
        this.targetPose = targetPose;
        this.name = name;
        this.isBlue = isBlue;
    }

    /**
     * Creates a new PathAction using the global match state for alliance color.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    public PathAction(Pose targetPose, String name) {
        this(targetPose, name, MatchState.isBlue);
    }

    /**
     * Initializes the path action.
     * <p>
     * Calculates the actual target pose based on alliance color, builds the path from the
     * robot's current position, and commands the follower to start driving.
     *
     * @param bot The robot instance.
     */
    @Override
    public void initialize(Robot bot) {
        Pose currentPose = bot.dt.follower.getPose();

        Pose actualTarget = isBlue
                ? targetPose
                : targetPose.mirror();

        generatedPath = buildPath(bot, currentPose, actualTarget);

        followPath(bot, generatedPath);
    }

    /**
     * Executes the path following logic.
     *
     * @param bot The robot instance.
     * @return true if the follower has finished the path, false otherwise.
     */
    @Override
    public boolean execute(Robot bot) {
        return !bot.dt.follower.isBusy();
    }

    /**
     * Ends the path action.
     * <p>
     * If interrupted, it forces the follower to stop immediately.
     *
     * @param bot         The robot instance.
     * @param interrupted True if the action was interrupted.
     */
    @Override
    public void end(Robot bot, boolean interrupted) {
        if (interrupted) {
            bot.dt.follower.breakFollowing();
        }
    }

    @Override
    public String getName() {
        return name;
    }

    /**
     * Constructs the specific path geometry from the start pose to the end pose.
     *
     * @param bot       The robot instance.
     * @param startPose The current pose of the robot.
     * @param endPose   The target pose (already mirrored if necessary).
     * @return The constructed {@link PathChain}.
     */
    protected abstract PathChain buildPath(Robot bot, Pose startPose, Pose endPose);

    /**
     * Commands the robot to follow the generated path.
     * <p>
     * Can be overridden by subclasses to apply specific following parameters.
     *
     * @param bot  The robot instance.
     * @param path The path to follow.
     */
    protected void followPath(Robot bot, PathChain path) {
        bot.dt.follower.followPath(path, true);
    }

    /**
     * Gets the final target pose, accounting for alliance mirroring.
     *
     * @return The actual target pose on the field.
     */
    public Pose getFinalTargetPose() {
        return isBlue
                ? targetPose
                : targetPose.mirror();
    }
}
