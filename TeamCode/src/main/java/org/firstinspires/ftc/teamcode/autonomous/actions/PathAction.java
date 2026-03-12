package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Abstract base class for all path-following actions.
 * Handles dynamic path building from current robot position to target poses,
 * with automatic alliance mirroring and different path types.
 * <p>
 * This replaces the rigid PathRegistry system with flexible, composable
 * actions.
 */
public abstract class PathAction implements AutonomousAction {
    protected final Pose targetPose;
    protected final String name;
    protected final boolean isBlue;

    private PathChain generatedPath;

    /**
     * Creates a new path action.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates)
     * @param name       Human-readable name for telemetry
     * @param isBlue     The alliance color for automatic mirroring
     */
    public PathAction(Pose targetPose, String name, boolean isBlue) {
        this.targetPose = targetPose;
        this.name = name;
        this.isBlue = isBlue;
    }

    /**
     * Convenience constructor that extracts alliance from MatchState.
     */
    public PathAction(Pose targetPose, String name) {
        this(targetPose, name, MatchState.isBlue);
    }

    @Override
    public void initialize(Robot bot) {
        // Get current robot position
        Pose currentPose = bot.dt.follower.getPose();

        // Mirror target pose if we're on RED alliance
        Pose actualTarget = isBlue
                ? targetPose
                : targetPose.mirror();

        // Build the path dynamically from current position to target
        generatedPath = buildPath(bot, currentPose, actualTarget);

        // Start following the path
        followPath(bot, generatedPath);
    }

    @Override
    public boolean execute(Robot bot) {
        // Action is complete when the follower is no longer busy
        return !bot.dt.follower.isBusy();
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        // Path following will naturally stop when complete
        // If interrupted, break
        if (interrupted) {
            bot.dt.follower.breakFollowing();
        }
    }

    @Override
    public String getName() {
        return name;
    }

    /**
     * Builds the actual path from start to end pose.
     * Subclasses implement this to create different path types (linear, curved,
     * etc.).
     *
     * @param bot       The mechanism manager
     * @param startPose The starting pose (robot's current position)
     * @param endPose   The target pose (already mirrored for alliance)
     * @return The constructed PathChain
     */
    protected abstract PathChain buildPath(Robot bot, Pose startPose, Pose endPose);

    /**
     * Starts following the generated path.
     * Subclasses can override this to modify following behavior (e.g., slow speed).
     *
     * @param bot  The mechanism manager
     * @param path The path to follow
     */
    protected void followPath(Robot bot, PathChain path) {
        bot.dt.follower.followPath(path, true);
    }

    /**
     * Gets the final target pose (after alliance mirroring).
     * Useful for debugging and telemetry.
     */
    public Pose getFinalTargetPose() {
        return isBlue
                ? targetPose
                : targetPose.mirror();
    }
}
