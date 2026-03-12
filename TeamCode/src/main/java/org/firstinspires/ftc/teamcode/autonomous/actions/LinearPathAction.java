package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * A path action that moves the robot in a straight line to the target pose.
 * <p>
 * Uses a {@link BezierLine} to generate a linear path. Heading interpolation is linear
 * between the start and end headings.
 */
public class LinearPathAction extends PathAction {

    /**
     * Creates a new LinearPathAction.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     * @param isBlue     True for BLUE alliance, false for RED.
     */
    public LinearPathAction(Pose targetPose, String name, boolean isBlue) {
        super(targetPose, name, isBlue);
    }

    /**
     * Creates a new LinearPathAction using the global match state.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    public LinearPathAction(Pose targetPose, String name) {
        super(targetPose, name);
    }

    /**
     * Creates a new LinearPathAction with a default name.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     */
    public LinearPathAction(Pose targetPose) {
        super(targetPose, "LinearPath");
    }

    @Override
    protected PathChain buildPath(Robot bot, Pose startPose, Pose endPose) {
        return bot.dt.follower.pathBuilder()
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }
}
