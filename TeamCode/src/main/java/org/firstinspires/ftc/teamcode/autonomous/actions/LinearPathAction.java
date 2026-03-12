package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Path action that creates a straight line path from current position to
 * target.
 * Uses BezierLine for direct, efficient movement.
 */
public class LinearPathAction extends PathAction {

    public LinearPathAction(Pose targetPose, String name, boolean isBlue) {
        super(targetPose, name, isBlue);
    }

    public LinearPathAction(Pose targetPose, String name) {
        super(targetPose, name);
    }

    /**
     * Convenience constructor with auto-generated name.
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
