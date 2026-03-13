package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.actions

import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot

/**
 * A path action that moves the robot in a straight line to the target pose.
 *
 * Uses a [BezierLine] to generate a linear path. Heading interpolation is linear
 * between the start and end headings.
 */
class LinearPathAction : PathAction {

    /**
     * Creates a new LinearPathAction.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     * @param isBlue     True for BLUE alliance, false for RED.
     */
    constructor(targetPose: Pose, name: String, isBlue: Boolean) : super(targetPose, name, isBlue)

    /**
     * Creates a new LinearPathAction using the global match state.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    constructor(targetPose: Pose, name: String) : super(targetPose, name)

    /**
     * Creates a new LinearPathAction with a default name.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     */
    constructor(targetPose: Pose) : super(targetPose, "LinearPath")

    override fun buildPath(bot: Robot, startPose: Pose, endPose: Pose): PathChain {
        return bot.dt.follower.pathBuilder()
            .addPath(BezierLine(startPose, endPose))
            .setLinearHeadingInterpolation(startPose.heading, endPose.heading)
            .build()
    }
}
