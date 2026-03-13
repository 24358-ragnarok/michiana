package org.firstinspires.ftc.teamcode.autonomous.actions

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.hardware.Robot

/**
 * A path action that moves the robot along a smooth spline curve to the target pose.
 *
 * This action can either use explicit control points or automatically generate a simple
 * curve using a midpoint. It uses [BezierCurve] for smooth motion.
 */
class SplinedPathAction : PathAction {

    private val controlPoints: Array<out Pose>?

    /**
     * Creates a SplinedPathAction with explicit control points.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param name          A descriptive name for the action.
     * @param isBlue        True for BLUE alliance, false for RED.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    constructor(
        targetPose: Pose,
        name: String,
        isBlue: Boolean,
        vararg controlPoints: Pose
    ) : super(targetPose, name, isBlue) {
        this.controlPoints = controlPoints
    }

    /**
     * Creates a SplinedPathAction with explicit control points using global match state.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param name          A descriptive name for the action.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    constructor(targetPose: Pose, name: String, vararg controlPoints: Pose) : super(
        targetPose,
        name
    ) {
        this.controlPoints = controlPoints
    }

    /**
     * Creates a SplinedPathAction that auto-generates a control point at the midpoint.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    constructor(targetPose: Pose, name: String) : super(targetPose, name) {
        this.controlPoints = null // Will be auto-generated
    }

    /**
     * Creates a SplinedPathAction with auto-generated control point and default name.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     */
    constructor(targetPose: Pose) : super(targetPose, "SplinedPath") {
        this.controlPoints = null // Will be auto-generated
    }

    override fun buildPath(bot: Robot, startPose: Pose, endPose: Pose): PathChain {
        val curve: BezierCurve

        if (controlPoints != null && controlPoints.isNotEmpty()) {
            // Use provided control points, mirroring them if necessary
            val allPoints = Array(controlPoints.size + 2) { Pose() }
            allPoints[0] = startPose

            for (i in controlPoints.indices) {
                allPoints[i + 1] = if (isBlue) controlPoints[i] else controlPoints[i].mirror()
            }

            allPoints[allPoints.size - 1] = endPose
            curve = BezierCurve(*allPoints)
        } else {
            // Auto-generate a midpoint control point for a simple curve
            val midPoint = Pose(
                (startPose.x + endPose.x) / 2,
                (startPose.y + endPose.y) / 2,
                (startPose.heading + endPose.heading) / 2
            )
            curve = BezierCurve(startPose, midPoint, endPose)
        }

        return bot.dt.follower.pathBuilder()
            .addPath(curve)
            .setLinearHeadingInterpolation(startPose.heading, endPose.heading)
            .build()
    }
}
