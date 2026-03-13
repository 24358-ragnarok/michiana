package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.actions

import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot

/**
 * A path action that follows a specific Bezier curve defined by control points.
 *
 * Unlike [SplinedPathAction], this class requires explicit control points to define
 * the curve shape. It is useful for navigating around obstacles or creating complex paths.
 */
class CurvePathAction : PathAction {

    private val controlPoints: Array<out Pose>

    /**
     * Creates a CurvePathAction with explicit control points.
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
     * Creates a CurvePathAction with explicit control points using global match state.
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
     * Creates a CurvePathAction with explicit control points and default name.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    constructor(targetPose: Pose, vararg controlPoints: Pose) : super(targetPose, "CurvePath") {
        this.controlPoints = controlPoints
    }

    override fun buildPath(bot: Robot, startPose: Pose, endPose: Pose): PathChain {
        val allPoints = Array(controlPoints.size + 2) { Pose() }
        allPoints[0] = startPose

        // Add control points, mirroring if necessary
        for (i in controlPoints.indices) {
            allPoints[i + 1] = if (isBlue) controlPoints[i] else controlPoints[i].mirror()
        }

        allPoints[allPoints.size - 1] = endPose

        val curve = BezierCurve(*allPoints)

        return bot.dt.follower.pathBuilder()
            .addPath(curve)
            .setLinearHeadingInterpolation(startPose.heading, endPose.heading)
            .build()
    }

    companion object {
        /**
         * Factory method to create a curve with a single control point (Quadratic Bezier).
         *
         * @param targetPose   The target pose in BLUE alliance coordinates.
         * @param controlPoint The single control point.
         * @param name         A descriptive name for the action.
         * @return A new CurvePathAction instance.
         */
        @JvmStatic
        fun withSingleControlPoint(
            targetPose: Pose,
            controlPoint: Pose,
            name: String
        ): CurvePathAction {
            return CurvePathAction(targetPose, name, controlPoint)
        }
    }
}
