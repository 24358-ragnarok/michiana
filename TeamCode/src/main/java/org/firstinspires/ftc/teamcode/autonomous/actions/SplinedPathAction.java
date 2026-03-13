package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * A path action that moves the robot along a smooth spline curve to the target pose.
 * <p>
 * This action can either use explicit control points or automatically generate a simple
 * curve using a midpoint. It uses {@link BezierCurve} for smooth motion.
 */
public class SplinedPathAction extends PathAction {

    private final Pose[] controlPoints;

    /**
     * Creates a SplinedPathAction with explicit control points.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param name          A descriptive name for the action.
     * @param isBlue        True for BLUE alliance, false for RED.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    public SplinedPathAction(Pose targetPose, String name, boolean isBlue, Pose... controlPoints) {
        super(targetPose, name, isBlue);
        this.controlPoints = controlPoints;
    }

    /**
     * Creates a SplinedPathAction with explicit control points using global match state.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param name          A descriptive name for the action.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    public SplinedPathAction(Pose targetPose, String name, Pose... controlPoints) {
        super(targetPose, name);
        this.controlPoints = controlPoints;
    }

    /**
     * Creates a SplinedPathAction that auto-generates a control point at the midpoint.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    public SplinedPathAction(Pose targetPose, String name) {
        super(targetPose, name);
        this.controlPoints = null; // Will be auto-generated
    }

    /**
     * Creates a SplinedPathAction with auto-generated control point and default name.
     *
     * @param targetPose The target pose in BLUE alliance coordinates.
     */
    public SplinedPathAction(Pose targetPose) {
        super(targetPose, "SplinedPath");
        this.controlPoints = null; // Will be auto-generated
    }

    @Override
    protected PathChain buildPath(Robot bot, Pose startPose, Pose endPose) {
        BezierCurve curve;

        if (controlPoints != null && controlPoints.length > 0) {
            // Use provided control points, mirroring them if necessary
            Pose[] allPoints = new Pose[controlPoints.length + 2];
            allPoints[0] = startPose;

            for (int i = 0; i < controlPoints.length; i++) {
                allPoints[i + 1] = isBlue
                        ? controlPoints[i]
                        : controlPoints[i].mirror();
            }

            allPoints[allPoints.length - 1] = endPose;
            curve = new BezierCurve(allPoints);
        } else {
            // Auto-generate a midpoint control point for a simple curve
            Pose midPoint = new Pose(
                    (startPose.getX() + endPose.getX()) / 2,
                    (startPose.getY() + endPose.getY()) / 2,
                    (startPose.getHeading() + endPose.getHeading()) / 2);
            curve = new BezierCurve(startPose, midPoint, endPose);
        }

        return bot.dt.follower.pathBuilder()
                .addPath(curve)
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }
}
