package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * A path action that follows a specific Bezier curve defined by control points.
 * <p>
 * Unlike {@link SplinedPathAction}, this class requires explicit control points to define
 * the curve shape. It is useful for navigating around obstacles or creating complex paths.
 */
public class CurvePathAction extends PathAction {

    private final Pose[] controlPoints;

    /**
     * Creates a CurvePathAction with explicit control points.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param name          A descriptive name for the action.
     * @param isBlue        True for BLUE alliance, false for RED.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    public CurvePathAction(Pose targetPose, String name, boolean isBlue, Pose... controlPoints) {
        super(targetPose, name, isBlue);
        this.controlPoints = controlPoints;
    }

    /**
     * Creates a CurvePathAction with explicit control points using global match state.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param name          A descriptive name for the action.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    public CurvePathAction(Pose targetPose, String name, Pose... controlPoints) {
        super(targetPose, name);
        this.controlPoints = controlPoints;
    }

    /**
     * Creates a CurvePathAction with explicit control points and default name.
     *
     * @param targetPose    The target pose in BLUE alliance coordinates.
     * @param controlPoints Intermediate control points for the curve (in BLUE coordinates).
     */
    public CurvePathAction(Pose targetPose, Pose... controlPoints) {
        super(targetPose, "CurvePath");
        this.controlPoints = controlPoints;
    }

    /**
     * Factory method to create a curve with a single control point (Quadratic Bezier).
     *
     * @param targetPose   The target pose in BLUE alliance coordinates.
     * @param controlPoint The single control point.
     * @param name         A descriptive name for the action.
     * @return A new CurvePathAction instance.
     */
    public static CurvePathAction withSingleControlPoint(Pose targetPose, Pose controlPoint, String name) {
        return new CurvePathAction(targetPose, name, controlPoint);
    }

    @Override
    protected PathChain buildPath(Robot bot, Pose startPose, Pose endPose) {
        Pose[] allPoints = new Pose[controlPoints.length + 2];
        allPoints[0] = startPose;

        // Add control points, mirroring if necessary
        for (int i = 0; i < controlPoints.length; i++) {
            allPoints[i + 1] = isBlue
                    ? controlPoints[i]
                    : controlPoints[i].mirror();
        }

        allPoints[allPoints.length - 1] = endPose;

        BezierCurve curve = new BezierCurve(allPoints);

        return bot.dt.follower.pathBuilder()
                .addPath(curve)
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .build();
    }
}
