package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.config.Settings.Logging.INTERVAL;
import static org.firstinspires.ftc.teamcode.config.Settings.Logging.followerLook;
import static org.firstinspires.ftc.teamcode.config.Settings.Logging.robotLook;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Settings;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * UnifiedLogging utilizing JoinedTelemetry for automatic DS/Panels synchronization.
 */
public class Logging {
    public final Telemetry log;
    private FieldRenderer fieldRenderer;
    private final Map<String, Telemetry.Item> itemCache = new HashMap<>();
    private final Map<String, Telemetry.Line> lineCache = new HashMap<>();
    private boolean retainedMode = false;

    public Logging(Telemetry telemetry) {
        // JoinedTelemetry automatically bridges DS and Panels
        this.log = new JoinedTelemetry(PanelsTelemetry.INSTANCE.getFtcTelemetry(), telemetry);
        Drawing.init();

        log.setAutoClear(false);
        log.setMsTransmissionInterval(INTERVAL);
        log.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        log.setItemSeparator("");
        log.setCaptionValueSeparator("");
        log.clearAll();
        log.update();
    }

    public void finishSetup() {
        this.fieldRenderer = new FieldRenderer(log);
    }


    /**
     * Replaces or creates a persistent item.
     * Use this for high-priority blocks (like the Dashboard) to keep them at the top.
     */
    public void setItem(String key, String content) {
        Telemetry.Item item = itemCache.get(key);
        if (item == null) {
            item = log.addData(key, content);
            item.setRetained(true);
            itemCache.put(key, item);
        } else {
            item.setValue(content);
        }
    }

    public Telemetry.Line addLabeledLine(String label) {
        Telemetry.Line line = lineCache.get(label);
        if (line == null) {
            line = log.addLine(label);
            lineCache.put(label, line);
        }
        return line;
    }

    public void addLine(LogLine line) {
        log.addLine(line.getHtml());
    }

    public void addLine(String line) {
        log.addLine(line);
    }

    public void addData(String key, Object value) {
        Telemetry.Item item = itemCache.get(key);
        if (item == null) {
            item = log.addData(key, value);
            if (retainedMode) {
                item.setRetained(true);
                itemCache.put(key, item);
            }
        } else {
            item.setValue(value);
        }
    }

    public void addNumber(String key, double value) {
        addData(key, String.format(Locale.US, "%.2f", value));
    }

    public void listen(String key, Func<?> valueProducer) {
        Telemetry.Item item = itemCache.get(key);
        if (item != null) {
            log.removeItem(item);
        }
        item = log.addData(key, valueProducer);
        item.setRetained(true);
        itemCache.put(key, item);
    }

    public void enableRetainedMode() {
        this.retainedMode = true;
    }

    public void clearDynamic() {
        log.clear();
    }

    public void clearAll() {
        log.clearAll();
        itemCache.clear();
        lineCache.clear();
    }

    public void drawRobot(Pose pose) {
        Drawing.drawRobot(pose);
    }

    public void drawDebug(Follower follower) {
        Drawing.drawDebug(follower);
    }

    public void drawPath(PathChain path) {
        Drawing.drawPath(path);
    }

    public void update(Pose robotPose) {
        Drawing.update();
        log.update();
        if (Settings.Logging.DRAW_FIELD) {
            fieldRenderer.render(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), robotLook.getOutlineFill());
        }
    }
}

class Drawing {
    public static final double ROBOT_RADIUS = (Settings.Dimensions.WIDTH + Settings.Dimensions.LENGTH) / 4;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        update();
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashboardDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPathChain() != null) {
            drawPath(follower.getCurrentPathChain(), followerLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), followerLook);
        } else if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), followerLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), followerLook);
        }
        drawPoseHistory(follower.getPoseHistory(), robotLook);
        drawRobot(follower.getPose(), robotLook);
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    @SuppressWarnings("MethodWithMultipleReturnPoints")
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        for (int i = 0; i < points[0].length - 1; i++) {
            panelsField.moveCursor(points[0][i], points[1][i]);
            panelsField.line(points[0][i + 1], points[1][i + 1]);
        }
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws a path chain.
     *
     * @param pathChain the PathChain to draw
     */
    public static void drawPath(PathChain pathChain) {
        drawPath(pathChain, robotLook);
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    @SuppressWarnings("unused")
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, robotLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void update() {
        panelsField.update();
    }
}