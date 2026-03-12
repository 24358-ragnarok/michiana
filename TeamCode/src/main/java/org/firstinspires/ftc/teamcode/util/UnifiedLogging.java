package org.firstinspires.ftc.teamcode.util;

import static org.firstinspires.ftc.teamcode.config.Constants.Logging.INTERVAL;
import static org.firstinspires.ftc.teamcode.config.Constants.Logging.followerLook;
import static org.firstinspires.ftc.teamcode.config.Constants.Logging.robotLook;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.Constants;

import java.util.HashMap;
import java.util.Locale;
import java.util.Map;

/**
 * UnifiedLogging allows us to send data to both the Driver Station and the
 * Panels Webview
 * simultaneously, and handles all the messy stuff like formatting and Field
 * Drawings.
 * <p>
 * Optimizations:
 * - Uses retained telemetry items (setAutoClear(false)) to avoid rebuilding
 * each frame
 * - Caches Telemetry.Item references for efficient updates
 * - Uses Func suppliers for expensive operations that are only evaluated when
 * transmission occurs
 * - Optimized transmission interval to reduce overhead
 */
@SuppressWarnings({"ClassHasNoToStringMethod", "ClassWithoutNoArgConstructor", "unused"})
public class UnifiedLogging {
    public final Telemetry driverStation;
    public final TelemetryManager panels;

    // Cache telemetry items for efficient updates
    private final Map<String, Telemetry.Item> itemCache = new HashMap<>();
    private final Map<String, Telemetry.Line> lineCache = new HashMap<>();

    // Track items that should be cleared each frame (non-retained)
    private boolean retainedMode = false;

    public UnifiedLogging(Telemetry telemetry, TelemetryManager panels) {
        this.driverStation = telemetry;
        this.panels = panels;
        Drawing.init();

        // Optimize telemetry settings
        driverStation.setAutoClear(false); // We'll manage clearing manually
        driverStation.setMsTransmissionInterval(INTERVAL);
        driverStation.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        driverStation.setItemSeparator("");
        driverStation.setCaptionValueSeparator("");
    }

    /**
     * Enable retained mode - items persist across update() calls.
     * Call this once during init to set up persistent telemetry items.
     */
    public void enableRetainedMode() {
        retainedMode = true;
    }

    /**
     * Adds a labeled line of text. HTML stripped for Panels.
     */
    public Telemetry.Line addLabeledLine(String label) {
        Telemetry.Line line = lineCache.get(label);
        if (line == null) {
            line = driverStation.addLine(label);
            lineCache.put(label, line);
        }
        panels.addLine(TextFormat.stripHtml(label));
        return line;
    }

    /**
     * Overload to natively accept fluent LogLines.
     */
    public void addLine(LogLine line) {
        driverStation.addLine(line.getHtml());
        panels.addLine(line.getPlainText());
    }

    /**
     * Adds a line of text. HTML tags are sent to DS, but stripped for Panels.
     */
    public void addLine(String line) {
        driverStation.addLine(line);
        panels.addLine(TextFormat.stripHtml(line));
    }


    /**
     * Adds or updates a data item. HTML stripped for Panels if value is a String.
     */
    public void addData(String key, Object value) {
        Telemetry.Item item = itemCache.get(key);
        if (item == null) {
            item = driverStation.addData(key, value);
            if (retainedMode) {
                item.setRetained(true);
                itemCache.put(key, item);
            }
        } else {
            item.setValue(value);
        }

        // Strip HTML if the object is a formatted string
        Object panelValue = (value instanceof String) ? TextFormat.stripHtml((String) value) : value;
        panels.addData(TextFormat.stripHtml(key), panelValue);
    }

    /**
     * Adds or updates a formatted number. Uses cached items for efficiency.
     */
    public void addNumber(String key, double value) {
        Telemetry.Item item = itemCache.get(key);
        if (item == null) {
            item = driverStation.addData(key, "%.2f", value);
            if (retainedMode) {
                item.setRetained(true);
                itemCache.put(key, item);
            }
        } else {
            item.setValue("%.2f", value);
        }
        panels.addData(key, String.format(Locale.US, "%.2f", value));
    }

    /**
     * Adds data with a custom format string.
     */
    public void addData(String key, String format, Object... args) {
        Telemetry.Item item = itemCache.get(key);
        if (item == null) {
            item = driverStation.addData(key, format, args);
            if (retainedMode) {
                item.setRetained(true);
                itemCache.put(key, item);
            }
        } else {
            item.setValue(format, args);
        }
        panels.addData(key, String.format(Locale.US, format, args));
    }

    /**
     * Adds data using a Func supplier - only evaluated when telemetry is
     * transmitted.
     * This is ideal for expensive operations like pose calculations.
     */
    public <T> void addDataLazy(String key, Func<T> valueProducer) {
        Telemetry.Item item = itemCache.get(key);
        if (item != null) {
            // Update the item with new func (requires recreating)
            driverStation.removeItem(item);
        }
        item = driverStation.addData(key, valueProducer);
        item.setRetained(true); // Func items are always retained
        itemCache.put(key, item);
    }

    /**
     * Adds formatted data using a Func supplier with custom formatting.
     */
    public <T> void addDataLazy(String key, String format, Func<T> valueProducer) {
        Telemetry.Item item = itemCache.get(key);
        if (item != null) {
            // Update the item with new func (requires recreating)
            driverStation.removeItem(item);
        }
        item = driverStation.addData(key, format, valueProducer);
        item.setRetained(true);
        itemCache.put(key, item);
    }

    /**
     * Clears only non-retained items. Call this at the start of each loop
     * if you want to clear dynamic data while keeping retained items.
     */
    public void clearDynamic() {
        driverStation.clear(); // Only removes non-retained items
    }

    /**
     * Clears ALL items including retained ones. Use sparingly.
     */
    public void clearAll() {
        driverStation.clearAll();
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

    public void update() {
        Drawing.update();
        panels.update();
        driverStation.update();
    }
}

class Drawing {
    public static final double ROBOT_RADIUS = (Constants.Dimensions.WIDTH + Constants.Dimensions.LENGTH) / 4;
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