package org.firstinspires.ftc.teamcode.util.telemetry;

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
 * A unified logging system that bridges standard FTC Telemetry with FTControl Panels.
 * <p>
 * This class provides a centralized way to send data to both the Driver Station and
 * the FTControl Panels dashboard. It handles:
 * <ul>
 *   <li>Automatic synchronization between DS and Panels.</li>
 *   <li>Caching of telemetry items for efficient updates.</li>
 *   <li>Field rendering and robot pose visualization.</li>
 *   <li>Formatted text output (HTML for DS).</li>
 * </ul>
 */
public class Logging {
    public final JoinedTelemetry log;
    private FieldRenderer fieldRenderer;
    private final Map<String, Telemetry.Item> itemCache = new HashMap<>();
    private final Map<String, Telemetry.Line> lineCache = new HashMap<>();

    /**
     * Initializes the logging system.
     *
     * @param telemetry The standard OpMode telemetry instance.
     */
    public Logging(Telemetry telemetry) {
        // joinedTelemetry automatically bridges DS and Panels
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

    /**
     * Completes the setup, including initializing the field renderer.
     * Should be called after hardware initialization is complete.
     */
    public void finishSetup() {
        this.fieldRenderer = new FieldRenderer(log);
    }

    /**
     * Updates or creates a persistent telemetry item.
     * <p>
     * Useful for high-priority status indicators that should remain visible at the top
     * of the telemetry log.
     *
     * @param key     The caption/key for the item.
     * @param content The value to display.
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

    /**
     * Adds a labeled line to the telemetry, caching it for reuse.
     *
     * @param label The label text.
     * @return The created or retrieved {@link Telemetry.Line}.
     */
    public Telemetry.Line addLabeledLine(String label) {
        Telemetry.Line line = lineCache.get(label);
        if (line == null) {
            line = log.addLine(label);
            lineCache.put(label, line);
        }
        return line;
    }

    /**
     * Adds a formatted {@link LogLine} to the telemetry output.
     *
     * @param line The LogLine object containing formatted text.
     */
    public void addLine(LogLine line) {
        log.addLine(line.getHtml());
    }

    /**
     * Adds a raw string line to the telemetry output.
     *
     * @param line The string to display.
     */
    public void addLine(String line) {
        log.addLine(line);
    }

    /**
     * Adds a data item to the telemetry.
     * <p>
     * If retained mode is enabled, the item is cached and updated in place.
     *
     * @param key   The caption.
     * @param value The value object.
     */
    public void addData(String key, Object value) {
        Telemetry.Item item = itemCache.get(key);
        if (item == null) {
            item = log.addData(key, value);
            item.setRetained(true);
            itemCache.put(key, item);
        } else {
            item.setValue(value);
        }
    }

    /**
     * Adds a formatted number to the telemetry.
     *
     * @param key   The caption.
     * @param value The double value, formatted to 2 decimal places.
     */
    public void addNumber(String key, double value) {
        addData(key, String.format(Locale.US, "%.2f", value));
    }

    /**
     * Registers a value producer that is polled automatically.
     *
     * @param key           The caption.
     * @param valueProducer A function that returns the value to display.
     */
    public void listen(String key, Func<?> valueProducer) {
        Telemetry.Item item = itemCache.get(key);
        if (item != null) {
            log.removeItem(item);
        }
        item = log.addData(key, valueProducer);
        item.setRetained(true);
        itemCache.put(key, item);
    }

    /**
     * Clears all non-retained telemetry items.
     */
    public void clearDynamic() {
        log.clear();
    }

    /**
     * Completely clears the telemetry and resets all caches.
     */
    public void clearAll() {
        log.clearAll();
        itemCache.clear();
        lineCache.clear();
    }

    // --- Drawing Methods ---

    public void drawRobot(Pose pose) {
        Drawing.drawRobot(pose);
    }

    public void drawDebug(Follower follower) {
        Drawing.drawDebug(follower);
    }

    public void drawPath(PathChain path) {
        Drawing.drawPath(path);
    }

    /**
     * Updates the telemetry transmission and field rendering.
     * <p>
     * Should be called at the end of every loop cycle.
     *
     */
    public void update() {
        Drawing.update();
        log.update();
        log.clear();
    }

    public void update(Pose robotPose) {
        if (Settings.Logging.DRAW_FIELD && fieldRenderer != null) {
            fieldRenderer.render(robotPose.getX(), robotPose.getY(), robotPose.getHeading(), robotLook.getOutlineFill());
        }
        update();
    }
}

/**
 * Helper class for drawing on the FTControl Panels field visualization.
 */
class Drawing {
    public static final double ROBOT_RADIUS = (Settings.Dimensions.WIDTH + Settings.Dimensions.LENGTH) / 4;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    /**
     * Initializes the field configuration with PedroPathing presets.
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        update();
    }

    /**
     * Draws debug information from the PedroPathing follower.
     * <p>
     * Visualizes the current path, the closest point on the path, the robot's pose history,
     * and the current robot pose.
     *
     * @param follower The PedroPathing follower instance.
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
     * Draws the robot at a specific pose.
     *
     * @param pose  The pose to draw.
     * @param style The visual style (color, opacity).
     */
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

    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

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

    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    public static void drawPath(PathChain pathChain) {
        drawPath(pathChain, robotLook);
    }

    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {
            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, robotLook);
    }

    /**
     * Sends the drawing packet to the Panels client.
     */
    public static void update() {
        panelsField.update();
    }
}
