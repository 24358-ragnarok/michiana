package org.firstinspires.ftc.teamcode.util.telemetry

import com.bylazar.field.FieldManager
import com.bylazar.field.PanelsField
import com.bylazar.field.Style
import com.bylazar.telemetry.JoinedTelemetry
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.PoseHistory
import org.firstinspires.ftc.robotcore.external.Func
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.config.Settings
import java.util.Locale

/**
 * A unified logging system that bridges standard FTC Telemetry with FTControl Panels.
 *
 * This class provides a centralized way to send data to both the Driver Station and
 * the FTControl Panels dashboard. It handles:
 *
 *   - Automatic synchronization between DS and Panels.
 *   - Caching of telemetry items for efficient updates.
 *   - Field rendering and robot pose visualization.
 *   - Formatted text output (HTML for DS).
 */
class Logging(telemetry: Telemetry) {
    val log: JoinedTelemetry = JoinedTelemetry(PanelsTelemetry.INSTANCE.ftcTelemetry, telemetry)
    private var fieldRenderer: FieldRenderer? = null
    private val itemCache = HashMap<String, Telemetry.Item>()
    private val lineCache = HashMap<String, Telemetry.Line>()

    init {
        // joinedTelemetry automatically bridges DS and Panels
        Drawing.init()

        log.isAutoClear = false
        log.msTransmissionInterval = Settings.Logging.INTERVAL
        log.setDisplayFormat(Telemetry.DisplayFormat.HTML)
        log.setItemSeparator("")
        log.setCaptionValueSeparator("")
        log.clearAll()
        log.update()
    }

    /**
     * Completes the setup, including initializing the field renderer.
     * Should be called after hardware initialization is complete.
     */
    fun finishSetup() {
        this.fieldRenderer = FieldRenderer(log)
    }

    /**
     * Updates or creates a persistent telemetry item.
     *
     * Useful for high-priority status indicators that should remain visible at the top
     * of the telemetry log.
     *
     * @param key     The caption/key for the item.
     * @param content The value to display.
     */
    fun setItem(key: String, content: String) {
        var item = itemCache[key]
        if (item == null) {
            item = log.addData(key, content)
            item.isRetained = true
            itemCache[key] = item
        } else {
            item.setValue(content)
        }
    }

    /**
     * Adds a labeled line to the telemetry, caching it for reuse.
     *
     * @param label The label text.
     * @return The created or retrieved [Telemetry.Line].
     */
    fun addLabeledLine(label: String): Telemetry.Line {
        var line = lineCache[label]
        if (line == null) {
            line = log.addLine(label)
            lineCache[label] = line
        }
        return line
    }

    /**
     * Adds a formatted [LogLine] to the telemetry output.
     *
     * @param line The LogLine object containing formatted text.
     */
    fun addLine(line: LogLine) {
        log.addLine(line.html)
    }

    /**
     * Adds a raw string line to the telemetry output.
     *
     * @param line The string to display.
     */
    fun addLine(line: String) {
        log.addLine(line)
    }

    /**
     * Adds a data item to the telemetry.
     *
     * If retained mode is enabled, the item is cached and updated in place.
     *
     * @param key   The caption.
     * @param value The value object.
     */
    fun addData(key: String, value: Any) {
        var item = itemCache[key]
        if (item == null) {
            item = log.addData(key, value)
            // Retained mode logic could be added here if needed, matching Java implementation
            // Assuming retainedMode logic is handled by caller or default behavior
            item.isRetained = true
            itemCache[key] = item
        } else {
            item.setValue(value)
        }
    }

    /**
     * Adds a formatted number to the telemetry.
     *
     * @param key   The caption.
     * @param value The double value, formatted to 2 decimal places.
     */
    fun addNumber(key: String, value: Double) {
        addData(key, String.format(Locale.US, "%.2f", value))
    }

    /**
     * Registers a value producer that is polled automatically.
     *
     * @param key           The caption.
     * @param valueProducer A function that returns the value to display.
     */
    fun listen(key: String, valueProducer: Func<*>) {
        var item = itemCache[key]
        if (item != null) {
            log.removeItem(item)
        }
        item = log.addData(key, valueProducer)
        item.isRetained = true
        itemCache[key] = item
    }

    /**
     * Clears all non-retained telemetry items.
     */
    fun clearDynamic() {
        log.clear()
    }

    /**
     * Completely clears the telemetry and resets all caches.
     */
    fun clearAll() {
        log.clearAll()
        itemCache.clear()
        lineCache.clear()
    }

    // --- Drawing Methods ---

    fun drawRobot(pose: Pose) {
        Drawing.drawRobot(pose)
    }

    fun drawDebug(follower: Follower) {
        Drawing.drawDebug(follower)
    }

    fun drawPath(path: PathChain) {
        Drawing.drawPath(path)
    }

    /**
     * Updates the telemetry transmission and field rendering.
     *
     * Should be called at the end of every loop cycle.
     */
    fun update() {
        Drawing.update()
        log.update()
        log.clear()
    }

    fun update(robotPose: Pose) {
        if (Settings.Logging.DRAW_FIELD && fieldRenderer != null) {
            fieldRenderer!!.render(
                robotPose.x,
                robotPose.y,
                robotPose.heading,
                Settings.Logging.robotLook.outlineFill
            )
        }
        update()
    }
}

/**
 * Helper class for drawing on the FTControl Panels field visualization.
 */
object Drawing {
    val ROBOT_RADIUS = (Settings.Dimensions.WIDTH + Settings.Dimensions.LENGTH) / 4.0
    private val panelsField: FieldManager = PanelsField.INSTANCE.field

    /**
     * Initializes the field configuration with PedroPathing presets.
     */
    fun init() {
        panelsField.setOffsets(PanelsField.INSTANCE.presets.PEDRO_PATHING)
        update()
    }

    /**
     * Draws debug information from the PedroPathing follower.
     *
     * Visualizes the current path, the closest point on the path, the robot's pose history,
     * and the current robot pose.
     *
     * @param follower The PedroPathing follower instance.
     */
    fun drawDebug(follower: Follower) {
        if (follower.currentPathChain != null) {
            drawPath(follower.currentPathChain, Settings.Logging.followerLook)
            val closestPoint = follower.getPointFromPath(follower.currentPath.closestPointTValue)
            drawRobot(
                Pose(
                    closestPoint.x,
                    closestPoint.y,
                    follower.currentPath.getHeadingGoal(follower.currentPath.closestPointTValue)
                ), Settings.Logging.followerLook
            )
        } else if (follower.currentPath != null) {
            drawPath(follower.currentPath, Settings.Logging.followerLook)
            val closestPoint = follower.getPointFromPath(follower.currentPath.closestPointTValue)
            drawRobot(
                Pose(
                    closestPoint.x,
                    closestPoint.y,
                    follower.currentPath.getHeadingGoal(follower.currentPath.closestPointTValue)
                ), Settings.Logging.followerLook
            )
        }
        drawPoseHistory(follower.poseHistory, Settings.Logging.robotLook)
        drawRobot(follower.pose, Settings.Logging.robotLook)
    }

    /**
     * Draws the robot at a specific pose.
     *
     * @param pose  The pose to draw.
     * @param style The visual style (color, opacity).
     */
    fun drawRobot(pose: Pose?, style: Style) {
        if (pose == null || java.lang.Double.isNaN(pose.x) || java.lang.Double.isNaN(pose.y) || java.lang.Double.isNaN(
                pose.heading
            )
        ) {
            return
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(pose.x, pose.y)
        panelsField.circle(ROBOT_RADIUS)

        val v = pose.headingAsUnitVector
        v.magnitude = v.magnitude * ROBOT_RADIUS
        val x1 = pose.x + v.xComponent / 2
        val y1 = pose.y + v.yComponent / 2
        val x2 = pose.x + v.xComponent
        val y2 = pose.y + v.yComponent

        panelsField.setStyle(style)
        panelsField.moveCursor(x1, y1)
        panelsField.line(x2, y2)
    }

    fun drawRobot(pose: Pose) {
        drawRobot(pose, Settings.Logging.robotLook)
    }

    fun drawPath(path: Path, style: Style) {
        val points = path.panelsDrawingPoints

        for (i in points[0].indices) {
            for (j in points.indices) {
                if (java.lang.Double.isNaN(points[j][i])) {
                    points[j][i] = 0.0
                }
            }
        }

        panelsField.setStyle(style)
        for (i in 0 until points[0].size - 1) {
            panelsField.moveCursor(points[0][i], points[1][i])
            panelsField.line(points[0][i + 1], points[1][i + 1])
        }
    }

    fun drawPath(pathChain: PathChain, style: Style) {
        for (i in 0 until pathChain.size()) {
            drawPath(pathChain.getPath(i), style)
        }
    }

    fun drawPath(pathChain: PathChain) {
        drawPath(pathChain, Settings.Logging.robotLook)
    }

    fun drawPoseHistory(poseTracker: PoseHistory, style: Style) {
        panelsField.setStyle(style)

        val size = poseTracker.xPositionsArray.size
        for (i in 0 until size - 1) {
            panelsField.moveCursor(poseTracker.xPositionsArray[i], poseTracker.yPositionsArray[i])
            panelsField.line(poseTracker.xPositionsArray[i + 1], poseTracker.yPositionsArray[i + 1])
        }
    }

    fun drawPoseHistory(poseTracker: PoseHistory) {
        drawPoseHistory(poseTracker, Settings.Logging.robotLook)
    }

    /**
     * Sends the drawing packet to the Panels client.
     */
    fun update() {
        panelsField.update()
    }
}
