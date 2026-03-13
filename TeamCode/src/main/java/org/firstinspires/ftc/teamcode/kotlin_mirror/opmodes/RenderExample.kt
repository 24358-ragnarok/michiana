package org.firstinspires.ftc.teamcode.kotlin_mirror.opmodes

import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.telemetry.LogLine
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.telemetry.TextFormat
import kotlin.math.max
import kotlin.math.min

/**
 * A test OpMode to demonstrate the capabilities of the custom telemetry and field rendering system.
 *
 * Allows the user to move a virtual robot on the dashboard field using the gamepad and
 * displays various formatted telemetry items.
 */
@TeleOp(name = "Rendering Example (Kotlin)", group = "Tests")
class RenderExample : OpMode() {

    private lateinit var dashboardItem: Telemetry.Item
    private lateinit var bot: Robot

    // Virtual pose for simulation
    private var poseX = 100.0
    private var poseY = 77.0
    private var poseHeading = 0.0

    override fun init() {
        bot = Robot(hardwareMap, telemetry, gamepad1, gamepad2)
        // Initialize logging (though Robot already does this, we do it here to show explicit usage if needed)
        // In a real OpMode, use bot.log
        dashboardItem = telemetry.addData("", "")
    }

    override fun loop() {
        // Simulate robot movement based on joystick input
        poseX += bot.ctrl.main.leftStickX().state() * 2.0
        poseY -= bot.ctrl.main.leftStickY().state() * 2.0
        poseHeading -= bot.ctrl.main.rightStickX().state() * 0.1

        // Clamp position to field boundaries (144x144 inches)
        poseX = max(0.0, min(144.0, poseX))
        poseY = max(0.0, min(144.0, poseY))

        // Build the dashboard HTML content
        val dashboardHtml = StringBuilder()

        dashboardHtml.append(TextFormat.header("RAGNAROK CONTROL SUBSYSTEM_v1.0")).append("<br><br>")

        dashboardHtml.append(TextFormat.subheader("Hardware Status")).append("<br>")
        dashboardHtml.append(TextFormat.bold("Drivetrain: ")).append(TextFormat.success("NOMINAL"))
            .append("<br>")
        dashboardHtml.append(TextFormat.bold("Intake: ")).append(TextFormat.warn("STALLED"))
            .append("<br>")
        dashboardHtml.append(TextFormat.bold("Vision: ")).append(TextFormat.error("DISCONNECTED"))
            .append("<br><br>")

        dashboardHtml.append(TextFormat.subheader("Odometry Localization")).append("<br>")
        val poseLine = LogLine()
            .appendBold("X: ")
            .appendColor(String.format("%5.1f", poseX), "#448aff")
            .append(" | ")
            .appendBold("Y: ")
            .appendColor(String.format("%5.1f", poseY), "#448aff")
            .append(" | ")
            .appendBold("H: ")
            .appendColor(String.format("%5.1f°", Math.toDegrees(poseHeading)), "#ff9800")

        dashboardHtml.append(poseLine.html).append("<br><br>")

        dashboardHtml.append(TextFormat.small(TextFormat.italic("Live Field Map Targeting (50ms Interval)")))

        dashboardItem.setValue(dashboardHtml.toString())

        // Update the field visualization
        bot.log.update(Pose(poseX, poseY, poseHeading))
    }
}
