package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.config.Constants.Logging.INTERVAL;

import com.bylazar.panels.Panels;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.FieldRenderer;
import org.firstinspires.ftc.teamcode.util.LogLine;
import org.firstinspires.ftc.teamcode.util.TextFormat;
import org.firstinspires.ftc.teamcode.util.UnifiedLogging;

@TeleOp(name = "Rendering Example", group = "Tests")
public class RenderExample extends OpMode {

    private Telemetry.Item dashboardItem;
    private FieldRenderer fieldRenderer;

    private double poseX = 100;
    private double poseY = 77.0;
    private double poseHeading = 0.0;

    @Override
    public void init() {
        UnifiedLogging l = new UnifiedLogging(telemetry, PanelsTelemetry.INSTANCE.getTelemetry());

        // Register the text dashboard item first to enforce rendering order (top)
        dashboardItem = telemetry.addData("", "");

        // FieldRenderer registers its own item sequentially after (bottom)
        fieldRenderer = new FieldRenderer(telemetry);
    }

    @Override
    public void loop() {
        poseX += gamepad1.left_stick_x * 2.0;
        poseY -= gamepad1.left_stick_y * 2.0;
        poseHeading -= gamepad1.right_stick_x * 0.1;

        poseX = Math.max(0, Math.min(144, poseX));
        poseY = Math.max(0, Math.min(144, poseY));

        StringBuilder dashboardHtml = new StringBuilder();

        dashboardHtml.append(TextFormat.header("RAGNAROK CONTROL SUBSYSTEM_v1.0")).append("<br><br>");

        dashboardHtml.append(TextFormat.subheader("Hardware Status")).append("<br>");
        dashboardHtml.append(TextFormat.bold("Drivetrain: ")).append(TextFormat.success("NOMINAL")).append("<br>");
        dashboardHtml.append(TextFormat.bold("Intake: ")).append(TextFormat.warn("STALLED")).append("<br>");
        dashboardHtml.append(TextFormat.bold("Vision: ")).append(TextFormat.error("DISCONNECTED")).append("<br><br>");

        dashboardHtml.append(TextFormat.subheader("Odometry Localization")).append("<br>");
        LogLine poseLine = new LogLine()
                .appendBold("X: ")
                .appendColor(String.format("%5.1f", poseX), "#448aff")
                .append(" | ")
                .appendBold("Y: ")
                .appendColor(String.format("%5.1f", poseY), "#448aff")
                .append(" | ")
                .appendBold("H: ")
                .appendColor(String.format("%5.1f°", Math.toDegrees(poseHeading)), "#ff9800");

        dashboardHtml.append(poseLine.getHtml()).append("<br><br>");

        dashboardHtml.append(TextFormat.small(TextFormat.italic("Live Field Map Targeting (50ms Interval)")));

        dashboardItem.setValue(dashboardHtml.toString());

        fieldRenderer.render(poseX, poseY, poseHeading, "#00ff88");

        telemetry.update();
    }
}