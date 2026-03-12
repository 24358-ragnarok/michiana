package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.telemetry.LogLine;
import org.firstinspires.ftc.teamcode.util.telemetry.Logging;
import org.firstinspires.ftc.teamcode.util.telemetry.TextFormat;

@TeleOp(name = "Rendering Example", group = "Tests")
public class RenderExample extends OpMode {

    private Telemetry.Item dashboardItem;
    private Robot bot;

    private double poseX = 100;
    private double poseY = 77.0;
    private double poseHeading = 0.0;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        Logging log = new Logging(telemetry);
        dashboardItem = telemetry.addData("", "");
    }

    @Override
    public void loop() {
        poseX += bot.ctrl.main.leftStickX().state() * 2.0;
        poseY -= bot.ctrl.main.leftStickY().state() * 2.0;
        poseHeading -= bot.ctrl.main.rightStickX().state() * 0.1;

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

        bot.log.update(new Pose(poseX, poseY, poseHeading));
    }
}