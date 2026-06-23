package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.sys.Robot;
import org.firstinspires.ftc.teamcode.sys.hardware.Turret;
import org.firstinspires.ftc.teamcode.util.shooter.PolynomialShooterModels;
import org.firstinspires.ftc.teamcode.util.shooter.ShotCalibrationSession;
import org.firstinspires.ftc.teamcode.util.telemetry.LogLine;
import org.firstinspires.ftc.teamcode.util.telemetry.TextFormat;

import java.util.Locale;

/**
 * Field calibration OpMode for building hood angle and flywheel RPM models.
 * <p>
 * Drive around the field, manually tune hood angle and RPM, shoot, and mark
 * successful
 * samples. Once enough samples are collected, the OpMode fits polynomial models
 * live and
 * prints coefficient arrays that can be pasted into {@link Settings.Turret}.
 */
@TeleOp(name = "Solution Tuner", group = "Tests")
public class SolutionTuner extends OpMode {
    private static final double HOOD_STEP_RAD = Math.toRadians(0.5);
    private static final double RPM_STEP = 50.0;
    private static final double COARSE_MULTIPLIER = 10.0;

    private Robot bot;
    private ShotCalibrationSession session;
    private Pose targetPose;

    private double manualHoodAngleRad;
    private double manualFlywheelRpm;

    private Telemetry.Item dashboardItem;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        session = new ShotCalibrationSession();
        session.setPolynomialDegree(Settings.Turret.SOLUTION_POLYNOMIAL_DEGREE);
        dashboardItem = telemetry.addData("Solution Tuner", "");
        dashboardItem.setRetained(true);
    }

    @Override
    public void init_loop() {
        if (bot.ctrl.main.bWasPressed()) {
            MatchState.isBlue = false;
        }
        if (bot.ctrl.main.xWasPressed()) {
            MatchState.isBlue = true;
        }
        renderInitTelemetry();
    }

    @Override
    public void start() {
        targetPose = resolveTargetPose();
        Turret.setTargetPose(targetPose);

        bot.dt.follower.setStartingPose(Settings.Positions.BotPoses.START_CLOSE);
        bot.start(time);

        Pose startPose = bot.dt.follower.getPose();
        double startDistance = horizontalDistance(startPose, targetPose);
        manualHoodAngleRad = PolynomialShooterModels.clipAngleRadians(
                PolynomialShooterModels.predictAngleRadians(Settings.Turret.ANGLE_COEFFICIENTS, startDistance));
        manualFlywheelRpm = 0;
    }

    @Override
    public void loop() {
        bot.peripherals.update();
        bot.dt.update();
        handleControls();

        bot.turret.getHood().setAngleRadians(manualHoodAngleRad);
        bot.turret.getFlywheel().setWheelTargetRPM(manualFlywheelRpm);
        bot.turret.updateYaw(bot.dt.follower.getPose(), bot.dt.follower.getVelocity());

        renderTelemetry();
        bot.log.update(bot.dt.follower);
    }

    private static Pose resolveTargetPose() {
        Pose blueTarget = Settings.Positions.TeleOp.FAR_SHOOT;
        return MatchState.isBlue ? blueTarget : blueTarget.mirror();
    }

    private void handleControls() {
        bot.dt.drive(
                -bot.ctrl.main.left_stick_y,
                -bot.ctrl.main.left_stick_x,
                -bot.ctrl.main.right_stick_x);

        double hoodStep = bot.ctrl.main.right_bumper ? HOOD_STEP_RAD * COARSE_MULTIPLIER : HOOD_STEP_RAD;
        double rpmStep = bot.ctrl.main.right_bumper ? RPM_STEP * COARSE_MULTIPLIER : RPM_STEP;

        if (bot.ctrl.main.dpadUpWasPressed()) {
            manualHoodAngleRad = PolynomialShooterModels.clipAngleRadians(manualHoodAngleRad - hoodStep);
        }
        if (bot.ctrl.main.dpadDownWasPressed()) {
            manualHoodAngleRad = PolynomialShooterModels.clipAngleRadians(manualHoodAngleRad + hoodStep);
        }
        if (bot.ctrl.main.dpadRightWasPressed()) {
            manualFlywheelRpm += rpmStep;
        }
        if (bot.ctrl.main.dpadLeftWasPressed()) {
            manualFlywheelRpm = Math.max(0.0, manualFlywheelRpm - rpmStep);
        }

        if (bot.ctrl.main.aWasPressed()) {
            Pose pose = bot.dt.follower.getPose();
            double distance = horizontalDistance(pose, targetPose);
            session.addSample(distance, manualHoodAngleRad, manualFlywheelRpm);
        }
        if (bot.ctrl.main.bWasPressed()) {
            session.removeLastSample();
        }
        if (bot.ctrl.main.xWasPressed()) {
            session.clearSamples();
        }
        if (bot.ctrl.main.yWasPressed()) {
            session.setPolynomialDegree(session.getPolynomialDegree() % 3 + 1);
        }

        if (bot.ctrl.main.right_trigger > 0.01) {
            bot.turret.getTung().open();
        } else if (bot.ctrl.main.left_trigger > 0.01) {
            bot.turret.getTung().close();
        }
        if (bot.ctrl.main.optionsWasPressed()) {
            bot.turret.getFlywheel().toggle();
        }
        if (bot.ctrl.main.shareWasPressed()) {
            bot.intake.in();
        }
        if (bot.ctrl.main.left_bumper && bot.ctrl.sub.right_bumper) {
            loadPredictionAtCurrentDistance();
        }
    }

    @Override
    public void stop() {
        bot.stop();
    }

    private void loadPredictionAtCurrentDistance() {
        Pose pose = bot.dt.follower.getPose();
        double distance = horizontalDistance(pose, targetPose);
        manualHoodAngleRad = PolynomialShooterModels.clipAngleRadians(
                PolynomialShooterModels.predictAngleRadians(Settings.Turret.ANGLE_COEFFICIENTS, distance));
        manualFlywheelRpm = PolynomialShooterModels.predictFlywheelRpm(
                Settings.Turret.RPM_COEFFICIENTS, distance);
    }

    private void renderTelemetry() {
        Pose pose = bot.dt.follower.getPose();
        bot.log.drawDebug(bot.dt.follower);
        double distance = horizontalDistance(pose, targetPose);

        StringBuilder html = new StringBuilder();
        html.append(TextFormat.header("Solution Tuner")).append(TextFormat.newline());
        html.append(TextFormat.subheader("Robot")).append(TextFormat.newline());
        html.append(new LogLine()
                .appendBold("X: ")
                .appendColor(String.format(Locale.US, "%.1f", pose.getX()), "#448aff")
                .append(" | ")
                .appendBold("Y: ")
                .appendColor(String.format(Locale.US, "%.1f", pose.getY()), "#448aff")
                .append(" | ")
                .appendBold("H: ")
                .appendColor(String.format(Locale.US, "%.1f°", Math.toDegrees(pose.getHeading())), "#ff9800")
                .getHtml());
        html.append(TextFormat.newline());
        html.append(TextFormat.bold("Distance to goal: "))
                .append(String.format(Locale.US, "%.2f in", distance))
                .append(TextFormat.newline());
        html.append(TextFormat.newline());

        html.append(TextFormat.subheader("Manual Solution")).append(TextFormat.newline());
        html.append(TextFormat.bold("Hood: "))
                .append(String.format(Locale.US, "%.2f° (%.4f rad)", Math.toDegrees(manualHoodAngleRad),
                        manualHoodAngleRad))
                .append(TextFormat.newline());
        html.append(TextFormat.bold("Flywheel: "))
                .append(String.format(Locale.US, "%.0f RPM (Target)", manualFlywheelRpm))
                .append(TextFormat.newline())
                .append(TextFormat.bold("Current: "))
                .append(String.format(Locale.US, "%.1f RPM", bot.turret.getFlywheel().getCurrentRPM()))
                .append(TextFormat.newline())
                .append(TextFormat.small(String.format(Locale.US, "R: %.0f TPS, L: %.0f TPS",
                        bot.turret.getFlywheel().getVelocity(),
                        bot.turret.getFlywheel().getLeftVelocity())))
                .append(TextFormat.newline());
        html.append(TextFormat.newline());

        html.append(TextFormat.subheader("Samples")).append(TextFormat.newline());
        html.append("Count: ").append(session.getSampleCount());
        html.append(" / min ").append(session.getMinimumSampleCount());
        html.append(" (degree ").append(session.getPolynomialDegree()).append(")");
        html.append(TextFormat.newline());
        html.append(TextFormat.small("A = success, B = undo, X = clear, Y = cycle degree"));
        html.append(TextFormat.newline());
        html.append(TextFormat.small("D-Pad = tune hood/RPM, RB = coarse step"));
        html.append(TextFormat.newline());
        html.append(TextFormat.small("LB+RB = load Settings model at current distance"));
        html.append(TextFormat.newline()).append(TextFormat.newline());

        html.append(TextFormat.subheader("Paste Into Settings.Turret")).append(TextFormat.newline());
        if (session.hasValidFit()) {
            html.append(TextFormat.success("Fit ready")).append(TextFormat.newline());
            html.append("<pre>").append(session.formatSettingsSnippet()).append("</pre>");
        } else {
            html.append(TextFormat.warn(session.formatSettingsSnippet()));
        }

        dashboardItem.setValue(html.toString());
        telemetry.addData("x:", pose.getX());
        telemetry.addData("y:", pose.getY());
        telemetry.addData("heading:", pose.getHeading());
    }

    private void renderInitTelemetry() {
        StringBuilder html = new StringBuilder();
        html.append(TextFormat.header("Solution Tuner")).append(TextFormat.newline());
        html.append(TextFormat.subheader("Alliance")).append(TextFormat.newline());
        if (MatchState.isBlue) {
            html.append(TextFormat.success("BLUE"));
        } else {
            html.append(TextFormat.error("RED"));
        }
        html.append(TextFormat.newline()).append(TextFormat.newline());
        html.append("B = Red, X = Blue").append(TextFormat.newline());
        html.append("Press START when ready.");
        dashboardItem.setValue(html.toString());
    }

    private static double horizontalDistance(Pose botPose, Pose goalPose) {
        double dx = goalPose.getX() - botPose.getX();
        double dy = goalPose.getY() - botPose.getY();
        return Math.hypot(dx, dy);
    }
}
