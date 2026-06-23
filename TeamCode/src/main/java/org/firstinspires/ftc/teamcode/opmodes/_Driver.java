package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.Style;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * The main TeleOp (Driver Controlled) mode.
 * <p>
 * This OpMode initializes the robot and maps gamepad inputs to robot actions.
 * It handles the driving logic and mechanism controls during the driver-controlled period.
 */
@TeleOp(name = "dreaming of deepak tn", group = "0: Competition Modes")
public class _Driver extends OpMode {
    /**
     * The robot hardware interface.
     */
    public Robot bot;

    @IgnoreConfigurable
    static TelemetryManager telemetryM;

    /**
     * Initializes the robot and subsystems.
     */
    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

    }

    /**
     * Runs constantly after initialization but before run.
     */
    @Override
    public void init_loop() {
        bot.log.drawRobot(MatchState.storedPose);
        bot.log.update();
    }

    /**
     * Runs at the beginning of the mode play.
     */
    @Override
    public void start() {
        bot.dt.follower.setStartingPose(MatchState.storedPose);
        bot.start(time);
    }

    /**
     * Main control loop.
     * <p>
     * Updates the robot state and drives the drivetrain based on gamepad input.
     */
    @Override
    public void loop() {
        bot.update(time);

        // Drive the robot using the left stick for translation and right stick for rotation
        bot.dt.drive(
                -bot.ctrl.main.left_stick_y,
                -bot.ctrl.main.left_stick_x,
                -bot.ctrl.main.right_stick_x + ((bot.ctrl.main.left_trigger - bot.ctrl.main.right_trigger) / 5)
        );
        if (bot.ctrl.main.psWasPressed()) {
            if (MatchState.isBlue) {
                bot.dt.follower.setPose(Settings.Positions.TeleOp.RESET);
            } else {
                bot.dt.follower.setPose(Settings.Positions.TeleOp.RESET.mirror());
            }
        }
        if (bot.ctrl.main.leftBumperWasPressed()) {
            bot.turret.getYaw().offset += Math.toRadians(2);
        }
        if (bot.ctrl.main.rightBumperWasPressed()) {
            bot.turret.getYaw().offset -= Math.toRadians(2);
        }
        if (bot.ctrl.sub.dpadUpWasReleased()) {
            bot.turret.getHood().offset += Math.toRadians(2);
        }
        if (bot.ctrl.sub.dpadDownWasPressed()) {
            bot.turret.getHood().offset -= Math.toRadians(2);
        }
        if (bot.ctrl.sub.dpadLeftWasPressed()) {
            bot.turret.getFlywheel().offset -= 50;
        }
        if (bot.ctrl.sub.dpadRightWasPressed()) {
            bot.turret.getFlywheel().offset += 50;
        }
        if (bot.ctrl.main.optionsWasPressed()) {
            bot.turret.getYaw().toggle();
        }
        if (bot.ctrl.sub.right_trigger > 0.1) {
            bot.intake.in();
        } else if (bot.ctrl.sub.left_trigger > 0.1) {
            bot.intake.out();
        } else {
            bot.intake.stop();
        }
        if (bot.ctrl.sub.leftBumperWasPressed()) {
            bot.turret.getTung().open();
        }
        if (bot.ctrl.sub.leftBumperWasReleased()) {
            bot.turret.getTung().close();
        }
        if (bot.ctrl.sub.backWasPressed()) {
            bot.turret.getFlywheel().offset = 0;
            bot.turret.getHood().offset = 0;
        }
        bot.log.addData("flyw offset", bot.turret.getFlywheel().offset);
        bot.log.addData("rpm", bot.turret.getFlywheel().getCurrentRPM());
        bot.log.addData("yaw offset", bot.turret.getYaw().offset);
        Drawing.drawRobot(MatchState.isBlue ? Settings.Positions.Towers.BLUE_GOAL : Settings.Positions.Towers.RED_GOAL, new Style("#5FD700", "#5FD700", 1));

    }

    /**
     * Stops the robot when the OpMode is stopped.
     */
    @Override
    public void stop() {
        MatchState.storedPose = bot.dt.follower.getPose();
        bot.stop();
    }
}
