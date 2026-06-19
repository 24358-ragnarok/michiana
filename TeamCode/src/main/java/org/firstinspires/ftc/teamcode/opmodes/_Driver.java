package org.firstinspires.ftc.teamcode.opmodes;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
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
@TeleOp(name = "Goatpak Strikes Back", group = "0: Competition Modes")
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
        bot.dt.follower.setStartingPose(MatchState.getStartsFar() ?
                Settings.Positions.BotPoses.START_FAR : Settings.Positions.BotPoses.START_CLOSE);
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
                -bot.ctrl.main.right_stick_x
        );
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
            bot.turret.getFlywheel().toggle();
        }
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
