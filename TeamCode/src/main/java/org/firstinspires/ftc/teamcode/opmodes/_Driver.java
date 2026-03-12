package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.hardware.Robot;

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

    /**
     * Initializes the robot and subsystems.
     */
    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
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
        bot.start();
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
                bot.ctrl.main.leftStickY().state(),
                bot.ctrl.main.leftStickX().state(),
                bot.ctrl.main.rightStickX().state()
        );
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
