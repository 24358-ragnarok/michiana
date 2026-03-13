package org.firstinspires.ftc.teamcode.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.config.MatchState
import org.firstinspires.ftc.teamcode.hardware.Robot

/**
 * The main TeleOp (Driver Controlled) mode.
 *
 * This OpMode initializes the robot and maps gamepad inputs to robot actions.
 * It handles the driving logic and mechanism controls during the driver-controlled period.
 */
@TeleOp(name = "Goatpak Strikes Back (Kotlin)", group = "0: Competition Modes")
class _Driver : OpMode() {
    /**
     * The robot hardware interface.
     */
    private lateinit var bot: Robot

    /**
     * Initializes the robot and subsystems.
     */
    override fun init() {
        bot = Robot(hardwareMap, telemetry, gamepad1, gamepad2)
    }

    /**
     * Runs constantly after initialization but before run.
     */
    override fun init_loop() {
        bot.log.drawRobot(MatchState.storedPose)
        bot.log.update()
    }

    /**
     * Runs at the beginning of the mode play.
     */
    override fun start() {
        bot.start()
    }

    /**
     * Main control loop.
     *
     * Updates the robot state and drives the drivetrain based on gamepad input.
     */
    override fun loop() {
        bot.update(time)

        // Drive the robot using the left stick for translation and right stick for rotation
        bot.dt.drive(
            bot.ctrl.main.leftStickY().state(),
            bot.ctrl.main.leftStickX().state(),
            bot.ctrl.main.rightStickX().state()
        )
    }

    /**
     * Stops the robot when the OpMode is stopped.
     */
    override fun stop() {
        MatchState.storedPose = bot.dt.follower.pose
        bot.stop()
    }
}
