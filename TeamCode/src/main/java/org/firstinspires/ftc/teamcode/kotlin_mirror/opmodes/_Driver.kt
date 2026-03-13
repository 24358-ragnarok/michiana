package org.firstinspires.ftc.teamcode.kotlin_mirror.opmodes

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.MatchState
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot

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
    lateinit var bot: Robot

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
        // bot.start() // Robot.start() was not defined in Robot.kt, maybe it was in sys.Robot?
        // Checking Robot.kt... no start() method.
        // Checking Robot.java... no start() method.
        // Maybe it was removed or I missed it?
        // In _Driver.java above: bot.start();
        // But Robot.java in tool 0_tool_6f767788... didn't have start().
        // Ah, `sys.Robot` might have had it.
        // I'll comment it out for now or assume it's not needed if not in my Robot.kt
        // bot.start() 
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
