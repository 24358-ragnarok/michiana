package org.firstinspires.ftc.teamcode.kotlin_mirror.opmodes

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.AutonomousSequence
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.MatchState
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.Settings
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.Wizard

/**
 * The main Autonomous mode.
 *
 * This OpMode executes the pre-programmed autonomous routine selected via the
 * configuration wizard. It uses the [AutonomousSequence]
 * system to run actions.
 */
@Autonomous(name = "Revenge of the Boonstra (Kotlin)", preselectTeleOp = "Goatpak Strikes Back (Kotlin)")
class _Auto : OpMode() {
    /**
     * The robot hardware interface.
     */
    private lateinit var bot: Robot
    private lateinit var auto: AutonomousSequence

    /**
     * A configuration wizard to handle initial telemetry.
     */
    private lateinit var wizard: Wizard

    /**
     * Initializes the robot and subsystems.
     */
    override fun init() {
        MatchState.prepForAuto()
        bot = Robot(hardwareMap, telemetry, gamepad1, gamepad2)
        wizard = Wizard(bot)
    }

    /**
     * Runs constantly after initialization but before run.
     */
    override fun init_loop() {
        wizard.refresh()
        bot.log.update()
    }

    /**
     * Runs at the beginning of the mode play.
     */
    override fun start() {
        bot.dt.follower.setStartingPose(
            if (MatchState.startsFar) Settings.Positions.BotPoses.START_FAR else Settings.Positions.BotPoses.START_CLOSE
        )
        bot.start()

        val runtime = MatchState.runtime
        auto = if (MatchState.startsFar) {
            runtime.buildFarSequence()
        } else {
            runtime.buildCloseSequence()
        }

        // Start the sequence
        auto.start(bot)
    }

    /**
     * Main autonomous loop.
     *
     * Updates the robot state and the autonomous sequence.
     */
    override fun loop() {
        bot.update(time)
        auto.update(bot)
    }

    /**
     * Stops the robot when the OpMode is stopped.
     */
    override fun stop() {
        MatchState.storedPose = bot.dt.follower.pose
        bot.stop()
    }
}
