package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.actions

import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.AutonomousAction
import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.AutonomousSequence
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.Settings
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot

/**
 * An action that repeatedly executes a sub-sequence of actions until a specified time remains.
 *
 * This is useful for maximizing the number of cycles (e.g., scoring samples) within the
 * autonomous period. The loop will complete the current iteration fully before checking
 * the time condition, ensuring the robot doesn't stop in an unsafe state.
 */
class LoopAction(
    private val secondsToLeave: Double,
    private val loopSequence: AutonomousSequence
) : AutonomousAction {
    private var loopCount: Int = 0

    override fun initialize(bot: Robot) {
        loopCount = 0
        loopSequence.start(bot)
    }

    override fun execute(bot: Robot): Boolean {
        val timeLeft = Settings.Autonomous.DURATION - bot.elapsedTime

        // Check if we have enough time to start another loop
        if (timeLeft <= secondsToLeave) {
            return true // Stop looping
        }

        // Update the sub-sequence
        loopSequence.update(bot)

        // If the sub-sequence finished, restart it and increment the counter
        if (loopSequence.isComplete) {
            loopCount++
            loopSequence.start(bot)
        }

        return false // Continue executing
    }

    override fun end(bot: Robot, interrupted: Boolean) {
        loopSequence.stop(bot)
    }

    override val name: String
        get() = "Loop until ${secondsToLeave}s \n-> ${loopSequence.currentActionName}"

    override val timeoutSeconds: Double
        get() = 0.0
}
