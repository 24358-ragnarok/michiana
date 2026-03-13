package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.actions

import com.pedropathing.util.Timer
import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.AutonomousAction
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot

/**
 * An action that does nothing but wait for a specified duration.
 *
 * Useful for adding delays between actions, allowing mechanisms to settle, or
 * synchronizing with alliance partners.
 */
class WaitAction(private val durationSeconds: Double) : AutonomousAction {
    private var timer: Timer? = null

    override fun initialize(bot: Robot) {
        timer = Timer()
        timer!!.resetTimer()
    }

    override fun execute(bot: Robot): Boolean {
        return timer!!.elapsedTimeSeconds >= durationSeconds
    }

    override fun end(bot: Robot, interrupted: Boolean) {
        // No cleanup needed
    }

    override val name: String
        get() = "Wait(${durationSeconds}s)"
}
