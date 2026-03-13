package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.actions

import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.AutonomousAction
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot
import java.util.Arrays

/**
 * An action that executes multiple child actions simultaneously.
 *
 * The parallel action completes only when ALL of its child actions have finished.
 * This is useful for performing multiple independent tasks at once, such as driving
 * while moving an intake or lift.
 */
class ParallelAction(vararg actions: AutonomousAction) : AutonomousAction {
    private val actions: List<AutonomousAction> = Arrays.asList(*actions)
    private val completed: BooleanArray = BooleanArray(this.actions.size)

    override fun initialize(bot: Robot) {
        Arrays.fill(completed, false)
        for (action in actions) {
            action.initialize(bot)
        }
    }

    override fun execute(bot: Robot): Boolean {
        var allComplete = true

        for (i in actions.indices) {
            if (!completed[i]) {
                // Execute the child action if it hasn't finished yet
                completed[i] = actions[i].execute(bot)
                if (completed[i]) {
                    actions[i].end(bot, false)
                }
            }
            allComplete = allComplete and completed[i]
        }

        return allComplete
    }

    override fun end(bot: Robot, interrupted: Boolean) {
        if (interrupted) {
            // Stop any actions that were still running
            for (i in actions.indices) {
                if (!completed[i]) {
                    actions[i].end(bot, true)
                }
            }
        }
    }

    override val name: String
        get() = "Parallel(${actions.size} actions)"
}
