package org.firstinspires.ftc.teamcode.kotlin_mirror.util

import org.firstinspires.ftc.teamcode.kotlin_mirror.config.MatchState
import org.firstinspires.ftc.teamcode.kotlin_mirror.hardware.Robot
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.telemetry.LogLine
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.telemetry.TextFormat

/**
 * Handles the pre-match configuration process during the init_loop.
 *
 * Allows the drive team to select:
 *
 *   - Alliance Color (Blue/Red)
 *   - Starting Position (Far/Close)
 *   - Autonomous Strategy (Runtime)
 *
 * Inputs are handled via the gamepad, and feedback is displayed on the telemetry.
 */
class Wizard(private val bot: Robot) {
    private var confirmed = false

    /**
     * Updates the wizard logic. Should be called once per init_loop iteration.
     *
     * Handles gamepad input to toggle settings and updates the telemetry display.
     * Once confirmed, settings are locked until unlocked.
     */
    fun refresh() {
        // If settings are confirmed, only allow START to unlock; ignore other inputs
        if (confirmed) {
            if (bot.ctrl.main.start().onTrue()) {
                confirmed = false
            }
            updateTelemetry()
            return
        }

        // Handle Alliance Color Selection
        if (bot.ctrl.main.b().onTrue()) {
            MatchState.isBlue = false
        }
        if (bot.ctrl.main.x().onTrue()) {
            MatchState.isBlue = true
        }

        // Handle Starting Position Selection
        if (bot.ctrl.main.a().onTrue()) {
            setStartingPositionWithRuntimeCheck(false) // Close
        }
        if (bot.ctrl.main.y().onTrue()) {
            setStartingPositionWithRuntimeCheck(true) // Far
        }

        // Handle Confirmation
        if (bot.ctrl.main.start().onTrue()) {
            confirmed = !confirmed
        }

        // Handle Runtime Selection (cycling through compatible options)
        if (bot.ctrl.main.dpadRight().onTrue()) {
            MatchState.nextAutonomousRuntimeForCurrentPosition()
        }
        if (bot.ctrl.main.dpadLeft().onTrue()) {
            MatchState.previousAutonomousRuntimeForCurrentPosition()
        }

        updateTelemetry()
    }

    /**
     * Sets the starting position and ensures the selected runtime is compatible.
     *
     * If the current runtime doesn't support the new position, it automatically switches
     * to the next available runtime that does.
     *
     * @param startsFar True for Far, False for Close.
     */
    private fun setStartingPositionWithRuntimeCheck(startsFar: Boolean) {
        MatchState.startsFar = startsFar

        // If current runtime doesn't support the new position, switch to one that does
        if (!MatchState.runtime.supportsPosition(startsFar)) {
            MatchState.setAutonomousRuntime(
                MatchState.runtime.nextFor(startsFar)
            )
        }
    }

    /**
     * Renders the configuration menu to the telemetry.
     */
    private fun updateTelemetry() {
        bot.log.clearDynamic()

        // Header
        val header = LogLine()
            .append(TextFormat.header("MATCH CONFIGURATION"))
        bot.log.addLine(header)

        // Instructions
        if (!confirmed) {
            bot.log.addLine(
                LogLine()
                    .appendBold("X/B")
                    .append("  → Alliance Color (")
                    .append(if (MatchState.isBlue) "Blue" else "Red")
                    .append(")")
            )
            bot.log.addLine(
                LogLine()
                    .appendBold("A/Y")
                    .append("  → Starting Position (")
                    .append(if (MatchState.startsFar) "Far" else "Close")
                    .append(")")
            )
            bot.log.addLine(
                LogLine()
                    .appendBold("D-Pad L/R")
                    .append("  → Runtime")
            )
            bot.log.addLine(
                LogLine()
                    .appendBold("START")
                    .append("  → Confirm")
            )
        } else {
            bot.log.addLine(
                LogLine()
                    .appendSuccess("CONFIGURATION CONFIRMED")
            )
            bot.log.addLine(
                LogLine()
                    .append("press ")
                    .appendBold("START")
                    .append(" to edit")
            )
        }

        bot.log.addLine("")

        // Current Settings Display
        val allianceLine = LogLine()
            .appendBold("Alliance: ")
            .appendColor(
                if (MatchState.isBlue) "BLUE" else "RED",
                if (MatchState.isBlue) "#448aff" else "#ef5350"
            )
        bot.log.addLine(allianceLine)

        val positionLine = LogLine()
            .appendBold("Start: ")
            .append(if (MatchState.startsFar) "Far" else "Close")
        bot.log.addLine(positionLine)

        val runtimeLine = LogLine()
            .appendBold("Runtime: ")
            .append(MatchState.runtime.displayName)
        bot.log.addLine(runtimeLine)
    }
}
