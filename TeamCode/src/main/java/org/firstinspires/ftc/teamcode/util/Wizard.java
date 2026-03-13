package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.telemetry.LogLine;
import org.firstinspires.ftc.teamcode.util.telemetry.TextFormat;

/**
 * Handles the pre-match configuration process during the init_loop.
 * <p>
 * Allows the drive team to select:
 * <ul>
 *   <li>Alliance Color (Blue/Red)</li>
 *   <li>Starting Position (Far/Close)</li>
 *   <li>Autonomous Strategy (Runtime)</li>
 * </ul>
 * Inputs are handled via the gamepad, and feedback is displayed on the telemetry.
 */
public class Wizard {
    private final Robot bot;
    private boolean confirmed = false;

    /**
     * Creates a new configuration wizard.
     *
     * @param bot The robot instance used for input and telemetry.
     */
    public Wizard(Robot bot) {
        this.bot = bot;
    }

    /**
     * Updates the wizard logic. Should be called once per init_loop iteration.
     * <p>
     * Handles gamepad input to toggle settings and updates the telemetry display.
     * Once confirmed, settings are locked until unlocked.
     */
    public void refresh() {
        // If settings are confirmed, only allow START to unlock; ignore other inputs
        if (confirmed) {
            if (bot.ctrl.main.start().onTrue()) {
                confirmed = false;
            }
            updateTelemetry();
            return;
        }

        // Handle Alliance Color Selection
        if (bot.ctrl.main.b().onTrue()) {
            MatchState.isBlue = false;
        }
        if (bot.ctrl.main.x().onTrue()) {
            MatchState.isBlue = true;
        }

        // Handle Starting Position Selection
        if (bot.ctrl.main.a().onTrue()) {
            setStartingPositionWithRuntimeCheck(false); // Close
        }
        if (bot.ctrl.main.y().onTrue()) {
            setStartingPositionWithRuntimeCheck(true); // Far
        }

        // Handle Confirmation
        if (bot.ctrl.main.start().onTrue()) {
            confirmed = !confirmed;
        }

        // Handle Runtime Selection (cycling through compatible options)
        if (bot.ctrl.main.dpadRight().onTrue()) {
            MatchState.nextAutonomousRuntimeForCurrentPosition();
        }
        if (bot.ctrl.main.dpadLeft().onTrue()) {
            MatchState.previousAutonomousRuntimeForCurrentPosition();
        }

        updateTelemetry();
    }

    /**
     * Sets the starting position and ensures the selected runtime is compatible.
     * <p>
     * If the current runtime doesn't support the new position, it automatically switches
     * to the next available runtime that does.
     *
     * @param startsFar True for Far, False for Close.
     */
    private void setStartingPositionWithRuntimeCheck(boolean startsFar) {
        MatchState.setStartsFar(startsFar);

        // If current runtime doesn't support the new position, switch to one that does
        if (!MatchState.getAutonomousRuntime().supportsPosition(startsFar)) {
            MatchState.setAutonomousRuntime(
                    MatchState.getAutonomousRuntime().nextFor(startsFar)
            );
        }
    }

    /**
     * Renders the configuration menu to the telemetry.
     */
    private void updateTelemetry() {
        bot.log.clearDynamic();

        // Header
        LogLine header = new LogLine()
                .append(TextFormat.header("MATCH CONFIGURATION"));
        bot.log.addLine(header);

        // Instructions
        if (!confirmed) {
            bot.log.addLine(new LogLine()
                    .appendBold("X/B")
                    .append("  → Alliance Color (")
                    .append(MatchState.isBlue ? "Blue" : "Red")
                    .append(")"));
            bot.log.addLine(new LogLine()
                    .appendBold("A/Y")
                    .append("  → Starting Position (")
                    .append(MatchState.getStartsFar() ? "Far" : "Close")
                    .append(")"));
            bot.log.addLine(new LogLine()
                    .appendBold("D-Pad L/R")
                    .append("  → Runtime"));
            bot.log.addLine(new LogLine()
                    .appendBold("START")
                    .append("  → Confirm"));
        } else {
            bot.log.addLine(new LogLine()
                    .appendSuccess("CONFIGURATION CONFIRMED"));
            bot.log.addLine(new LogLine()
                    .append("press ")
                    .appendBold("START")
                    .append(" to edit"));
        }

        bot.log.addLine("");

        // Current Settings Display
        LogLine allianceLine = new LogLine()
                .appendBold("Alliance: ")
                .appendColor(MatchState.isBlue ? "BLUE" : "RED",
                        MatchState.isBlue ? "#448aff" : "#ef5350");
        bot.log.addLine(allianceLine);

        LogLine positionLine = new LogLine()
                .appendBold("Start: ")
                .append(MatchState.getStartsFar() ? "Far" : "Close");
        bot.log.addLine(positionLine);

        LogLine runtimeLine = new LogLine()
                .appendBold("Runtime: ")
                .append(MatchState.getAutonomousRuntime().getDisplayName());
        bot.log.addLine(runtimeLine);
    }
}
