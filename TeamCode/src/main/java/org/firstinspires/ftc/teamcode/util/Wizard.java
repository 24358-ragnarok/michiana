package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.telemetry.LogLine;
import org.firstinspires.ftc.teamcode.util.telemetry.TextFormat;

/**
 * MatchConfigurationWizard provides an interface for configuring match settings
 * during the init_loop phase of autonomous programs.
 */
public class Wizard {
    private final Robot bot;
    private boolean confirmed = false;

    /**
     * Creates a new MatchConfigurationWizard
     *
     * @param bot the Robot
     */
    public Wizard(Robot bot) {
        this.bot = bot;
    }

    /**
     * Call this method repeatedly in init_loop to process input and update display
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

        if (bot.ctrl.main.b().onTrue()) {
            MatchState.isBlue = false;
        }

        // Detect rising edge of dpad_down (just pressed)
        if (bot.ctrl.main.x().onTrue()) {
            MatchState.isBlue = true;
        }

        if (bot.ctrl.main.a().onTrue()) {
            setStartingPositionWithRuntimeCheck(false);
        }

        if (bot.ctrl.main.y().onTrue()) {
            setStartingPositionWithRuntimeCheck(true);
        }

        if (bot.ctrl.main.start().onTrue()) {
            confirmed = !confirmed;
        }

        // Cycle through runtimes with left/right d-pad (only to compatible ones)
        if (bot.ctrl.main.dpadRight().onTrue()) {
            MatchState.nextAutonomousRuntimeForCurrentPosition();
        }

        if (bot.ctrl.main.dpadLeft().onTrue()) {
            MatchState.previousAutonomousRuntimeForCurrentPosition();
        }

        // Update telemetry display
        updateTelemetry();
    }

    /**
     * Sets the starting position, switching runtime if the current one doesn't
     * support it.
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
     * Updates telemetry with current configuration and instructions
     */
    private void updateTelemetry() {
        bot.log.clearDynamic();

        // Header
        LogLine header = new LogLine()
                .append(TextFormat.header("MATCH CONFIGURATION"));
        bot.log.addLine(header);

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

        // Current selection summary
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
