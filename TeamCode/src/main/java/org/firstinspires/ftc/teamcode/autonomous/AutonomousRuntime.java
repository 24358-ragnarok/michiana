package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.telemetry.Wizard;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.config.Settings;

/**
 * Enumeration of available autonomous strategies.
 * <p>
 * Each enum constant represents a distinct autonomous routine (e.g., "Safe",
 * "Aggressive").
 * It defines how to build the sequence for both "Far" and "Close" starting
 * positions.
 * <p>
 * This allows the {@link Wizard} to cycle through
 * available strategies and select the appropriate one based on the robot's
 * starting position.
 */
public enum AutonomousRuntime {
    DEFAULT("I have yet to make an autonomous mode.") {
        @Override
        public AutonomousSequence buildFarSequence() {
            return new SequenceBuilder()
                    .build();
        }

        @Override
        public AutonomousSequence buildCloseSequence() {
            return new SequenceBuilder()
                    .build();
        }
    },
    CLASSIC_12_BALL("Classic 12 Ball") {
        @Override
        public AutonomousSequence buildFarSequence() {
            return new SequenceBuilder()
                    // Preload
                    .moveTo(Settings.Positions.TeleOp.FAR_SHOOT_AUTO, "Launch Preload")
                    .launch()
                    // Set 1
                    .moveSplineTo(Settings.Positions.Samples.Preset1.END, "Grab Set 1",
                            Settings.Positions.ControlPoints.PRESET_1_APPROACH_FAR)
                    .withIntake()
                    .moveCurveToVia(Settings.Positions.TeleOp.FAR_SHOOT_AUTO,
                            Settings.Positions.ControlPoints.PRESET_1_END_TO_FAR_SHOOT, "Return Set 1")
                    .launch()
                    // Set 2
                    .moveSplineTo(Settings.Positions.Samples.Preset2.END, "Grab Set 2",
                            Settings.Positions.ControlPoints.PRESET_2_APPROACH_FAR)
                    .withIntake()
                    .moveTo(Settings.Positions.TeleOp.FAR_SHOOT_AUTO, "Return Set 2")
                    .launch()
                    // Park
                    .moveTo(Settings.Positions.Park.FAR, "Park")
                    .endAt(Settings.Positions.Park.FAR)
                    .build();
        }

        @Override
        public AutonomousSequence buildCloseSequence() {
            return new SequenceBuilder()
                    // Preload
                    .moveTo(Settings.Positions.TeleOp.CLOSE_SHOOT_AUTO, "Launch Preload")
                    .launch()
                    // Preset 2
                    .moveCurveToVia(Settings.Positions.Samples.Preset2.END_AND_EMPTY_GATE,
                            Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET2_END, "Grab Preset 2")
                    .withIntake()
                    .moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT_AUTO,
                            Settings.Positions.ControlPoints.FROM_CLOSE_SHOOT_TO_PRESET2_END, "Return Preset 2")
                    .launch()
                    // Loop the "Eat" logic until 6 seconds remain
                    .loopUntilSecondsLeft(6, loop -> loop
                            .moveCurveToVia(Settings.Positions.Samples.GateAndEating.EMPTY_GATE,
                                    Settings.Positions.ControlPoints.EMPTY_GATE_APPROACH, "Collect")
                            .withIntake()
                            .wait(0.5)
                            .moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT_AUTO,
                                    Settings.Positions.ControlPoints.EMPTY_GATE_APPROACH, "Return")
                            .launch())
                    // Park
                    .moveTo(Settings.Positions.Park.CLOSE, "Park")
                    .endAt(Settings.Positions.Park.CLOSE)
                    .build();
        }
    },
    ;

    private final String displayName;

    AutonomousRuntime(String displayName) {
        this.displayName = displayName;
    }

    /**
     * Gets the human-readable name of the runtime for telemetry.
     *
     * @return The display name.
     */
    public String getDisplayName() {
        return displayName;
    }

    /**
     * Builds the autonomous sequence for the FAR starting position.
     *
     * @return The constructed AutonomousSequence.
     */
    public abstract AutonomousSequence buildFarSequence();

    /**
     * Builds the autonomous sequence for the CLOSE starting position.
     *
     * @return The constructed AutonomousSequence.
     */
    public abstract AutonomousSequence buildCloseSequence();

    /**
     * Checks if this runtime supports the FAR starting position.
     *
     * @return true if supported, false otherwise.
     */
    public boolean supportsFar() {
        return true;
    }

    /**
     * Checks if this runtime supports the CLOSE starting position.
     *
     * @return true if supported, false otherwise.
     */
    public boolean supportsClose() {
        return true;
    }

    /**
     * Checks if this runtime supports the given starting position.
     *
     * @param startsFar true for FAR position, false for CLOSE position.
     * @return true if the position is supported.
     */
    public boolean supportsPosition(boolean startsFar) {
        return startsFar
                ? supportsFar()
                : supportsClose();
    }

    /**
     * Gets the next runtime in the enum declaration order (cyclic).
     *
     * @return The next AutonomousRuntime.
     */
    public AutonomousRuntime next() {
        AutonomousRuntime[] values = values();
        return values[(this.ordinal() + 1) % values.length];
    }

    /**
     * Gets the previous runtime in the enum declaration order (cyclic).
     *
     * @return The previous AutonomousRuntime.
     */
    public AutonomousRuntime previous() {
        AutonomousRuntime[] values = values();
        return values[(this.ordinal() - 1 + values.length) % values.length];
    }

    /**
     * Gets the next runtime that supports the specified starting position.
     * <p>
     * Skips runtimes that are incompatible with the current position setting.
     *
     * @param startsFar The starting position requirement.
     * @return The next compatible AutonomousRuntime.
     */
    public AutonomousRuntime nextFor(boolean startsFar) {
        AutonomousRuntime candidate = this.next();
        int attempts = 0;
        while (!candidate.supportsPosition(startsFar) && attempts < values().length) {
            candidate = candidate.next();
            attempts++;
        }
        return candidate;
    }

    /**
     * Gets the previous runtime that supports the specified starting position.
     * <p>
     * Skips runtimes that are incompatible with the current position setting.
     *
     * @param startsFar The starting position requirement.
     * @return The previous compatible AutonomousRuntime.
     */
    public AutonomousRuntime previousFor(boolean startsFar) {
        AutonomousRuntime candidate = this.previous();
        int attempts = 0;
        while (!candidate.supportsPosition(startsFar) && attempts < values().length) {
            candidate = candidate.previous();
            attempts++;
        }
        return candidate;
    }
}
