package org.firstinspires.ftc.teamcode.autonomous;

import org.firstinspires.ftc.teamcode.util.telemetry.Wizard;

/**
 * Enumeration of available autonomous strategies.
 * <p>
 * Each enum constant represents a distinct autonomous routine (e.g., "Safe", "Aggressive").
 * It defines how to build the sequence for both "Far" and "Close" starting positions.
 * <p>
 * This allows the {@link Wizard} to cycle through
 * available strategies and select the appropriate one based on the robot's starting position.
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
