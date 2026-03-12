package org.firstinspires.ftc.teamcode.autonomous;

/**
 * Defines different autonomous runtime strategies.
 * Each runtime provides both a FAR and CLOSE sequence variant.
 * <p>
 * Use the MatchConfigurationWizard to select the desired runtime before the
 * match.
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
     * @return Human-readable name for telemetry display
     */
    public String getDisplayName() {
        return displayName;
    }

    /**
     * Builds the autonomous sequence for FAR starting position.
     *
     * @return The built AutonomousSequence
     */
    public abstract AutonomousSequence buildFarSequence();

    /**
     * Builds the autonomous sequence for CLOSE starting position.
     *
     * @return The built AutonomousSequence
     */
    public abstract AutonomousSequence buildCloseSequence();

    /**
     * @return true if this runtime supports FAR starting position
     */
    public boolean supportsFar() {
        return true;
    }

    /**
     * @return true if this runtime supports CLOSE starting position
     */
    public boolean supportsClose() {
        return true;
    }

    /**
     * Checks if this runtime supports the given starting position.
     *
     * @param startsFar The starting position to check
     * @return true if supported
     */
    public boolean supportsPosition(boolean startsFar) {
        return startsFar
                ? supportsFar()
                : supportsClose();
    }

    /**
     * Gets the next runtime in the cycle (for d-pad navigation).
     *
     * @return The next runtime
     */
    public AutonomousRuntime next() {
        AutonomousRuntime[] values = values();
        return values[(this.ordinal() + 1) % values.length];
    }

    /**
     * Gets the previous runtime in the cycle (for d-pad navigation).
     *
     * @return The previous runtime
     */
    public AutonomousRuntime previous() {
        AutonomousRuntime[] values = values();
        return values[(this.ordinal() - 1 + values.length) % values.length];
    }

    /**
     * Gets the next runtime that supports the given position.
     *
     * @param startsFar The starting position that must be supported
     * @return The next compatible runtime
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
     * Gets the previous runtime that supports the given position.
     *
     * @param startsFar The starting position that must be supported
     * @return The previous compatible runtime
     */
    public AutonomousRuntime previousFor(
            boolean startsFar) {
        AutonomousRuntime candidate = this.previous();
        int attempts = 0;
        while (!candidate.supportsPosition(startsFar) && attempts < values().length) {
            candidate = candidate.previous();
            attempts++;
        }
        return candidate;
    }
}
