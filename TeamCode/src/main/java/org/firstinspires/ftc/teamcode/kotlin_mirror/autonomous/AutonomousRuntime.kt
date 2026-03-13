package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous

import org.firstinspires.ftc.teamcode.kotlin_mirror.util.Wizard

/**
 * Enumeration of available autonomous strategies.
 *
 * Each enum constant represents a distinct autonomous routine (e.g., "Safe", "Aggressive").
 * It defines how to build the sequence for both "Far" and "Close" starting positions.
 *
 * This allows the [Wizard] to cycle through
 * available strategies and select the appropriate one based on the robot's starting position.
 */
enum class AutonomousRuntime(
    /**
     * Gets the human-readable name of the runtime for telemetry.
     *
     * @return The display name.
     */
    val displayName: String
) {
    DEFAULT("I have yet to make an autonomous mode.") {
        override fun buildFarSequence(): AutonomousSequence {
            return SequenceBuilder()
                .build()
        }

        override fun buildCloseSequence(): AutonomousSequence {
            return SequenceBuilder()
                .build()
        }
    };

    /**
     * Builds the autonomous sequence for the FAR starting position.
     *
     * @return The constructed AutonomousSequence.
     */
    abstract fun buildFarSequence(): AutonomousSequence

    /**
     * Builds the autonomous sequence for the CLOSE starting position.
     *
     * @return The constructed AutonomousSequence.
     */
    abstract fun buildCloseSequence(): AutonomousSequence

    /**
     * Checks if this runtime supports the FAR starting position.
     *
     * @return true if supported, false otherwise.
     */
    fun supportsFar(): Boolean {
        return true
    }

    /**
     * Checks if this runtime supports the CLOSE starting position.
     *
     * @return true if supported, false otherwise.
     */
    fun supportsClose(): Boolean {
        return true
    }

    /**
     * Checks if this runtime supports the given starting position.
     *
     * @param startsFar true for FAR position, false for CLOSE position.
     * @return true if the position is supported.
     */
    fun supportsPosition(startsFar: Boolean): Boolean {
        return if (startsFar) supportsFar() else supportsClose()
    }

    /**
     * Gets the next runtime in the enum declaration order (cyclic).
     *
     * @return The next AutonomousRuntime.
     */
    operator fun next(): AutonomousRuntime {
        val values = values()
        return values[(this.ordinal + 1) % values.size]
    }

    /**
     * Gets the previous runtime in the enum declaration order (cyclic).
     *
     * @return The previous AutonomousRuntime.
     */
    fun previous(): AutonomousRuntime {
        val values = values()
        return values[(this.ordinal - 1 + values.size) % values.size]
    }

    /**
     * Gets the next runtime that supports the specified starting position.
     *
     * Skips runtimes that are incompatible with the current position setting.
     *
     * @param startsFar The starting position requirement.
     * @return The next compatible AutonomousRuntime.
     */
    fun nextFor(startsFar: Boolean): AutonomousRuntime {
        var candidate = this.next()
        var attempts = 0
        while (!candidate.supportsPosition(startsFar) && attempts < values().size) {
            candidate = candidate.next()
            attempts++
        }
        return candidate
    }

    /**
     * Gets the previous runtime that supports the specified starting position.
     *
     * Skips runtimes that are incompatible with the current position setting.
     *
     * @param startsFar The starting position requirement.
     * @return The previous compatible AutonomousRuntime.
     */
    fun previousFor(startsFar: Boolean): AutonomousRuntime {
        var candidate = this.previous()
        var attempts = 0
        while (!candidate.supportsPosition(startsFar) && attempts < values().size) {
            candidate = candidate.previous()
            attempts++
        }
        return candidate
    }
}
