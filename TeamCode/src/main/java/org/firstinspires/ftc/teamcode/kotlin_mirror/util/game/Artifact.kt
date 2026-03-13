package org.firstinspires.ftc.teamcode.kotlin_mirror.util.game

import java.util.Objects

/**
 * Represents a single game element (Artifact) with a specific color.
 *
 * Used in conjunction with [Motif] and [Classifier] to track game state.
 */
class Artifact(var color: Color = Color.NONE) {

    /**
     * Creates an empty artifact (NONE).
     */
    constructor() : this(Color.NONE)

    override fun toString(): String {
        return "${color.name[0]}"
    }

    /**
     * Checks for equality based solely on the artifact's color.
     *
     * @param other The object to compare.
     * @return true if the colors match, false otherwise.
     */
    override fun equals(other: Any?): Boolean {
        if (this === other) return true
        if (other == null || javaClass != other.javaClass) return false
        val artifact = other as Artifact
        return color == artifact.color
    }

    override fun hashCode(): Int {
        return Objects.hash(color)
    }

    /**
     * Enumeration of possible artifact colors.
     */
    enum class Color {
        GREEN,
        PURPLE,
        NONE
    }

    companion object {
        // Predefined artifact instances for common colors
        @JvmField
        val GREEN = Artifact(Color.GREEN)
        @JvmField
        val PURPLE = Artifact(Color.PURPLE)
        @JvmField
        val NONE = Artifact(Color.NONE)
    }
}
