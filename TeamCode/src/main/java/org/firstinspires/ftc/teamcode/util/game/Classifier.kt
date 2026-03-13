package org.firstinspires.ftc.teamcode.util.game

import java.util.Arrays
import kotlin.math.min

/**
 * Manages the state of collected game elements (Artifacts) and the target scoring pattern (Motif).
 *
 * This class tracks which artifacts the robot is currently holding and determines the next
 * desired artifact color based on the identified motif.
 */
class Classifier(
    val motif: Motif,
    initial: Array<Artifact>? = null
) {
    // Fixed size buffer for stored balls
    val state: Array<Artifact> = Array(MAX_CAPACITY) { Artifact.NONE }
    
    // Number of balls currently stored
    var ballCount: Int = 0
        private set

    init {
        // Copy initial artifacts up to capacity
        if (initial != null) {
            val toCopy = min(initial.size, MAX_CAPACITY)
            System.arraycopy(initial, 0, this.state, 0, toCopy)
            this.ballCount = toCopy
        }
    }

    /**
     * Adds an artifact to the robot's storage.
     *
     * @param ball The Artifact to add.
     * @return true if added successfully, false if storage is full.
     */
    fun addBall(ball: Artifact): Boolean {
        if (ballCount < MAX_CAPACITY) {
            this.state[ballCount] = ball
            ballCount++
            return true
        }
        return false
    }

    /**
     * Determines the color of the next artifact needed to satisfy the motif pattern.
     *
     * Calculates the position in the repeating motif sequence based on the number of
     * balls already collected.
     *
     * @return The desired [Artifact.Color], or NONE if the motif is unknown.
     */
    fun getNextDesiredColor(): Artifact.Color {
        if (motif === Motif.UNKNOWN || motif.state.isEmpty()) {
            return Artifact.Color.NONE
        }
        val idx = ballCount % motif.state.size
        return motif.state[idx].color
    }

    override fun toString(): String {
        val sb = StringBuilder()
        sb.append("Motif: ").append(motif.toString()).append("\n")
        sb.append("Stored Balls (").append(ballCount).append("/").append(MAX_CAPACITY).append("): [")

        for (i in 0 until ballCount) {
            sb.append(state[i].color.name[0])
            if (i < ballCount - 1) {
                sb.append(", ")
            }
        }
        sb.append("]")
        return sb.toString()
    }

    companion object {
        private const val MAX_CAPACITY = 9

        /**
         * Creates an empty Classifier with an UNKNOWN motif.
         *
         * Useful for initialization before the randomization is detected.
         *
         * @return A new, empty Classifier instance.
         */
        @JvmStatic
        fun empty(): Classifier {
            val emptyClassifier = Classifier(Motif.UNKNOWN)
            emptyClassifier.ballCount = 0
            return emptyClassifier
        }
    }
}
