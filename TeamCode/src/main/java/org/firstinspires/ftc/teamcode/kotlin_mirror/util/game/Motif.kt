package org.firstinspires.ftc.teamcode.kotlin_mirror.util.game

import java.util.Arrays

/**
 * Represents the scoring pattern (motif) of three Artifacts (e.g., Green-Purple-Purple).
 *
 * This class is used to identify the randomization pattern of game elements, which corresponds
 * to specific AprilTag IDs. It helps the robot decide which game piece to score next.
 */
class Motif(val state: Array<Artifact>) {

    /**
     * Creates a new Motif with the specified sequence of artifacts.
     *
     * @param artifacts An array of exactly 3 artifacts.
     * @throws IllegalArgumentException If the array is null or does not contain exactly 3 elements.
     */
    init {
        require(state.size == 3) { "Motif must consist of exactly three Artifacts." }
    }

    /**
     * Converts this Motif to its corresponding AprilTag ID.
     *
     * @return The AprilTag ID (21, 22, or 23), or 0 if the motif is unknown.
     */
    fun toApriltag(): Int {
        return when {
            Arrays.equals(this.state, GPP.state) -> 21
            Arrays.equals(this.state, PGP.state) -> 22
            Arrays.equals(this.state, PPG.state) -> 23
            else -> 0
        }
    }

    override fun toString(): String {
        return "[${state[0].color}, ${state[1].color}, ${state[2].color}]"
    }

    companion object {
        // Standard motif patterns defined by the game rules
        @JvmField
        val GPP = Motif(arrayOf(Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE))
        @JvmField
        val PGP = Motif(arrayOf(Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE))
        @JvmField
        val PPG = Motif(arrayOf(Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN))

        // Represents an unknown or invalid pattern
        @JvmField
        val UNKNOWN = Motif(arrayOf(Artifact.NONE, Artifact.NONE, Artifact.NONE))

        /**
         * Determines the Motif corresponding to a detected AprilTag ID.
         *
         * @param apriltagValue The ID of the detected AprilTag (21, 22, or 23).
         * @return The matching Motif, or UNKNOWN if the ID is not recognized.
         */
        @JvmStatic
        fun fromApriltag(apriltagValue: Int): Motif {
            return when (apriltagValue) {
                21 -> GPP
                22 -> PGP
                23 -> PPG
                else -> UNKNOWN
            }
        }
    }
}
