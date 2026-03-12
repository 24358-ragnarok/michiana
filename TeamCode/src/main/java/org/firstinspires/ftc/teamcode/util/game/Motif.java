package org.firstinspires.ftc.teamcode.util.game;

import java.util.Arrays;

/**
 * Represents the scoring pattern (motif) of three Artifacts (e.g., Green-Purple-Purple).
 * <p>
 * This class is used to identify the randomization pattern of game elements, which corresponds
 * to specific AprilTag IDs. It helps the robot decide which game piece to score next.
 */
public class Motif {
    // Standard motif patterns defined by the game rules
    public static final Motif GPP = new Motif(new Artifact[]{Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE});
    public static final Motif PGP = new Motif(new Artifact[]{Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE});
    public static final Motif PPG = new Motif(new Artifact[]{Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN});

    // Represents an unknown or invalid pattern
    public static final Motif UNKNOWN = new Motif(new Artifact[]{Artifact.NONE, Artifact.NONE, Artifact.NONE});

    public final Artifact[] state;

    /**
     * Creates a new Motif with the specified sequence of artifacts.
     *
     * @param artifacts An array of exactly 3 artifacts.
     * @throws IllegalArgumentException If the array is null or does not contain exactly 3 elements.
     */
    public Motif(Artifact[] artifacts) {
        if (artifacts == null || artifacts.length != 3) {
            throw new IllegalArgumentException("Motif must consist of exactly three Artifacts.");
        }
        this.state = artifacts;
    }

    /**
     * Determines the Motif corresponding to a detected AprilTag ID.
     *
     * @param apriltagValue The ID of the detected AprilTag (21, 22, or 23).
     * @return The matching Motif, or UNKNOWN if the ID is not recognized.
     */
    public static Motif fromApriltag(int apriltagValue) {
        switch (apriltagValue) {
            case 21:
                return GPP;
            case 22:
                return PGP;
            case 23:
                return PPG;
            default:
                return UNKNOWN;
        }
    }

    /**
     * Converts this Motif to its corresponding AprilTag ID.
     *
     * @return The AprilTag ID (21, 22, or 23), or 0 if the motif is unknown.
     */
    public int toApriltag() {
        if (Arrays.equals(this.state, GPP.state)) {
            return 21;
        } else if (Arrays.equals(this.state, PGP.state)) {
            return 22;
        } else if (Arrays.equals(this.state, PPG.state)) {
            return 23;
        }
        return 0;
    }

    @Override
    public String toString() {
        return "[" + this.state[0].color + ", " + this.state[1].color + ", " + this.state[2].color + "]";
    }
}
