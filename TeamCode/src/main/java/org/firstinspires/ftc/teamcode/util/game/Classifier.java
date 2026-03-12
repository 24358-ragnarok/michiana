package org.firstinspires.ftc.teamcode.util.game;

import androidx.annotation.NonNull;

import java.util.Arrays;

/**
 * Manages the state of collected game elements (Artifacts) and the target scoring pattern (Motif).
 * <p>
 * This class tracks which artifacts the robot is currently holding and determines the next
 * desired artifact color based on the identified motif.
 */
public class Classifier {
    private static final int MAX_CAPACITY = 9;
    private final Motif motif;
	private final Artifact[] state; // Fixed size buffer for stored balls
	private int ballCount; // Number of balls currently stored

    /**
	 * Initializes the Classifier with a known Motif and an initial set of Artifacts.
     *
	 * @param classifierMotif The determined Motif pattern.
	 * @param initial         An array of Artifacts already in the robot's possession.
     */
    public Classifier(Motif classifierMotif, Artifact[] initial) {
        this.motif = classifierMotif;
        this.state = new Artifact[MAX_CAPACITY];
        this.ballCount = 0;

		// Copy initial artifacts up to capacity
        if (initial != null) {
            int toCopy = Math.min(initial.length, MAX_CAPACITY);
            System.arraycopy(initial, 0, this.state, 0, toCopy);
            this.ballCount = toCopy;
        }

		// Fill remaining slots with NONE
        for (int i = this.ballCount; i < MAX_CAPACITY; i++) {
            this.state[i] = Artifact.NONE;
        }
    }

    /**
	 * Initializes the Classifier with a known Motif and no initial artifacts.
	 *
	 * @param classifierMotif The determined Motif pattern.
	 */
    public Classifier(Motif classifierMotif) {
        this(classifierMotif, new Artifact[0]);
    }

    /**
	 * Creates an empty Classifier with an UNKNOWN motif.
	 * <p>
	 * Useful for initialization before the randomization is detected.
     *
     * @return A new, empty Classifier instance.
     */
    public static Classifier empty() {
        Artifact[] emptyState = new Artifact[MAX_CAPACITY];
        Arrays.fill(emptyState, Artifact.NONE);
        Classifier emptyClassifier = new Classifier(Motif.UNKNOWN, emptyState);
        emptyClassifier.ballCount = 0;
        return emptyClassifier;
    }

    /**
	 * Adds an artifact to the robot's storage.
     *
	 * @param ball The Artifact to add.
	 * @return true if added successfully, false if storage is full.
     */
    public boolean addBall(Artifact ball) {
        if (ballCount < MAX_CAPACITY) {
            this.state[ballCount] = ball;
            ballCount++;
            return true;
        }
		return false;
    }

    public Motif getMotif() {
        return motif;
    }

    /**
	 * Gets the current buffer of stored artifacts.
     *
	 * @return The internal array of artifacts. Note: Contains MAX_CAPACITY elements.
     */
    public Artifact[] getState() {
        return state;
    }

    public int getBallCount() {
        return ballCount;
    }

    /**
	 * Determines the color of the next artifact needed to satisfy the motif pattern.
	 * <p>
	 * Calculates the position in the repeating motif sequence based on the number of
	 * balls already collected.
     *
	 * @return The desired {@link Artifact.Color}, or NONE if the motif is unknown.
     */
    public Artifact.Color getNextDesiredColor() {
        if (motif == null || motif == Motif.UNKNOWN || motif.state.length == 0) {
            return Artifact.Color.NONE;
        }
        int idx = ballCount % motif.state.length;
        return motif.state[idx].color;
    }

    @NonNull
    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Motif: ").append(motif.toString()).append("\n");
        sb.append("Stored Balls (").append(ballCount).append("/").append(MAX_CAPACITY).append("): [");

        for (int i = 0; i < ballCount; i++) {
            sb.append(state[i].color.name().charAt(0));
            if (i < ballCount - 1) {
                sb.append(", ");
            }
        }
        sb.append("]");
        return sb.toString();
    }
}
