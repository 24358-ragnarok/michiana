package org.firstinspires.ftc.teamcode.util.game;

import androidx.annotation.NonNull;

import java.util.Objects;

/**
 * Represents a single game element (Artifact) with a specific color.
 * <p>
 * Used in conjunction with {@link Motif} and {@link Classifier} to track game state.
 */
public class Artifact {
    // Predefined artifact instances for common colors
    public static final Artifact GREEN = new Artifact(Color.GREEN);
    public static final Artifact PURPLE = new Artifact(Color.PURPLE);
    public static final Artifact NONE = new Artifact(Color.NONE);

    public Color color;

    /**
     * Creates an empty artifact (NONE).
     */
    public Artifact() {
        this.color = Color.NONE;
    }

    /**
     * Creates an artifact with the specified color.
     *
     * @param color The color of the artifact.
     */
    public Artifact(Color color) {
        this.color = color;
    }

    @NonNull
    @Override
    public String toString() {
        return "" + color.name().charAt(0);
    }

    /**
     * Checks for equality based solely on the artifact's color.
     *
     * @param o The object to compare.
     * @return true if the colors match, false otherwise.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        Artifact artifact = (Artifact) o;
        return color == artifact.color;
    }

    @Override
    public int hashCode() {
        return Objects.hash(color);
    }

    /**
     * Enumeration of possible artifact colors.
     */
    public enum Color {
        GREEN,
        PURPLE,
        NONE
    }
}
