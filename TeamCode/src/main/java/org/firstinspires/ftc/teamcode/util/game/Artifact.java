package org.firstinspires.ftc.teamcode.util.game;

import androidx.annotation.NonNull;

import java.util.Objects;

public class Artifact {
    // Static instances for clean pattern definition in Motif
    public static final Artifact GREEN = new Artifact(Color.GREEN);
    public static final Artifact PURPLE = new Artifact(Color.PURPLE);
    public static final Artifact NONE = new Artifact(Color.NONE);
    public Color color;

    public Artifact() {
        this.color = Color.NONE;
    }

    public Artifact(Color color) {
        this.color = color;
    }

    @NonNull
    @Override
    public String toString() {
        return "" + color.name().charAt(0);
    }

    /**
     * Determines equality based ONLY on the color, which is essential for Motif
     * pattern comparison.
     */
    @Override
    public boolean equals(Object o) {
        if (this == o)
            return true;
        if (o == null || getClass() != o.getClass())
            return false;
        Artifact artifact = (Artifact) o;
        return color == artifact.color;
    }

    @Override
    public int hashCode() {
        return Objects.hash(color);
    }

    public enum Color {
        GREEN,
        PURPLE,
        NONE
    }
}