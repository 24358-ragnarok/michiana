package org.firstinspires.ftc.teamcode.config;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRuntime;
import org.firstinspires.ftc.teamcode.util.game.Classifier;

/**
 * Global state manager for the match.
 * <p>
 * This class holds the current configuration for the match, including alliance color,
 * starting position, selected autonomous runtime, and the game element classifier state.
 * It is used to pass information between the initialization wizard and the running OpMode.
 */
public class MatchState {
    public static Classifier classifier = Classifier.empty();
    public static boolean isBlue = true;
    public static boolean startsFar = true;
    public static Pose storedPose = new Pose();
    private static AutonomousRuntime runtime = AutonomousRuntime.DEFAULT;

    /**
     * Resets the match state to default values.
     * Should be called before the start of an autonomous run.
     */
    public static void prepForAuto() {
        classifier = Classifier.empty();
        // Reset configuration to sane defaults before each auto
        isBlue = true;
        startsFar = true;
        runtime = AutonomousRuntime.DEFAULT;
    }

    public static AutonomousRuntime getAutonomousRuntime() {
        return runtime;
    }

    public static void setAutonomousRuntime(AutonomousRuntime newRuntime) {
        if (newRuntime != null) {
            runtime = newRuntime;
        }
    }

    public static boolean getStartsFar() {
        return startsFar;
    }

    public static void setStartsFar(boolean value) {
        startsFar = value;
    }

    /**
     * Cycles to the next available autonomous runtime that supports the current starting position.
     */
    public static void nextAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.nextFor(startsFar);
    }

    /**
     * Cycles to the previous available autonomous runtime that supports the current starting position.
     */
    public static void previousAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.previousFor(startsFar);
    }
}
