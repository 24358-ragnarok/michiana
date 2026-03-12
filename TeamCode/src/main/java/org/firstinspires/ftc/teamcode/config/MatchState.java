package org.firstinspires.ftc.teamcode.config;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRuntime;
import org.firstinspires.ftc.teamcode.util.game.Classifier;

public class MatchState {
    public static Classifier classifier = Classifier.empty();
    public static boolean isBlue = true;
    public static boolean startsFar = true;
    private static AutonomousRuntime runtime = AutonomousRuntime.DEFAULT;

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

    public static void nextAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.nextFor(startsFar);
    }

    public static void previousAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.previousFor(startsFar);
    }
}
