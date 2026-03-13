package org.firstinspires.ftc.teamcode.config

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.autonomous.AutonomousRuntime
import org.firstinspires.ftc.teamcode.util.game.Classifier

/**
 * Global state manager for the match.
 *
 * This class holds the current configuration for the match, including alliance color,
 * starting position, selected autonomous runtime, and the game element classifier state.
 * It is used to pass information between the initialization wizard and the running OpMode.
 */
object MatchState {
    var classifier: Classifier = Classifier.empty()
    var isBlue: Boolean = true
    var startsFar: Boolean = true
    var storedPose: Pose = Pose()

    var runtime: AutonomousRuntime = AutonomousRuntime.DEFAULT
        private set

    /**
     * Resets the match state to default values.
     * Should be called before the start of an autonomous run.
     */
    fun prepForAuto() {
        classifier = Classifier.empty()
        // Reset configuration to sane defaults before each auto
        isBlue = true
        startsFar = true
        runtime = AutonomousRuntime.DEFAULT
    }

    /**
     * Cycles to the next available autonomous runtime that supports the current starting position.
     */
    fun nextAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.nextFor(startsFar)
    }

    /**
     * Cycles to the previous available autonomous runtime that supports the current starting position.
     */
    fun previousAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.previousFor(startsFar)
    }

    fun setAutonomousRuntime(newRuntime: AutonomousRuntime?) {
        if (newRuntime != null) {
            runtime = newRuntime
        }
    }
}
