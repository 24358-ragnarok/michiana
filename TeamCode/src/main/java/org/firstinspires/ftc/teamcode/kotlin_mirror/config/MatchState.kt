package org.firstinspires.ftc.teamcode.kotlin_mirror.config

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.AutonomousRuntime
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.game.Classifier

/**
 * Global state manager for the match.
 *
 * This class holds the current configuration for the match, including alliance color,
 * starting position, selected autonomous runtime, and the game element classifier state.
 * It is used to pass information between the initialization wizard and the running OpMode.
 */
object MatchState {
    @JvmField
    var classifier: Classifier = Classifier.empty()
    @JvmField
    var isBlue: Boolean = true
    @JvmField
    var startsFar: Boolean = true
    @JvmField
    var storedPose: Pose = Pose()
    
    private var runtime: AutonomousRuntime = AutonomousRuntime.DEFAULT

    /**
     * Resets the match state to default values.
     * Should be called before the start of an autonomous run.
     */
    @JvmStatic
    fun prepForAuto() {
        classifier = Classifier.empty()
        // Reset configuration to sane defaults before each auto
        isBlue = true
        startsFar = true
        runtime = AutonomousRuntime.DEFAULT
    }

    @JvmStatic
    fun getAutonomousRuntime(): AutonomousRuntime {
        return runtime
    }

    @JvmStatic
    fun setAutonomousRuntime(newRuntime: AutonomousRuntime?) {
        if (newRuntime != null) {
            runtime = newRuntime
        }
    }

    @JvmStatic
    fun getStartsFar(): Boolean {
        return startsFar
    }

    @JvmStatic
    fun setStartsFar(value: Boolean) {
        startsFar = value
    }

    /**
     * Cycles to the next available autonomous runtime that supports the current starting position.
     */
    @JvmStatic
    fun nextAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.nextFor(startsFar)
    }

    /**
     * Cycles to the previous available autonomous runtime that supports the current starting position.
     */
    @JvmStatic
    fun previousAutonomousRuntimeForCurrentPosition() {
        runtime = runtime.previousFor(startsFar)
    }
}
