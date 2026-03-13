package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous

import com.pedropathing.geometry.Pose
import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.actions.*
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.MatchState
import java.util.function.Consumer

/**
 * A fluent builder API for creating [AutonomousSequence]s.
 *
 * This class provides a readable, chainable way to define autonomous routines.
 * It abstracts away the creation of specific [AutonomousAction] instances.
 *
 * Example:
 * ```
 * SequenceBuilder()
 *     .moveTo(targetPose)
 *     .wait(1.0)
 *     .build()
 * ```
 */
class SequenceBuilder {

    private val sequence: AutonomousSequence = AutonomousSequence()

    /**
     * Adds a linear path action to the sequence.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates).
     * @param name       A descriptive name for the action.
     * @return This builder instance.
     */
    fun moveTo(targetPose: Pose, name: String): SequenceBuilder {
        sequence.addAction(LinearPathAction(targetPose, name))
        return this
    }

    /**
     * Adds a linear path action with an auto-generated name.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates).
     * @return This builder instance.
     */
    fun moveTo(targetPose: Pose): SequenceBuilder {
        sequence.addAction(LinearPathAction(targetPose))
        return this
    }

    /**
     * Adds a splined path action with explicit control points.
     *
     * @param targetPose    The target pose (in BLUE alliance coordinates).
     * @param name          A descriptive name for the action.
     * @param controlPoints The control points defining the spline.
     * @return This builder instance.
     */
    fun moveSplineTo(targetPose: Pose, name: String, vararg controlPoints: Pose): SequenceBuilder {
        sequence.addAction(SplinedPathAction(targetPose, name, *controlPoints))
        return this
    }

    /**
     * Adds a splined path action with auto-generated control points.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates).
     * @param name       A descriptive name for the action.
     * @return This builder instance.
     */
    fun moveSplineTo(targetPose: Pose, name: String): SequenceBuilder {
        sequence.addAction(SplinedPathAction(targetPose, name))
        return this
    }

    /**
     * Adds a curved path action with explicit control points.
     *
     * @param targetPose    The target pose (in BLUE alliance coordinates).
     * @param name          A descriptive name for the action.
     * @param controlPoints The control points defining the curve.
     * @return This builder instance.
     */
    fun moveCurveTo(targetPose: Pose, name: String, vararg controlPoints: Pose): SequenceBuilder {
        sequence.addAction(CurvePathAction(targetPose, name, *controlPoints))
        return this
    }

    /**
     * Adds a curved path action with explicit control points and an auto-generated name.
     *
     * @param targetPose    The target pose (in BLUE alliance coordinates).
     * @param controlPoints The control points defining the curve.
     * @return This builder instance.
     */
    fun moveCurveTo(targetPose: Pose, vararg controlPoints: Pose): SequenceBuilder {
        sequence.addAction(CurvePathAction(targetPose, *controlPoints))
        return this
    }

    /**
     * Adds a curved path action with a single control point.
     *
     * @param targetPose   The target pose (in BLUE alliance coordinates).
     * @param controlPoint The single control point.
     * @param name         A descriptive name for the action.
     * @return This builder instance.
     */
    fun moveCurveToVia(targetPose: Pose, controlPoint: Pose, name: String): SequenceBuilder {
        sequence.addAction(CurvePathAction.withSingleControlPoint(targetPose, controlPoint, name))
        return this
    }

    /**
     * Adds an action to hold the robot's position at the specified pose indefinitely.
     *
     * This is typically used as the final action in a sequence.
     *
     * @param targetPose The pose to hold (in BLUE alliance coordinates).
     * @return This builder instance.
     */
    fun endAt(targetPose: Pose): SequenceBuilder {
        val finalPose = if (MatchState.isBlue) targetPose else targetPose.mirror()
        sequence.addAction(EndAtAction(finalPose))
        return this
    }

    /**
     * Adds a wait action to the sequence.
     *
     * @param seconds The duration to wait in seconds.
     * @return This builder instance.
     */
    fun wait(seconds: Double): SequenceBuilder {
        sequence.addAction(WaitAction(seconds))
        return this
    }

    /**
     * Adds a custom [AutonomousAction] to the sequence.
     *
     * @param action The action to add.
     * @return This builder instance.
     */
    fun addAction(action: AutonomousAction): SequenceBuilder {
        sequence.addAction(action)
        return this
    }

    /**
     * Adds a set of actions to run in parallel.
     *
     * The parallel action completes when all of its child actions have completed.
     *
     * @param actions The actions to run simultaneously.
     * @return This builder instance.
     */
    fun parallel(vararg actions: AutonomousAction): SequenceBuilder {
        sequence.addAction(ParallelAction(*actions))
        return this
    }

    /**
     * Creates a loop that repeats a sub-sequence of actions until a certain time remains.
     *
     * Useful for maximizing cycles in the remaining autonomous period.
     *
     * @param secondsToLeave The minimum time (in seconds) that must remain to start another iteration.
     * @param loopBuilder    A consumer that defines the sequence to be looped.
     * @return This builder instance.
     */
    fun loopUntilSecondsLeft(
        secondsToLeave: Double,
        loopBuilder: Consumer<SequenceBuilder>
    ): SequenceBuilder {
        val subBuilder = SequenceBuilder()
        loopBuilder.accept(subBuilder)
        sequence.addAction(LoopAction(secondsToLeave, subBuilder.build()))
        return this
    }

    /**
     * Builds and returns the final [AutonomousSequence].
     *
     * @return The constructed sequence.
     */
    fun build(): AutonomousSequence {
        return sequence
    }
}
