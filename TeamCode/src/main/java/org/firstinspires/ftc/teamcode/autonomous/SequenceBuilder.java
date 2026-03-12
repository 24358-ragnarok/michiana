package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.actions.CurvePathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.EndAtAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LinearPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.LoopAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.ParallelAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.SplinedPathAction;
import org.firstinspires.ftc.teamcode.autonomous.actions.WaitAction;
import org.firstinspires.ftc.teamcode.config.MatchState;

import java.util.function.Consumer;

/**
 * A fluent builder API for creating {@link AutonomousSequence}s.
 * <p>
 * This class provides a readable, chainable way to define autonomous routines.
 * It abstracts away the creation of specific {@link AutonomousAction} instances.
 * <p>
 * Example:
 * <pre>
 * new SequenceBuilder()
 *     .moveTo(targetPose)
 *     .wait(1.0)
 *     .build();
 * </pre>
 */
public class SequenceBuilder {

    private final AutonomousSequence sequence;

    /**
     * Creates a new SequenceBuilder with an empty sequence.
     */
    public SequenceBuilder() {
        this.sequence = new AutonomousSequence();
    }

    /**
     * Adds a linear path action to the sequence.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates).
     * @param name       A descriptive name for the action.
     * @return This builder instance.
     */
    public SequenceBuilder moveTo(Pose targetPose, String name) {
        sequence.addAction(new LinearPathAction(targetPose, name));
        return this;
    }

    /**
     * Adds a linear path action with an auto-generated name.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates).
     * @return This builder instance.
     */
    public SequenceBuilder moveTo(Pose targetPose) {
        sequence.addAction(new LinearPathAction(targetPose));
        return this;
    }

    /**
     * Adds a splined path action with explicit control points.
     *
     * @param targetPose    The target pose (in BLUE alliance coordinates).
     * @param name          A descriptive name for the action.
     * @param controlPoints The control points defining the spline.
     * @return This builder instance.
     */
    public SequenceBuilder moveSplineTo(Pose targetPose, String name, Pose... controlPoints) {
        sequence.addAction(new SplinedPathAction(targetPose, name, controlPoints));
        return this;
    }

    /**
     * Adds a splined path action with auto-generated control points.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates).
     * @param name       A descriptive name for the action.
     * @return This builder instance.
     */
    public SequenceBuilder moveSplineTo(Pose targetPose, String name) {
        sequence.addAction(new SplinedPathAction(targetPose, name));
        return this;
    }

    /**
     * Adds a curved path action with explicit control points.
     *
     * @param targetPose    The target pose (in BLUE alliance coordinates).
     * @param name          A descriptive name for the action.
     * @param controlPoints The control points defining the curve.
     * @return This builder instance.
     */
    public SequenceBuilder moveCurveTo(Pose targetPose, String name, Pose... controlPoints) {
        sequence.addAction(new CurvePathAction(targetPose, name, controlPoints));
        return this;
    }

    /**
     * Adds a curved path action with explicit control points and an auto-generated name.
     *
     * @param targetPose    The target pose (in BLUE alliance coordinates).
     * @param controlPoints The control points defining the curve.
     * @return This builder instance.
     */
    public SequenceBuilder moveCurveTo(Pose targetPose, Pose... controlPoints) {
        sequence.addAction(new CurvePathAction(targetPose, controlPoints));
        return this;
    }

    /**
     * Adds a curved path action with a single control point.
     *
     * @param targetPose   The target pose (in BLUE alliance coordinates).
     * @param controlPoint The single control point.
     * @param name         A descriptive name for the action.
     * @return This builder instance.
     */
    public SequenceBuilder moveCurveToVia(Pose targetPose, Pose controlPoint, String name) {
        sequence.addAction(CurvePathAction.withSingleControlPoint(targetPose, controlPoint, name));
        return this;
    }

    /**
     * Adds an action to hold the robot's position at the specified pose indefinitely.
     * <p>
     * This is typically used as the final action in a sequence.
     *
     * @param targetPose The pose to hold (in BLUE alliance coordinates).
     * @return This builder instance.
     */
    public SequenceBuilder endAt(Pose targetPose) {
        Pose finalPose = MatchState.isBlue
                ? targetPose
                : targetPose.mirror();
        sequence.addAction(new EndAtAction(finalPose));
        return this;
    }

    /**
     * Adds a wait action to the sequence.
     *
     * @param seconds The duration to wait in seconds.
     * @return This builder instance.
     */
    public SequenceBuilder wait(double seconds) {
        sequence.addAction(new WaitAction(seconds));
        return this;
    }

    /**
     * Adds a custom {@link AutonomousAction} to the sequence.
     *
     * @param action The action to add.
     * @return This builder instance.
     */
    public SequenceBuilder addAction(AutonomousAction action) {
        sequence.addAction(action);
        return this;
    }

    /**
     * Adds a set of actions to run in parallel.
     * <p>
     * The parallel action completes when all of its child actions have completed.
     *
     * @param actions The actions to run simultaneously.
     * @return This builder instance.
     */
    public SequenceBuilder parallel(AutonomousAction... actions) {
        sequence.addAction(new ParallelAction(actions));
        return this;
    }

    /**
     * Creates a loop that repeats a sub-sequence of actions until a certain time remains.
     * <p>
     * Useful for maximizing cycles in the remaining autonomous period.
     *
     * @param secondsToLeave The minimum time (in seconds) that must remain to start another iteration.
     * @param loopBuilder    A consumer that defines the sequence to be looped.
     * @return This builder instance.
     */
    public SequenceBuilder loopUntilSecondsLeft(double secondsToLeave, Consumer<SequenceBuilder> loopBuilder) {
        SequenceBuilder subBuilder = new SequenceBuilder();
        loopBuilder.accept(subBuilder);
        sequence.addAction(new LoopAction(secondsToLeave, subBuilder.build()));
        return this;
    }

    /**
     * Builds and returns the final {@link AutonomousSequence}.
     *
     * @return The constructed sequence.
     */
    public AutonomousSequence build() {
        return sequence;
    }
}
