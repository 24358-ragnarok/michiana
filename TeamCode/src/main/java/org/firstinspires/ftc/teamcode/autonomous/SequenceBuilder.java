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
 * Fluent builder for creating autonomous sequences.
 * Provides a clean, readable DSL for defining autonomous routines.
 * <p>
 * Example usage:
 *
 * <pre>
 * AutonomousSequence sequence = new SequenceBuilder(matchSettings)
 * 		.moveTo(Settings.Positions.Samples.Preset1.PREP)
 * 		.startIntake()
 * 		.moveSlowlyTo(Settings.Positions.Samples.Preset1.END)
 * 		.stopIntake()
 * 		.moveCurveToVia(Settings.Positions.TeleOp.CLOSE_SHOOT,
 * 				Settings.Positions.ControlPoints.FROM_PRESET1_TO_CLOSE, "Launch")
 * 		.launch()
 * 		.build();
 * </pre>
 */
public class SequenceBuilder {

    private final AutonomousSequence sequence;

    /**
     * Creates a new sequence builder.
     */
    public SequenceBuilder() {
        this.sequence = new AutonomousSequence();
    }

    /**
     * Adds a linear path action to a target pose.
     *
     * @param targetPose The target pose (in BLUE alliance coordinates)
     * @param name       Human-readable name for telemetry
     * @return this (for method chaining)
     */
    public SequenceBuilder moveTo(Pose targetPose, String name) {
        sequence.addAction(new LinearPathAction(targetPose, name));
        return this;
    }

    /**
     * Adds a linear path action to a target pose with auto-generated name.
     */
    public SequenceBuilder moveTo(Pose targetPose) {
        sequence.addAction(new LinearPathAction(targetPose));
        return this;
    }

    /**
     * Adds a splined path action to a target pose.
     */
    public SequenceBuilder moveSplineTo(Pose targetPose, String name, Pose... controlPoints) {
        sequence.addAction(new SplinedPathAction(targetPose, name, controlPoints));
        return this;
    }

    /**
     * Adds a splined path action with auto-generated control points.
     */
    public SequenceBuilder moveSplineTo(Pose targetPose, String name) {
        sequence.addAction(new SplinedPathAction(targetPose, name));
        return this;
    }

    /**
     * Adds a curve path action with control points.
     */
    public SequenceBuilder moveCurveTo(Pose targetPose, String name, Pose... controlPoints) {
        sequence.addAction(new CurvePathAction(targetPose, name, controlPoints));
        return this;
    }

    /**
     * Adds a curve path action with control points and auto-generated name.
     */
    public SequenceBuilder moveCurveTo(Pose targetPose, Pose... controlPoints) {
        sequence.addAction(new CurvePathAction(targetPose, controlPoints));
        return this;
    }

    /**
     * Convenience method for single control point curves.
     */
    public SequenceBuilder moveCurveToVia(Pose targetPose, Pose controlPoint, String name) {
        sequence.addAction(CurvePathAction.withSingleControlPoint(targetPose, controlPoint, name));
        return this;
    }

    /**
     * Adds an end-at action to hold position at a target pose.
     */
    public SequenceBuilder endAt(Pose targetPose) {
        Pose finalPose = MatchState.isBlue
                ? targetPose
                : targetPose.mirror();
        sequence.addAction(new EndAtAction(finalPose));
        return this;
    }

    /**
     * Adds a wait action.
     *
     * @param seconds Duration to wait in seconds
     * @return this (for method chaining)
     */
    public SequenceBuilder wait(double seconds) {
        sequence.addAction(new WaitAction(seconds));
        return this;
    }

    /**
     * Adds a custom action.
     *
     * @param action The action to add
     * @return this (for method chaining)
     */
    public SequenceBuilder addAction(AutonomousAction action) {
        sequence.addAction(action);
        return this;
    }

    /**
     * Adds multiple actions that run in parallel.
     *
     * @param actions The actions to run simultaneously
     * @return this (for method chaining)
     */
    public SequenceBuilder parallel(AutonomousAction... actions) {
        sequence.addAction(new ParallelAction(actions));
        return this;
    }

    /**
     * Loops a sub-sequence of actions until a specified number of seconds
     * are left in the autonomous period.
     * <p>
     * Example usage:
     *
     * <pre>
     * .loopUntilSecondsLeft(5, loop -> loop
     *     .moveTo(Settings.Positions.Samples.HumanPlayerPreset.PREP)
     *     .startPickup()
     *     .moveTo(Settings.Positions.Samples.HumanPlayerPreset.END)
     *     .prepLaunch()
     *     .moveTo(Settings.Positions.TeleOp.FAR_SHOOT)
     *     .launch()
     * )
     * </pre>
     *
     * @param secondsToLeave Number of seconds to leave remaining before exiting the
     *                       loop
     * @param loopBuilder    A consumer that builds the loop sequence
     * @return this (for method chaining)
     */
    public SequenceBuilder loopUntilSecondsLeft(double secondsToLeave, Consumer<SequenceBuilder> loopBuilder) {
        SequenceBuilder subBuilder = new SequenceBuilder();
        loopBuilder.accept(subBuilder);
        sequence.addAction(new LoopAction(secondsToLeave, subBuilder.build()));
        return this;
    }

    /**
     * Builds the sequence.
     *
     * @return The completed autonomous sequence
     */
    public AutonomousSequence build() {
        return sequence;
    }
}
