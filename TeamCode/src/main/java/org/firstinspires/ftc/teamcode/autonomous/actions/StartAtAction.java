package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Initialization action that locks the drivetrain to a starting pose.
 * Sets a hold point immediately to eliminate race conditions between
 * pose initialization and the first movement action.
 */
public class StartAtAction implements AutonomousAction {
    private final Pose targetPose;
    private final String name;
    private final boolean isBlue;

    public StartAtAction(Pose targetPose, String name, boolean isBlue) {
        this.targetPose = targetPose;
        this.name = name;
        this.isBlue = isBlue;
    }

    public StartAtAction(Pose targetPose, String name) {
        this(targetPose, name, MatchState.isBlue);
    }

    public StartAtAction(Pose targetPose) {
        this(targetPose, "StartAt");
    }

    @Override
    public void initialize(Robot bot) {
        Pose actualPose = isBlue
                ? targetPose
                : targetPose.mirror();

        // Ensure follower starts at the desired pose and holds position
        bot.dt.follower.setStartingPose(actualPose);
        bot.dt.follower.holdPoint(actualPose);
    }

    @Override
    public boolean execute(Robot bot) {
        // Nothing to wait on; move to next action immediately.
        return true;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        // No cleanup required; holdPoint maintains position if still running.
    }

    @Override
    public String getName() {
        return name;
    }
}
