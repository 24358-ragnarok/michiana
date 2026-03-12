package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * An initialization action that sets the robot's starting pose.
 * <p>
 * This action should be the first one in any autonomous sequence. It tells the follower
 * where the robot is starting on the field and holds that position to prevent drift
 * before the first movement command.
 */
public class StartAtAction implements AutonomousAction {
    private final Pose targetPose;
    private final String name;
    private final boolean isBlue;

    /**
     * Creates a new StartAtAction.
     *
     * @param targetPose The starting pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     * @param isBlue     True for BLUE alliance, false for RED.
     */
    public StartAtAction(Pose targetPose, String name, boolean isBlue) {
        this.targetPose = targetPose;
        this.name = name;
        this.isBlue = isBlue;
    }

    /**
     * Creates a new StartAtAction using global match state.
     *
     * @param targetPose The starting pose in BLUE alliance coordinates.
     * @param name       A descriptive name for the action.
     */
    public StartAtAction(Pose targetPose, String name) {
        this(targetPose, name, MatchState.isBlue);
    }

    /**
     * Creates a new StartAtAction with a default name.
     *
     * @param targetPose The starting pose in BLUE alliance coordinates.
     */
    public StartAtAction(Pose targetPose) {
        this(targetPose, "StartAt");
    }

    @Override
    public void initialize(Robot bot) {
        Pose actualPose = isBlue
                ? targetPose
                : targetPose.mirror();

        // Initialize the follower's pose estimate
        bot.dt.follower.setStartingPose(actualPose);
        // Actively hold this position
        bot.dt.follower.holdPoint(actualPose);
    }

    @Override
    public boolean execute(Robot bot) {
        // Initialization is instantaneous
        return true;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        // Continue holding position until the next action takes over
    }

    @Override
    public String getName() {
        return name;
    }
}
