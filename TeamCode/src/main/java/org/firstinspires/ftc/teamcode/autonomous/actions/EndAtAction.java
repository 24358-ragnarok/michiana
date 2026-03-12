package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Action that follows a PathChain using the drivetrain's path follower.
 */
public class EndAtAction implements AutonomousAction {
    private final Pose pose;
    private final String name;

    public EndAtAction(Pose pose, String name) {
        this.pose = pose;
        this.name = name;
    }

    public EndAtAction(Pose pose) {
        this(pose, "EndAt");
    }


    @Override
    public void initialize(Robot bot) {
        bot.dt.follower.holdPoint(pose);
    }

    @Override
    public boolean execute(Robot bot) {
        // End actions never complete
        return false;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        // never ends
    }

    @Override
    public String getName() {
        return name;
    }
}
