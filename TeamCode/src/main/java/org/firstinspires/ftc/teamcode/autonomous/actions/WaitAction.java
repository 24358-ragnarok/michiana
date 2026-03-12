package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Action that waits for a specified duration before completing.
 */
public class WaitAction implements AutonomousAction {
    private final double durationSeconds;
    private Timer timer;

    public WaitAction(double durationSeconds) {
        this.durationSeconds = durationSeconds;
    }

    @Override
    public void initialize(Robot bot) {
        timer = new Timer();
        timer.resetTimer();
    }

    @Override
    public boolean execute(Robot bot) {
        return timer.getElapsedTimeSeconds() >= durationSeconds;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        // Nothing to clean up
    }

    @Override
    public String getName() {
        return "Wait(" + durationSeconds + "s)";
    }
}
