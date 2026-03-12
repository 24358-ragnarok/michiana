package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * An action that does nothing but wait for a specified duration.
 * <p>
 * Useful for adding delays between actions, allowing mechanisms to settle, or
 * synchronizing with alliance partners.
 */
public class WaitAction implements AutonomousAction {
    private final double durationSeconds;
    private Timer timer;

    /**
     * Creates a new WaitAction.
     *
     * @param durationSeconds The time to wait in seconds.
     */
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
        // No cleanup needed
    }

    @Override
    public String getName() {
        return "Wait(" + durationSeconds + "s)";
    }
}
