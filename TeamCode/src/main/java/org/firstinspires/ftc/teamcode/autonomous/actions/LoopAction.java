package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousSequence;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * An action that repeatedly executes a sub-sequence of actions until a specified time remains.
 * <p>
 * This is useful for maximizing the number of cycles (e.g., scoring samples) within the
 * autonomous period. The loop will complete the current iteration fully before checking
 * the time condition, ensuring the robot doesn't stop in an unsafe state.
 */
public class LoopAction implements AutonomousAction {

    private final double secondsToLeave;
    private final AutonomousSequence loopSequence;
    private int loopCount;

    /**
     * Creates a new LoopAction.
     *
     * @param secondsToLeave The minimum time (in seconds) that must remain to start another iteration.
     * @param loopSequence   The sequence of actions to be repeated.
     */
    public LoopAction(double secondsToLeave, AutonomousSequence loopSequence) {
        this.secondsToLeave = secondsToLeave;
        this.loopSequence = loopSequence;
        this.loopCount = 0;
    }

    @Override
    public void initialize(Robot bot) {
        loopCount = 0;
        loopSequence.start(bot);
    }

    @Override
    public boolean execute(Robot bot) {
        // Update the sub-sequence
        loopSequence.update(bot);

        // If the sub-sequence finished, check if we should restart it
        if (loopSequence.isComplete()) {
            double timeLeft = Settings.Autonomous.DURATION - bot.elapsedTime;
            if (timeLeft > secondsToLeave) {
                loopCount++;
                loopSequence.start(bot);
            } else {
                return true; // Stop looping
            }
        }

        return false; // Continue executing
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        loopSequence.stop(bot);
    }

    @Override
    public String getName() {
        return "Loop until " + secondsToLeave + "s " + "\n-> " + loopSequence.getCurrentActionName();
    }

    @Override
    public double getTimeoutSeconds() {
        // No timeout; the loop is controlled by the remaining match time
        return 0.0;
    }
}
