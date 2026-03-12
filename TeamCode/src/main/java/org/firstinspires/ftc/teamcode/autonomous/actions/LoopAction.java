package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousSequence;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.hardware.Robot;

/**
 * Action that loops a sub-sequence of actions until a specified number of
 * seconds
 * are left in the autonomous period.
 * <p>
 * This enables time-aware looping strategies like:
 * - Repeatedly collecting and launching balls from human player until 5 seconds
 * left
 * - Running a cycle as many times as possible before needing to park
 * <p>
 * The loop will complete its current iteration before checking if time has
 * expired.
 * To guarantee exit time, ensure individual loop actions are short enough.
 */
public class LoopAction implements AutonomousAction {

    private final double secondsToLeave;
    private final AutonomousSequence loopSequence;
    private int loopCount;

    /**
     * Creates a new loop action.
     *
     * @param secondsToLeave Number of seconds to leave remaining when stopping the
     *                       loop
     * @param loopSequence   The sequence of actions to loop
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
        double timeLeft = Settings.Autonomous.DURATION - bot.elapsedTime;

        // Check if we should stop looping (not enough time left)
        if (timeLeft <= secondsToLeave) {
            return true; // Exit the loop
        }

        // Update the loop sequence
        loopSequence.update(bot);

        // If the loop sequence completed, restart it for another iteration
        if (loopSequence.isComplete()) {
            loopCount++;
            loopSequence.start(bot);
        }

        return false; // Continue looping
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
        // No timeout - the loop is time-managed by secondsToLeave
        return 0.0;
    }
}
