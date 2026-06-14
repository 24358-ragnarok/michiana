package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * An action that stops the intake mechanism.
 * This action completes immediately.
 */
public class StopIntakeAction implements AutonomousAction {
    @Override
    public void initialize(Robot bot) {
        bot.intake.stop();
    }

    @Override
    public boolean execute(Robot bot) {
        return true;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
    }
}
