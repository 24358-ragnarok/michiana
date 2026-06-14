package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * An action that performs another action while the intake is running,
 * then stops the intake once that action completes.
 * Useful for "Intake while driving to point X".
 */
public class WithIntakeAction implements AutonomousAction {
    private final AutonomousAction action;

    public WithIntakeAction(AutonomousAction action) {
        this.action = action;
    }

    @Override
    public void initialize(Robot bot) {
        bot.intake.in();
        action.initialize(bot);
    }

    @Override
    public boolean execute(Robot bot) {
        return action.execute(bot);
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        action.end(bot, interrupted);
        bot.intake.stop();
    }
}
