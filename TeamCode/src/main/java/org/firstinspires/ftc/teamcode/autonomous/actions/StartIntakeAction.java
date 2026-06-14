package org.firstinspires.ftc.teamcode.autonomous.actions;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * An action that starts the intake mechanism.
 * This action completes immediately after starting the intake.
 */
public class StartIntakeAction implements AutonomousAction {
    private final boolean reverse;

    public StartIntakeAction() {
        this(false);
    }

    public StartIntakeAction(boolean reverse) {
        this.reverse = reverse;
    }

    @Override
    public void initialize(Robot bot) {
        if (reverse) {
            bot.intake.out();
        } else {
            bot.intake.in();
        }
    }

    @Override
    public boolean execute(Robot bot) {
        return true; // Completes immediately
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        // No cleanup needed, intake stays running
    }
}
