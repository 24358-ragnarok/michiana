package org.firstinspires.ftc.teamcode.autonomous.actions;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousAction;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.sys.Robot;

/**
 * Action that launches all samples by opening the Tung gate for a set duration.
 */
public class LaunchAction implements AutonomousAction {
    private Timer timer;

    @Override
    public void initialize(Robot bot) {
        timer = new Timer();
        bot.turret.getTung().open();
        bot.intake.in();
        timer.resetTimer();
    }

    @Override
    public boolean execute(Robot bot) {
        return timer.getElapsedTime() > Settings.Autonomous.LAUNCH_DURATION_MS;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        bot.turret.getTung().close();
        bot.intake.stop();
    }

    @Override
    public String getName() {
        return "Launch";
    }
}
