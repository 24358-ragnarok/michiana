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
        timer.resetTimer();
    }

    @Override
    public boolean execute(Robot bot) {
        if (timer.getElapsedTime() > Settings.Autonomous.LAUNCH_CALIBRATION_MS) {
            bot.turret.getTung().open();
        }
        return timer.getElapsedTime() > Settings.Autonomous.LAUNCH_DURATION_MS;
    }

    @Override
    public void end(Robot bot, boolean interrupted) {
        bot.turret.getTung().close();
    }

    @Override
    public String getName() {
        return "Launch";
    }
}
