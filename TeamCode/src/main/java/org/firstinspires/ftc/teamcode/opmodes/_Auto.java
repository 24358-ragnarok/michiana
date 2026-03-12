package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.autonomous.AutonomousRuntime;
import org.firstinspires.ftc.teamcode.autonomous.AutonomousSequence;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Wizard;

/**
 * The main Autonomous mode.
 * <p>
 * This OpMode executes the pre-programmed autonomous routine selected via the
 * configuration wizard. It uses the {@link org.firstinspires.ftc.teamcode.autonomous.AutonomousSequence}
 * system to run actions.
 */
@Autonomous(name = "Revenge of the Boonstra", preselectTeleOp = "Goatpak Strikes Back")
public class _Auto extends OpMode {
    /**
     * The robot hardware interface.
     */
    public Robot bot;
    public AutonomousSequence auto;

    /**
     * A configuration wizard to handle initial telemetry.
     */
    public Wizard wizard;

    /**
     * Initializes the robot and subsystems.
     */
    @Override
    public void init() {
        MatchState.prepForAuto();
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
        wizard = new Wizard(bot);
    }

    /**
     * Runs constantly after initialization but before run.
     */
    @Override
    public void init_loop() {
        // bot.log.drawRobot(MatchState.getAutonomousRuntime());
        wizard.refresh();
        bot.log.update();
    }

    /**
     * Runs at the beginning of the mode play.
     */
    @Override
    public void start() {
        bot.dt.follower.setStartingPose(MatchState.getStartsFar() ?
                Settings.Positions.BotPoses.START_FAR : Settings.Positions.BotPoses.START_CLOSE);
        bot.start();

        AutonomousRuntime runtime = MatchState.getAutonomousRuntime();
        if (MatchState.getStartsFar()) {
            auto = runtime.buildFarSequence();
        } else {
            auto = runtime.buildCloseSequence();
        }

        // Start the sequence
        auto.start(bot);
    }

    /**
     * Main autonomous loop.
     * <p>
     * Updates the robot state and the autonomous sequence.
     */
    @Override
    public void loop() {
        bot.update(time);
        auto.update(bot);
    }

    /**
     * Stops the robot when the OpMode is stopped.
     */
    @Override
    public void stop() {
        MatchState.storedPose = bot.dt.follower.getPose();
        bot.stop();
    }
}
