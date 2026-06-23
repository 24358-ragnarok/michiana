package org.firstinspires.ftc.teamcode.sys;

import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.config.MatchState;
import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.sys.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.sys.hardware.Intake;
import org.firstinspires.ftc.teamcode.sys.hardware.Turret;
import org.firstinspires.ftc.teamcode.sys.software.Peripherals;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.telemetry.Logging;

import java.util.Locale;

/**
 * The central hardware abstraction class for the robot.
 * <p>
 * This class initializes and manages all hardware subsystems, including the
 * drivetrain,
 * controllers, and logging. It also handles the bulk caching mode for the Lynx
 * modules
 * to optimize loop times.
 */
public class Robot {
    /**
     * The unified logging system for telemetry.
     */
    public final Logging log;

    /**
     * The controller wrapper for handling gamepad inputs.
     */
    public final Controller ctrl;

    /**
     * The drivetrain subsystem, supporting both Mecanum and Tank drive modes.
     */
    public final Drivetrain dt;

    /**
     * You know, because we're fancy like that.
     */
    public final Peripherals peripherals;
    /**
     * To launch the balls...
     */
    public final Turret turret;
    /**
     * To get the balls
     */
    public final Intake intake;
    /**
     * The elapsed time since the start of the OpMode, in seconds.
     */
    public volatile double elapsedTime;
    /**
     * The time at which the OpMode was started.
     */
    private double startTime;

    /**
     * Initializes the robot hardware and subsystems.
     * <p>
     * Sets up PhotonCore for optimized bulk caching, initializes the logging
     * system,
     * wraps the gamepads in a Controller instance, and initializes the drivetrain.
     *
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry   The telemetry instance from the OpMode.
     * @param gamepad1    The first gamepad.
     * @param gamepad2    The second gamepad.
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        log = new Logging(telemetry);
        ctrl = new Controller(gamepad1, gamepad2);
        dt = new Drivetrain(hardwareMap);
        turret = new Turret(hardwareMap);
        intake = new Intake(hardwareMap);
        peripherals = new Peripherals(hardwareMap, telemetry);
        log.finishSetup();
    }

    /**
     * Updates the robot's state.
     * <p>
     * This method should be called once per loop iteration. It clears the bulk
     * cache,
     * updates the drivetrain, and updates the telemetry logging.
     *
     * @param time The current time in seconds (relative to init).
     */
    public void update(double time) {
        elapsedTime = time - startTime;
        peripherals.update();
        dt.update();
        turret.update(dt.follower.getPose(), dt.follower.getVelocity());
        log.update(dt.follower);

        log.setItem("<b>Status</b>", turret.isReady() ? "<font color='#00FF00'>READY TO FIRE</font>" : "<font color='#FF0000'>PREPARING</font>");
        log.setItem("Intake", botVelocityToColorString(dt.follower.getVelocity())); // Just an example
    }

    private String botVelocityToColorString(Vector velocity) {
        return String.format(Locale.getDefault(), "%.1f, %.1f", velocity.getXComponent(), velocity.getYComponent());
    }

    /**
     * Start all systems. This begins processes restrained by not being able to move
     * during init.
     */
    public void start(double time) {
        this.startTime = time;
        dt.start();
        Turret.setTargetPose(
                MatchState.isBlue ? Settings.Positions.Towers.BLUE_GOAL : Settings.Positions.Towers.RED_GOAL);
        turret.start();
        intake.start();
    }

    /**
     * Start all systems.
     */
    public void start() {
        start(0);
    }

    /**
     * Stops the robot and all its subsystems.
     */
    public void stop() {
        dt.stop();
        turret.stop();
    }

    static class RobotInitializationError extends RuntimeException {
        public RobotInitializationError(Throwable m) {
            super(m);
        }
    }
}
