package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.telemetry.Logging;

/**
 * The central hardware abstraction class for the robot.
 * <p>
 * This class initializes and manages all hardware subsystems, including the drivetrain,
 * controllers, and logging. It also handles the bulk caching mode for the Lynx modules
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
    public final ButterflyDrivetrain dt;

    /**
     * The elapsed time since the last update, in seconds.
     */
    public volatile double elapsedTime;

    /**
     * Initializes the robot hardware and subsystems.
     * <p>
     * Sets up PhotonCore for optimized bulk caching, initializes the logging system,
     * wraps the gamepads in a Controller instance, and initializes the drivetrain.
     *
     * @param hardwareMap The hardware map from the OpMode.
     * @param telemetry   The telemetry instance from the OpMode.
     * @param gamepad1    The first gamepad.
     * @param gamepad2    The second gamepad.
     */
    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        // Configure bulk caching to MANUAL mode for better performance
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();

        log = new Logging(telemetry);
        ctrl = new Controller(gamepad1, gamepad2);
        dt = new ButterflyDrivetrain(hardwareMap);
    }

    /**
     * Updates the robot's state.
     * <p>
     * This method should be called once per loop iteration. It clears the bulk cache,
     * updates the drivetrain, and updates the telemetry logging.
     *
     * @param time The current time in seconds.
     */
    public void update(double time) {
        elapsedTime = time;
        // Clear the bulk cache to ensure fresh data for this loop iteration
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();

        dt.update();
        log.update(dt.follower.getPose());
    }

    /**
     * Start all systems. This begins processes restrained by not being able to move during init.
     */
    public void start() {
        dt.start();
    }

    /**
     * Stops the robot and all its subsystems.
     */
    public void stop() {
        dt.stop();
    }
}
