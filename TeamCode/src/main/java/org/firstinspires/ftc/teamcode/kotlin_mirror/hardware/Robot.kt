package org.firstinspires.ftc.teamcode.kotlin_mirror.hardware

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import com.seattlesolvers.solverslib.photon.PhotonCore
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.Controller
import org.firstinspires.ftc.teamcode.kotlin_mirror.util.telemetry.Logging

/**
 * The central hardware abstraction class for the robot.
 *
 * This class initializes and manages all hardware subsystems, including the drivetrain,
 * controllers, and logging. It also handles the bulk caching mode for the Lynx modules
 * to optimize loop times.
 */
class Robot(
    hardwareMap: HardwareMap,
    telemetry: Telemetry,
    gamepad1: Gamepad,
    gamepad2: Gamepad
) {
    /**
     * The unified logging system for telemetry.
     */
    @JvmField
    val log: Logging

    /**
     * The controller wrapper for handling gamepad inputs.
     */
    @JvmField
    val ctrl: Controller

    /**
     * The drivetrain subsystem, supporting both Mecanum and Tank drive modes.
     */
    @JvmField
    val dt: ButterflyDrivetrain

    /**
     * The elapsed time since the last update, in seconds.
     */
    @Volatile
    var elapsedTime: Double = 0.0

    /**
     * Initializes the robot hardware and subsystems.
     *
     * Sets up PhotonCore for optimized bulk caching, initializes the logging system,
     * wraps the gamepads in a Controller instance, and initializes the drivetrain.
     */
    init {
        // Configure bulk caching to MANUAL mode for better performance
        PhotonCore.CONTROL_HUB.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        PhotonCore.EXPANSION_HUB.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        PhotonCore.enable()

        log = Logging(telemetry)
        ctrl = Controller(gamepad1, gamepad2)
        dt = ButterflyDrivetrain(hardwareMap)
    }

    /**
     * Starts robot subsystems that need explicit begin-of-opmode initialization.
     */
    fun start() {
        log.finishSetup()
    }

    /**
     * Updates the robot's state.
     *
     * This method should be called once per loop iteration. It clears the bulk cache,
     * updates the drivetrain, and updates the telemetry logging.
     *
     * @param time The current time in seconds.
     */
    fun update(time: Double) {
        elapsedTime = time
        // Clear the bulk cache to ensure fresh data for this loop iteration
        PhotonCore.CONTROL_HUB.clearBulkCache()
        PhotonCore.EXPANSION_HUB.clearBulkCache()

        dt.update()
        log.update(dt.follower.pose)
    }

    /**
     * Stops the robot and all its subsystems.
     */
    fun stop() {
        dt.stop()
    }
}
