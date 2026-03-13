package org.firstinspires.ftc.teamcode.kotlin_mirror.hardware

import com.pedropathing.follower.Follower
import com.pedropathing.ftc.FollowerBuilder
import com.pedropathing.ftc.drivetrains.Mecanum
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous.TankDrivetrain
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.Settings
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.Settings.Drivetrain.MECANUM_DOWN_POSITION
import org.firstinspires.ftc.teamcode.kotlin_mirror.config.Settings.Drivetrain.TANK_DOWN_POSITION

/**
 * Represents a Butterfly Drivetrain that can switch between Mecanum and Tank drive modes.
 *
 * This class manages the PedroPathing follower and the servo mechanism used to engage/disengage
 * the traction wheels for tank drive.
 */
class ButterflyDrivetrain(private val hardwareMap: HardwareMap) {
    /**
     * The PedroPathing follower instance used for path following and teleop control.
     */
    @JvmField
    var follower: Follower

    private val butterfly: ServoImplEx

    /**
     * The current state of the drivetrain (MECANUM or TANK).
     */
    @JvmField
    var state: ButterflyState

    /**
     * Initializes the Butterfly Drivetrain.
     *
     * Creates the initial Mecanum follower and initializes the butterfly servo.
     */
    init {
        follower = createMecanumFollower(hardwareMap)
        butterfly = hardwareMap.get(ServoImplEx::class.java, Settings.Hardware.BUTTERFLY)
        state = ButterflyState.MECANUM
    }

    /**
     * Updates the follower. Should be called every loop.
     */
    fun update() {
        follower.update()
    }

    /**
     * Stops the follower and breaks any active path following.
     */
    fun stop() {
        follower.breakFollowing()
    }

    /**
     * Drives the robot in TeleOp mode.
     *
     * @param drive    The forward/backward power.
     * @param strafe   The left/right strafe power.
     * @param rotation The rotation power.
     */
    fun drive(drive: Double, strafe: Double, rotation: Double) {
        if (follower.isBusy) {
            follower.startTeleOpDrive()
        }
        follower.setTeleOpDrive(drive, strafe, rotation)
    }

    /**
     * Moves the robot to a specific pose and holds it there.
     *
     * If the robot is already moving to this pose, this method does nothing.
     *
     * @param preset The target pose to hold.
     */
    fun goToPreset(preset: Pose) {
        if (follower.isBusy && follower.currentPath.endPose() == preset) {
            return
        }

        follower.holdPoint(preset, false)
    }

    /**
     * Transforms the drivetrain between Mecanum and Tank modes.
     *
     * Adjusts the butterfly servo position and reconfigures the follower with the appropriate
     * drive constants and constraints.
     *
     * @param targetState The desired drivetrain state.
     */
    fun transform(targetState: ButterflyState) {
        if (targetState == state) {
            return
        }
        if (targetState == ButterflyState.MECANUM) {
            butterfly.position = MECANUM_DOWN_POSITION
            follower.pathConstraints = Settings.PedroPathing.Path.MECANUM
            follower.constants = Settings.PedroPathing.Follower.MECANUM
            follower.drivetrain = Mecanum(hardwareMap, Settings.PedroPathing.Drive.MECANUM)
        }
        if (targetState == ButterflyState.TANK) {
            butterfly.position = TANK_DOWN_POSITION
            follower.pathConstraints = Settings.PedroPathing.Path.TANK
            follower.constants = Settings.PedroPathing.Follower.TANK
            follower.drivetrain = TankDrivetrain(hardwareMap, Settings.PedroPathing.Drive.TANK)
        }
        state = targetState
    }

    /**
     * Creates a new Follower instance configured for Mecanum drive.
     *
     * @param hardwareMap The hardware map.
     * @return A configured Follower instance.
     */
    private fun createMecanumFollower(hardwareMap: HardwareMap): Follower {
        return FollowerBuilder(Settings.PedroPathing.Follower.MECANUM, hardwareMap)
            .mecanumDrivetrain(Settings.PedroPathing.Drive.MECANUM)
            .pathConstraints(Settings.PedroPathing.Path.MECANUM)
            .pinpointLocalizer(Settings.PedroPathing.Localizer.PINPOINT)
            .build()
    }

    /**
     * Enum representing the possible states of the butterfly drivetrain.
     */
    enum class ButterflyState {
        MECANUM,
        TANK
    }
}
