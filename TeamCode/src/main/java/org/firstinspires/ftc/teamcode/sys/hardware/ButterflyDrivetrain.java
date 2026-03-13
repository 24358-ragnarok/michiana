package org.firstinspires.ftc.teamcode.sys.hardware;

import static org.firstinspires.ftc.teamcode.config.Settings.Drivetrain.LEFT_MECANUM_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.config.Settings.Drivetrain.LEFT_TANK_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.config.Settings.Drivetrain.RIGHT_MECANUM_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.config.Settings.Drivetrain.RIGHT_TANK_DOWN_POSITION;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.autonomous.TankDrivetrain;
import org.firstinspires.ftc.teamcode.config.Settings;

/**
 * Represents a Butterfly Drivetrain that can switch between Mecanum and Tank drive modes.
 * <p>
 * This class manages the PedroPathing follower and the servo mechanism used to engage/disengage
 * the traction wheels for tank drive.
 */
public class ButterflyDrivetrain {
    private final ServoImplEx butterflyLeft;
    private final ServoImplEx butterflyRight;
    private final HardwareMap hardwareMap;
    /**
     * The PedroPathing follower instance used for path following and teleop control.
     */
    public Follower follower;
    /**
     * The current state of the drivetrain (MECANUM or TANK).
     */
    public ButterflyState state;

    /**
     * Initializes the Butterfly Drivetrain.
     * <p>
     * Creates the initial Mecanum follower and initializes the butterfly servo.
     *
     * @param hardwareMap The hardware map.
     */
    public ButterflyDrivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        follower = createMecanumFollower(hardwareMap);
        butterflyLeft = hardwareMap.get(ServoImplEx.class, Settings.Hardware.BUTTERFLY_LEFT);
        butterflyRight = hardwareMap.get(ServoImplEx.class, Settings.Hardware.BUTTERFLY_RIGHT);
        state = ButterflyState.MECANUM;
    }

    /**
     * Updates the follower. Should be called every loop.
     */
    public void update() {
        follower.update();
    }

    /**
     * Runs as the game begins.
     */
    public void start() {
        butterflyLeft.setPosition(LEFT_MECANUM_DOWN_POSITION);
        butterflyRight.setPosition(RIGHT_MECANUM_DOWN_POSITION);
    }

    /**
     * Stops the follower and breaks any active path following.
     */
    public void stop() {
        follower.breakFollowing();
    }

    /**
     * Drives the robot in TeleOp mode.
     *
     * @param drive    The forward/backward power.
     * @param strafe   The left/right strafe power.
     * @param rotation The rotation power.
     */
    public void drive(double drive, double strafe, double rotation) {
        if (follower.isBusy()) {
            follower.startTeleOpDrive();
        }
        follower.setTeleOpDrive(drive, strafe, rotation);
    }

    /**
     * Moves the robot to a specific pose and holds it there.
     * <p>
     * If the robot is already moving to this pose, this method does nothing.
     *
     * @param preset The target pose to hold.
     */
    public void goToPreset(Pose preset) {
        if (follower.isBusy() && follower.getCurrentPath().endPose() == preset) {
            return;
        }

        follower.holdPoint(preset, false);
    }

    /**
     * Transforms the drivetrain between Mecanum and Tank modes.
     * <p>
     * Adjusts the butterfly servo position and reconfigures the follower with the appropriate
     * drive constants and constraints.
     *
     * @param targetState The desired drivetrain state.
     */
    public void transform(ButterflyState targetState) {
        if (targetState == state) {
            return;
        }
        if (targetState == ButterflyState.MECANUM) {
            butterflyLeft.setPosition(LEFT_MECANUM_DOWN_POSITION);
            butterflyRight.setPosition(RIGHT_MECANUM_DOWN_POSITION);
            follower.pathConstraints = Settings.PedroPathing.Path.MECANUM;
            follower.constants = Settings.PedroPathing.Follower.MECANUM;
            follower.drivetrain = new Mecanum(hardwareMap, Settings.PedroPathing.Drive.MECANUM);
        }
        if (targetState == ButterflyState.TANK) {
            butterflyLeft.setPosition(LEFT_TANK_DOWN_POSITION);
            butterflyRight.setPosition(RIGHT_TANK_DOWN_POSITION);
            follower.pathConstraints = Settings.PedroPathing.Path.TANK;
            follower.constants = Settings.PedroPathing.Follower.TANK;
            follower.drivetrain = new TankDrivetrain(hardwareMap, Settings.PedroPathing.Drive.TANK);
        }
        state = targetState;
    }

    /**
     * Creates a new Follower instance configured for Mecanum drive.
     *
     * @param hardwareMap The hardware map.
     * @return A configured Follower instance.
     */
    private Follower createMecanumFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(Settings.PedroPathing.Follower.MECANUM, hardwareMap)
                .mecanumDrivetrain(Settings.PedroPathing.Drive.MECANUM)
                .pathConstraints(Settings.PedroPathing.Path.MECANUM)
                .pinpointLocalizer(Settings.PedroPathing.Localizer.PINPOINT)
                .build();
    }

    /**
     * Enum representing the possible states of the butterfly drivetrain.
     */
    public enum ButterflyState {
        MECANUM,
        TANK
    }
}
