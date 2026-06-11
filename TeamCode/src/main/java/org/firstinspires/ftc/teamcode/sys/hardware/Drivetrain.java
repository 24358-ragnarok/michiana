package org.firstinspires.ftc.teamcode.sys.hardware;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Settings;

/**
 * Represents a Butterfly Drivetrain that can switch between Mecanum and Tank drive modes.
 * <p>
 * This class manages the PedroPathing follower and the servo mechanism used to engage/disengage
 * the traction wheels for tank drive.
 */
public class Drivetrain {
    private final HardwareMap hardwareMap;
    /**
     * The PedroPathing follower instance used for path following and teleop control.
     */
    public Follower follower;

    /**
     * Initializes the Butterfly Drivetrain.
     * <p>
     * Creates the initial Mecanum follower and initializes the butterfly servo.
     *
     * @param hardwareMap The hardware map.
     */
    public Drivetrain(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        follower = createMecanumFollower(hardwareMap);
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
}
