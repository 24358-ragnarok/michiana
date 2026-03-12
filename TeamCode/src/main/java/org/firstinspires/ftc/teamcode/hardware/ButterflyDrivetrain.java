package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.config.Settings.Drivetrain.MECANUM_DOWN_POSITION;
import static org.firstinspires.ftc.teamcode.config.Settings.Drivetrain.TANK_DOWN_POSITION;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.config.Settings;
import org.firstinspires.ftc.teamcode.pathing.tank.TankDrivetrain;

public class ButterflyDrivetrain {
    public Follower follower;
    public ServoImplEx butterfly;
    public HardwareMap hardwareMap;
    public ButterflyState state;

    public ButterflyDrivetrain(HardwareMap hardwareMap) {
        // default to holo
        follower = Settings.PedroPathing.Builder.mecanum(hardwareMap);
        butterfly = hardwareMap.get(ServoImplEx.class, Settings.Hardware.BUTTERFLY);
        state = ButterflyState.MECANUM;
        this.hardwareMap = hardwareMap;
    }

    public void update() {
        follower.update();
    }

    public void stop() {
        follower.breakFollowing();
    }

    public void drive(double drive, double strafe, double rotation) {
        if (follower.isBusy()) {
            follower.startTeleOpDrive();
        }
        follower.setTeleOpDrive(drive, strafe, rotation);
    }

    public void goToPreset(Pose preset) {
        if (follower.isBusy() && follower.getCurrentPath().endPose() == preset) {
            return;
        }

        follower.holdPoint(preset, false);
    }

    public void transform(ButterflyState targetState) {
        if (targetState == state) {
            return;
        }
        if (targetState == ButterflyState.MECANUM) {
            butterfly.setPosition(MECANUM_DOWN_POSITION);
            follower.pathConstraints = Settings.PedroPathing.Path.MECANUM;
            follower.constants = Settings.PedroPathing.Follower.MECANUM;
            follower.drivetrain = new Mecanum(hardwareMap, Settings.PedroPathing.Drive.MECANUM);
        }
        if (targetState == ButterflyState.TANK) {
            butterfly.setPosition(TANK_DOWN_POSITION);
            follower.pathConstraints = Settings.PedroPathing.Path.TANK;
            follower.constants = Settings.PedroPathing.Follower.TANK;
            follower.drivetrain = new TankDrivetrain(hardwareMap, Settings.PedroPathing.Drive.TANK);
        }
    }

    public enum ButterflyState {
        MECANUM,
        TANK
    }
}
