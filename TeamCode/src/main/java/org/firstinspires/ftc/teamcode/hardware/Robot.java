package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.photon.PhotonCore;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Controller;
import org.firstinspires.ftc.teamcode.util.Logging;

public class Robot {
    public final Logging log;
    public final Controller ctrl;
    public final ButterflyDrivetrain dt;


    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        PhotonCore.CONTROL_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.EXPANSION_HUB.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        PhotonCore.enable();
        log = new Logging(telemetry);
        ctrl = new Controller(gamepad1, gamepad2);
        dt = new ButterflyDrivetrain(hardwareMap);
    }

    public void update() {
        PhotonCore.CONTROL_HUB.clearBulkCache();
        PhotonCore.EXPANSION_HUB.clearBulkCache();
        dt.update();
        log.update(dt.follower.getPose());
    }

    public void stop() {
        dt.stop();
    }
}
