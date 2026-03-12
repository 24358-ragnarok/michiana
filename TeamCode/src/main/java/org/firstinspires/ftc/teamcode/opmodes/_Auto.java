package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Autonomous(name = "Revenge of the Boonstra", preselectTeleOp = "Goatpak Strikes Back")
public class _Auto extends OpMode {
    public Robot bot;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        bot.update();
    }

    @Override
    public void stop() {
        bot.stop();
    }
}
