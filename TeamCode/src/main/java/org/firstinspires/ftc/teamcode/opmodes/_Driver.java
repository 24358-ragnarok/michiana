package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@TeleOp(name = "Goatpak Strikes Back", group = "0: Competition Modes")
public class _Driver extends OpMode {
    public Robot bot;

    @Override
    public void init() {
        bot = new Robot(hardwareMap, telemetry, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        bot.update(time);
        bot.dt.drive(bot.ctrl.main.leftStickY().state(), bot.ctrl.main.leftStickX().state(), bot.ctrl.main.rightStickX().state());
    }

    @Override
    public void stop() {
        bot.stop();
    }
}
