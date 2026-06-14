package org.firstinspires.ftc.teamcode.sys.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.Settings;

public class Intake {
    public final DcMotorEx motor;

    public Intake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, Settings.Hardware.INTAKE_MOTOR);
    }

    public void start() {
    }

    public void in() {
        motor.setPower(1);
    }

    public void out() {
        motor.setPower(-1);
    }

    public void stop() {
        motor.setPower(0);
    }
}
