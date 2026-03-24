package org.firstinspires.ftc.teamcode.sys.software;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class YawThroughBoreEncoder {
    private static final double EPSILON = 1e-6;

    private final DcMotorEx encoderMotor;
    private final double zeroTicks;
    private final double ticksPerRev;
    private final double direction;

    public YawThroughBoreEncoder(
            DcMotorEx encoderMotor,
            double zeroTicks,
            double ticksPerRev,
            double direction) {
        this.encoderMotor = encoderMotor;
        this.zeroTicks = zeroTicks;
        this.ticksPerRev = Math.max(Math.abs(ticksPerRev), EPSILON);
        this.direction = direction == 0.0 ? 1.0 : Math.signum(direction);
    }

    public double getYawAngleRadians() {
        double ticks = encoderMotor.getCurrentPosition();
        double revolutions = (ticks - zeroTicks) / ticksPerRev;
        return revolutions * (2.0 * Math.PI) * direction;
    }
}
