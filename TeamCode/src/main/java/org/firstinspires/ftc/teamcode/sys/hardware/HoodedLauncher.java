package org.firstinspires.ftc.teamcode.sys.hardware;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Settings;

public class HoodedLauncher {
    private static Pose targetPose = Settings.Positions.TeleopPresets.FAR_SHOOT;

    private final ServoImplEx hoodServo;
    private final YawUnion yaw;
    private final FlywheelUnion flywheel;
    private Settings.Launcher.ShotModel shotModel = Settings.Launcher.SHOT_MODEL;

    public HoodedLauncher(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(ServoImplEx.class, Settings.Hardware.HOOD);
        CRServo yawR = hardwareMap.get(CRServo.class, Settings.Hardware.YAW_R);
        CRServo yawL = hardwareMap.get(CRServo.class, Settings.Hardware.YAW_L);

        DcMotorEx flywheelR = hardwareMap.get(DcMotorEx.class, Settings.Hardware.FLYWHEEL_R);
        DcMotorEx flywheelL = hardwareMap.get(DcMotorEx.class, Settings.Hardware.FLYWHEEL_L);

        yaw = new YawUnion(
                yawR,
                yawL,
                Settings.Launcher.YAW_PIDF_R,
                Settings.Launcher.YAW_PIDF_L,
                Settings.Launcher.YAW_MAX_POWER);
        flywheel = new FlywheelUnion(
                flywheelR,
                flywheelL,
                Settings.Launcher.GEAR_RATIO_MOTOR_TO_FLYWHEEL);

        flywheel.configureMotorPid(
                Settings.Launcher.PIDF_R,
                Settings.Launcher.PIDF_L);
    }

    public static Pose getTargetPose() {
        return targetPose;
    }

    public static void setTargetPose(Pose targetPose) {
        HoodedLauncher.targetPose = targetPose;
    }

    private static double normalizeRadians(double angleRadians) {
        while (angleRadians > Math.PI) {
            angleRadians -= (2.0 * Math.PI);
        }
        while (angleRadians < -Math.PI) {
            angleRadians += (2.0 * Math.PI);
        }
        return angleRadians;
    }

    public void update(Pose botPose) {
        if (botPose == null || targetPose == null) {
            return;
        }

        double dx = targetPose.getX() - botPose.getX();
        double dy = targetPose.getY() - botPose.getY();
        double distanceToTarget = Math.hypot(dx, dy);
        double fieldAngleToTarget = Math.atan2(dy, dx);
        double robotRelativeYawTarget = normalizeRadians(fieldAngleToTarget - botPose.getHeading());

        Settings.Launcher.ShotSolution solution = shotModel.solve(distanceToTarget);

        yaw.setTargetAngleRadians(robotRelativeYawTarget);
        yaw.update();

        setHoodAngleRadians(solution.hoodAngleRadians);
        flywheel.setTargetFlywheelVelocity(solution.flywheelVelocity);
        flywheel.update();
    }

    public void setShotModel(Settings.Launcher.ShotModel shotModel) {
        if (shotModel != null) {
            this.shotModel = shotModel;
        }
    }

    public void setYawEncoder(YawAngleReader yawAngleReader) {
        yaw.setYawAngleReader(yawAngleReader);
    }

    public void setHoodAngleRadians(double hoodAngleRadians) {
        double clamped = Range.clip(
                hoodAngleRadians,
                Settings.Launcher.HOOD_MIN_ANGLE_RAD,
                Settings.Launcher.HOOD_MAX_ANGLE_RAD);
        double normalized = (clamped - Settings.Launcher.HOOD_MIN_ANGLE_RAD)
                / (Settings.Launcher.HOOD_MAX_ANGLE_RAD - Settings.Launcher.HOOD_MIN_ANGLE_RAD);
        double servoPosition = Settings.Launcher.HOOD_MIN_SERVO_POSITION
                + (normalized
                * (Settings.Launcher.HOOD_MAX_SERVO_POSITION - Settings.Launcher.HOOD_MIN_SERVO_POSITION));
        hoodServo.setPosition(Range.clip(servoPosition, 0.0, 1.0));
    }

    public void setFlywheelVelocity(double flywheelVelocity) {
        flywheel.setTargetFlywheelVelocity(flywheelVelocity);
    }

    public void stop() {
        yaw.stop();
        flywheel.stop();
    }

    public interface YawAngleReader {
        double getYawAngleRadians();
    }

    public static class YawUnion {
        private final CRServo yawRight;
        private final CRServo yawLeft;
        private final ElapsedTime loopTimer = new ElapsedTime();
        private final PIDFCoefficients rightPidf;
        private final PIDFCoefficients leftPidf;
        private final double maxPower;

        private YawAngleReader yawAngleReader;
        private double targetAngleRadians = 0.0;
        private double rightIntegral = 0.0;
        private double leftIntegral = 0.0;
        private double rightPreviousError = 0.0;
        private double leftPreviousError = 0.0;

        public YawUnion(
                CRServo yawRight,
                CRServo yawLeft,
                PIDFCoefficients rightPidf,
                PIDFCoefficients leftPidf,
                double maxPower) {
            this.yawRight = yawRight;
            this.yawLeft = yawLeft;
            this.yawLeft.setDirection(DcMotorSimple.Direction.REVERSE);
            this.rightPidf = rightPidf;
            this.leftPidf = leftPidf;
            this.maxPower = Math.abs(maxPower);
            loopTimer.reset();
        }

        private static double calculatePidfOutput(
                PIDFCoefficients pidf,
                double error,
                double integral,
                double derivative) {
            double fTerm = pidf.f * Math.signum(error);
            return (pidf.p * error) + (pidf.i * integral) + (pidf.d * derivative) + fTerm;
        }

        public void setYawAngleReader(YawAngleReader yawAngleReader) {
            this.yawAngleReader = yawAngleReader;
        }

        public void setTargetAngleRadians(double targetAngleRadians) {
            this.targetAngleRadians = normalizeRadians(targetAngleRadians);
        }

        public void update() {
            if (yawAngleReader == null) {
                yawRight.setPower(0.0);
                yawLeft.setPower(0.0);
                return;
            }

            double dt = Math.max(loopTimer.seconds(), 1e-3);
            loopTimer.reset();

            double currentAngle = yawAngleReader.getYawAngleRadians();
            double error = normalizeRadians(targetAngleRadians - currentAngle);

            rightIntegral += error * dt;
            leftIntegral += error * dt;

            double rightDerivative = (error - rightPreviousError) / dt;
            double leftDerivative = (error - leftPreviousError) / dt;
            rightPreviousError = error;
            leftPreviousError = error;

            double rightOutput = calculatePidfOutput(rightPidf, error, rightIntegral, rightDerivative);
            double leftOutput = calculatePidfOutput(leftPidf, error, leftIntegral, leftDerivative);

            // Both servos share the same axle, so apply a unified command.
            double commandedPower = Range.clip((rightOutput + leftOutput) * 0.5, -maxPower, maxPower);

            yawRight.setPower(commandedPower);
            yawLeft.setPower(commandedPower);
        }

        public void stop() {
            yawRight.setPower(0.0);
            yawLeft.setPower(0.0);
            rightIntegral = 0.0;
            leftIntegral = 0.0;
            rightPreviousError = 0.0;
            leftPreviousError = 0.0;
        }
    }

    public static class FlywheelUnion {
        private final DcMotorEx rightMotor;
        private final DcMotorEx leftMotor;
        private final double motorToFlywheelGearRatio;
        private double targetFlywheelVelocity = 0.0;

        public FlywheelUnion(
                DcMotorEx rightMotor,
                DcMotorEx leftMotor,
                double motorToFlywheelGearRatio) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.motorToFlywheelGearRatio = motorToFlywheelGearRatio <= 0.0
                    ? 1.0
                    : motorToFlywheelGearRatio;
            this.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public void configureMotorPid(
                com.qualcomm.robotcore.hardware.PIDFCoefficients rightPidf,
                com.qualcomm.robotcore.hardware.PIDFCoefficients leftPidf) {
            rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, rightPidf);
            leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, leftPidf);
        }

        public void setTargetFlywheelVelocity(double targetFlywheelVelocity) {
            this.targetFlywheelVelocity = targetFlywheelVelocity;
        }

        public void update() {
            double targetMotorVelocity = targetFlywheelVelocity * motorToFlywheelGearRatio;
            rightMotor.setVelocity(targetMotorVelocity);
            leftMotor.setVelocity(targetMotorVelocity);
        }

        public void stop() {
            rightMotor.setVelocity(0.0);
            leftMotor.setVelocity(0.0);
        }
    }
}
