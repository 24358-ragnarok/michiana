package org.firstinspires.ftc.teamcode.sys.hardware;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
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
import org.firstinspires.ftc.teamcode.sys.software.YawThroughBoreEncoder;

public class HoodedLauncher {
    private static Pose targetPose = Settings.Positions.TeleopPresets.FAR_SHOOT;
    private static final double EPSILON = 1e-6;

    private final ServoImplEx hoodServo;
    private final YawUnion yaw;
    private final FlywheelUnion flywheel;
    private final Settings.Launcher.RPMSolutionModel rpmSolutionModel = Settings.Launcher.RPM_SOLUTION_MODEL;

    public HoodedLauncher(HardwareMap hardwareMap) {
        hoodServo = hardwareMap.get(ServoImplEx.class, Settings.Hardware.HOOD);
        CRServo yawR = hardwareMap.get(CRServo.class, Settings.Hardware.YAW_R);
        CRServo yawL = hardwareMap.get(CRServo.class, Settings.Hardware.YAW_L);
        DcMotorEx yawEncoder = hardwareMap.get(DcMotorEx.class, Settings.Hardware.YAW_ENCODER);

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
                Settings.Launcher.GEAR_RATIO_MOTOR_TO_FLYWHEEL,
                Settings.Launcher.FLYWHEEL_TICKS_PER_REV);

        flywheel.configureMotorPid(
                Settings.Launcher.PIDF_R,
                Settings.Launcher.PIDF_L);

        if (Settings.Launcher.RESET_YAW_ENCODER_ON_INIT) {
            yawEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        yawEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        yaw.setYawReader(new YawThroughBoreEncoder(
                yawEncoder,
                Settings.Launcher.YAW_ZERO_TICKS,
                Settings.Launcher.YAW_TICKS_PER_REV,
                Settings.Launcher.YAW_TICK_DIRECTION
        ));
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

    private static double nearestEquivalentAngle(double wrappedAngle, double continuousReference) {
        double wrappedReference = normalizeRadians(continuousReference);
        double delta = normalizeRadians(wrappedAngle - wrappedReference);
        return continuousReference + delta;
    }

    private double computeHoodAngleRadians(double dx, double dy) {
        double magnitude = Math.hypot(dx, dy);
        if (magnitude < EPSILON) {
            return Settings.Launcher.HOOD_MAX_ANGLE_RAD;
        }
        double verticalDelta = Settings.Launcher.GOAL_HEIGHT_INCHES - Settings.Launcher.LAUNCHER_HEIGHT_INCHES;
        return Math.atan2(verticalDelta, magnitude) + Settings.Launcher.HOOD_ANGLE_OFFSET_RAD;
    }

    private double applyYawMotionCompensation(
            double dx,
            double dy,
            double hoodAngleRadians,
            double baseFlywheelRpm,
            Vector botVelocityRobotCentric,
            Pose botPose
    ) {
        double magnitude = Math.hypot(dx, dy);
        if (magnitude < EPSILON) {
            return 0.0;
        }
        double directionX = dx / magnitude;
        double directionY = dy / magnitude;

        if (!Settings.Launcher.USE_MOTION_COMPENSATION || botVelocityRobotCentric == null || botPose == null) {
            return Math.atan2(directionY, directionX);
        }

        double speedScale = Settings.Launcher.FLYWHEEL_RPM_TO_EXIT_SPEED;
        if (speedScale <= EPSILON) {
            return Math.atan2(directionY, directionX);
        }
        double baseHorizontalSpeed = baseFlywheelRpm * speedScale * Math.cos(hoodAngleRadians);
        if (baseHorizontalSpeed <= EPSILON) {
            return Math.atan2(directionY, directionX);
        }

        // getVelocity() is robot-centric, so rotate into field space.
        double heading = botPose.getHeading();
        double localVx = botVelocityRobotCentric.getXComponent();
        double localVy = botVelocityRobotCentric.getYComponent();
        double robotVx = (localVx * Math.cos(heading)) - (localVy * Math.sin(heading));
        double robotVy = (localVx * Math.sin(heading)) + (localVy * Math.cos(heading));

        // v_projectile_field = v_projectile_relative + v_robot_field.
        double requiredRelativeVx = (directionX * baseHorizontalSpeed) - robotVx;
        double requiredRelativeVy = (directionY * baseHorizontalSpeed) - robotVy;
        if (Math.hypot(requiredRelativeVx, requiredRelativeVy) < EPSILON) {
            return Math.atan2(directionY, directionX);
        }
        return Math.atan2(requiredRelativeVy, requiredRelativeVx);
    }

    public void update(Pose botPose, Vector botVelocity) {
        if (botPose == null || targetPose == null) {
            return;
        }

        double dx = targetPose.getX() - botPose.getX();
        double dy = targetPose.getY() - botPose.getY();
        double distanceToTarget = Math.hypot(dx, dy);
        double hoodAngleRadians = computeHoodAngleRadians(dx, dy);
        Settings.Launcher.RPMSolution rpmSolution = rpmSolutionModel.solve(distanceToTarget);
        double compensatedFieldYaw = applyYawMotionCompensation(
                dx,
                dy,
                hoodAngleRadians,
                rpmSolution.flywheelRpm,
                botVelocity,
                botPose
        );
        double desiredRobotRelativeYawWrapped = normalizeRadians(compensatedFieldYaw - botPose.getHeading());
        double robotRelativeYawTarget = desiredRobotRelativeYawWrapped;
        if (yaw.hasReader()) {
            robotRelativeYawTarget = nearestEquivalentAngle(
                    desiredRobotRelativeYawWrapped,
                    yaw.getCurrentAngleRadians()
            );
        }

        yaw.setTargetAngleRadians(robotRelativeYawTarget);
        yaw.update();

        setHoodAngleRadians(hoodAngleRadians);
        flywheel.setTargetFlywheelRPM(rpmSolution.flywheelRpm);
        flywheel.update();
    }

    private void setHoodAngleRadians(double hoodAngleRadians) {
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

    public void stop() {
        yaw.stop();
        flywheel.stop();
    }

    public static class YawUnion {
        private final CRServo yawRight;
        private final CRServo yawLeft;
        private final ElapsedTime loopTimer = new ElapsedTime();
        private final PIDFCoefficients rightPidf;
        private final PIDFCoefficients leftPidf;
        private final double maxPower;

        private YawThroughBoreEncoder yawReader;
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

        public void setYawReader(YawThroughBoreEncoder yawReader) {
            this.yawReader = yawReader;
        }

        public boolean hasReader() {
            return yawReader != null;
        }

        public double getCurrentAngleRadians() {
            if (yawReader == null) {
                return targetAngleRadians;
            }
            return yawReader.getYawAngleRadians();
        }

        public void setTargetAngleRadians(double targetAngleRadians) {
            this.targetAngleRadians = targetAngleRadians;
        }

        public void update() {
            if (yawReader == null) {
                yawRight.setPower(0.0);
                yawLeft.setPower(0.0);
                return;
            }

            double dt = Math.max(loopTimer.seconds(), 1e-3);
            loopTimer.reset();

            double currentAngle = yawReader.getYawAngleRadians();
            double error = targetAngleRadians - currentAngle;

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
        private final double flywheelToMotorRatio;
        private final double flywheelTicksPerRev;
        private double targetFlywheelRPM = 0.0;

        public FlywheelUnion(
                DcMotorEx rightMotor,
                DcMotorEx leftMotor,
                double flywheelToMotorRatio, double flywheelTicksPerRev) {
            this.rightMotor = rightMotor;
            this.leftMotor = leftMotor;
            this.flywheelToMotorRatio = flywheelToMotorRatio;
            this.flywheelTicksPerRev = flywheelTicksPerRev;
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

        public void setTargetFlywheelRPM(double rpm) {
            this.targetFlywheelRPM = rpm;
        }

        public void update() {
            // Flywheel RPM > Motor RPM > Motor TPM > Motor TPS
            double targetMotorVelocity = (targetFlywheelRPM * flywheelToMotorRatio * flywheelTicksPerRev) / 60;
            // velocity in ticks per second
            rightMotor.setVelocity(targetMotorVelocity);
            leftMotor.setVelocity(targetMotorVelocity);
        }

        public void stop() {
            rightMotor.setVelocity(0.0);
            leftMotor.setVelocity(0.0);
        }
    }
}
