package org.firstinspires.ftc.teamcode.sys.hardware;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Settings;

public class Turret {
    private static Pose targetPose = Settings.Positions.TeleopPresets.FAR_SHOOT;
    private static final double EPSILON = 1e-6;

    private final ServoImplEx hood;
    private final DcMotorEx yaw;

    private final DcMotorExUnion flywheel;
    private final Settings.Turret.RPMSolutionModel rpmSolutionModel = Settings.Turret.RPM_SOLUTION_MODEL;

    public Turret(HardwareMap hardwareMap) {
        hood = hardwareMap.get(ServoImplEx.class, Settings.Hardware.HOOD);
        yaw = hardwareMap.get(DcMotorEx.class, Settings.Hardware.YAW);

        DcMotorEx flywheelR = hardwareMap.get(DcMotorEx.class, Settings.Hardware.FLYWHEEL_R);
        DcMotorEx flywheelL = hardwareMap.get(DcMotorEx.class, Settings.Hardware.FLYWHEEL_L);


        flywheel = new DcMotorExUnion(
                flywheelR,
                flywheelL,
                Settings.Turret.GEAR_RATIO_MOTOR_TO_FLYWHEEL,
                Settings.Turret.FLYWHEEL_TICKS_PER_REV);

        flywheel.configureMotorPid(
                Settings.Turret.PIDF_R,
                Settings.Turret.PIDF_L);

        if (Settings.Turret.RESET_YAW_ENCODER_ON_INIT) {
            yaw.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    public static void setTargetPose(Pose targetPose) {
        org.firstinspires.ftc.teamcode.sys.hardware.Turret.targetPose = targetPose;
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
            return Settings.Turret.HOOD_MAX_ANGLE_RAD;
        }
        double verticalDelta = Settings.Turret.GOAL_HEIGHT_INCHES - Settings.Turret.LAUNCHER_HEIGHT_INCHES;
        return Math.atan2(verticalDelta, magnitude) + Settings.Turret.HOOD_ANGLE_OFFSET_RAD;
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

        if (!Settings.Turret.USE_MOTION_COMPENSATION || botVelocityRobotCentric == null || botPose == null) {
            return Math.atan2(directionY, directionX);
        }

        double speedScale = Settings.Turret.FLYWHEEL_RPM_TO_EXIT_SPEED;
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
        Settings.Turret.RPMSolution rpmSolution = rpmSolutionModel.solve(distanceToTarget);
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
                Settings.Turret.HOOD_MIN_ANGLE_RAD,
                Settings.Turret.HOOD_MAX_ANGLE_RAD);
        double normalized = (clamped - Settings.Turret.HOOD_MIN_ANGLE_RAD)
                / (Settings.Turret.HOOD_MAX_ANGLE_RAD - Settings.Turret.HOOD_MIN_ANGLE_RAD);
        double servoPosition = Settings.Turret.HOOD_MIN_SERVO_POSITION
                + (normalized
                * (Settings.Turret.HOOD_MAX_SERVO_POSITION - Settings.Turret.HOOD_MIN_SERVO_POSITION));
        hood.setPosition(Range.clip(servoPosition, 0.0, 1.0));
    }

    public void stop() {
        yaw.stop();
        flywheel.stop();
    }

    public static class DcMotorExUnion {
        private final DcMotorEx rightMotor;
        private final DcMotorEx leftMotor;
        private final double flywheelToMotorRatio;
        private final double flywheelTicksPerRev;
        private double targetFlywheelRPM = 0.0;

        public DcMotorExUnion(
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
