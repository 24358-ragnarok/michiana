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

    private final Hood hood;
    private final Yaw yaw;
    private final Flywheel flywheel;
    private final Tung tung;

    public Turret(HardwareMap hardwareMap) {
        this.hood = new Hood(hardwareMap);
        this.yaw = new Yaw(hardwareMap);
        this.flywheel = new Flywheel(hardwareMap);
        this.tung = new Tung(hardwareMap);
    }

    public static void setTargetPose(Pose targetPose) {
        Turret.targetPose = targetPose;
    }

    public void start() {
        yaw.start();
        tung.start();
    }

    public void update(Pose botPose, Vector botVelocity) {
        if (botPose == null || targetPose == null) {
            return;
        }

        double dx = targetPose.getX() - botPose.getX();
        double dy = targetPose.getY() - botPose.getY();
        double distance = Math.hypot(dx, dy);

        hood.update(distance);
        flywheel.update(distance);
        updateYaw(botPose, botVelocity);
    }

    public void updateYaw(Pose botPose, Vector botVelocity) {
        if (botPose == null || targetPose == null) {
            return;
        }
        yaw.update(botPose, botVelocity, targetPose, hood.getAngleRadians(), flywheel.getTargetRPM());
    }

    public double getTargetDistance(Pose botPose) {
        if (botPose == null || targetPose == null) {
            return 0.0;
        }
        double dx = targetPose.getX() - botPose.getX();
        double dy = targetPose.getY() - botPose.getY();
        return Math.hypot(dx, dy);
    }

    public void stop() {
        yaw.stop();
        flywheel.stop();
    }

    public Hood getHood() {
        return hood;
    }

    public Yaw getYaw() {
        return yaw;
    }

    public Flywheel getFlywheel() {
        return flywheel;
    }

    public Tung getTung() {
        return tung;
    }

    public class Hood {
        private final ServoImplEx servo;
        private double currentAngleRad;

        public Hood(HardwareMap hardwareMap) {
            this.servo = hardwareMap.get(ServoImplEx.class, Settings.Hardware.HOOD);
        }

        public void update(double distance) {
            Settings.Turret.AngleSolution solution = Settings.Turret.ANGLE_SOLUTION_MODEL.solve(distance);
            setAngleRadians(solution.angleRadians);
        }

        public void setAngleRadians(double angleRad) {
            this.currentAngleRad = Range.clip(angleRad, Settings.Turret.HOOD_MIN_ANGLE_RAD, Settings.Turret.HOOD_MAX_ANGLE_RAD);
            double normalized = (currentAngleRad - Settings.Turret.HOOD_MIN_ANGLE_RAD)
                    / (Settings.Turret.HOOD_MAX_ANGLE_RAD - Settings.Turret.HOOD_MIN_ANGLE_RAD);
            double position = Settings.Turret.HOOD_MIN_SERVO_POSITION
                    + normalized * (Settings.Turret.HOOD_MAX_SERVO_POSITION - Settings.Turret.HOOD_MIN_SERVO_POSITION);
            servo.setPosition(Range.clip(position, 0.0, 1.0));
        }

        public double getAngleRadians() {
            return currentAngleRad;
        }
    }

    public class Yaw {
        private final DcMotorEx motor;

        public Yaw(HardwareMap hardwareMap) {
            this.motor = hardwareMap.get(DcMotorEx.class, Settings.Hardware.YAW);
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setTargetPosition(0);
            this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, Settings.Turret.YAW_PIDF);
        }

        public void start() {
            this.motor.setPower(Settings.Turret.YAW_MAX_POWER);
        }

        public void update(Pose botPose, Vector botVelocity, Pose targetPose, double hoodAngle, double flywheelRPM) {
            double dx = targetPose.getX() - botPose.getX();
            double dy = targetPose.getY() - botPose.getY();

            double fieldYaw = applyMotionCompensation(dx, dy, hoodAngle, flywheelRPM, botVelocity, botPose);
            double robotRelativeYaw = normalizeRadians(fieldYaw - botPose.getHeading());

            double targetTicks = angleToTicks(Math.toDegrees(robotRelativeYaw));
            motor.setTargetPosition((int) Range.clip(targetTicks, Settings.Turret.YAW_MIN_TICKS, Settings.Turret.YAW_MAX_TICKS));
        }

        private double applyMotionCompensation(double dx, double dy, double hoodAngle, double flywheelRPM, Vector botVelocity, Pose botPose) {
            if (!Settings.Turret.USE_MOTION_COMPENSATION || botVelocity == null) {
                return Math.atan2(dy, dx);
            }

            double speedScale = Settings.Turret.FLYWHEEL_RPM_TO_EXIT_SPEED;
            double baseHorizontalSpeed = flywheelRPM * speedScale * Math.cos(hoodAngle);
            if (baseHorizontalSpeed <= EPSILON) {
                return Math.atan2(dy, dx);
            }

            double heading = botPose.getHeading();
            double robotVx = (botVelocity.getXComponent() * Math.cos(heading)) - (botVelocity.getYComponent() * Math.sin(heading));
            double robotVy = (botVelocity.getXComponent() * Math.sin(heading)) + (botVelocity.getYComponent() * Math.cos(heading));

            double magnitude = Math.hypot(dx, dy);
            double dirX = dx / magnitude;
            double dirY = dy / magnitude;

            double reqRelVx = (dirX * baseHorizontalSpeed) - robotVx;
            double reqRelVy = (dirY * baseHorizontalSpeed) - robotVy;

            return Math.atan2(reqRelVy, reqRelVx);
        }

        private double angleToTicks(double angleDeg) {
            double normalized = (angleDeg - Settings.Turret.YAW_MIN_ANGLE_DEG)
                    / (Settings.Turret.YAW_MAX_ANGLE_DEG - Settings.Turret.YAW_MIN_ANGLE_DEG);
            return Settings.Turret.YAW_MIN_TICKS + normalized * (Settings.Turret.YAW_MAX_TICKS - Settings.Turret.YAW_MIN_TICKS);
        }

        private double normalizeRadians(double angle) {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        public void stop() {
            motor.setTargetPosition(0);
            motor.setPower(Settings.Turret.YAW_MAX_POWER);
        }
    }

    public class Flywheel {
        private final DcMotorEx rightMotor;
        private final DcMotorEx leftMotor;
        private double targetRPM;
        public boolean active = true;

        public Flywheel(HardwareMap hardwareMap) {
            this.rightMotor = hardwareMap.get(DcMotorEx.class, Settings.Hardware.FLYWHEEL_R);
            this.leftMotor = hardwareMap.get(DcMotorEx.class, Settings.Hardware.FLYWHEEL_L);

            rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            rightMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Settings.Turret.PIDF_R);
            leftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, Settings.Turret.PIDF_L);
        }

        public void update(double distance) {
            if (!active) {return;}
            Settings.Turret.RPMSolution solution = Settings.Turret.RPM_SOLUTION_MODEL.solve(distance);
            setTargetRPM(solution.flywheelRpm);
        }

        public void setTargetRPM(double rpm) {
            this.targetRPM = rpm;
            double motorVelocity = (rpm * Settings.Turret.GEAR_RATIO_MOTOR_TO_FLYWHEEL * Settings.Turret.FLYWHEEL_TICKS_PER_REV) / 60.0;
            if (!active) {return;}
            rightMotor.setVelocity(motorVelocity);
            leftMotor.setVelocity(motorVelocity);
        }

        public double getTargetRPM() {
            return targetRPM;
        }

        public void stop() {
            rightMotor.setVelocity(0);
            leftMotor.setVelocity(0);
        }

        public void toggle() {
            active = !active;
            if (!active) {
                stop();
            }
        }
    }

    public class Tung {
        private final ServoImplEx servo;

        public Tung(HardwareMap hardwareMap) {
            this.servo = hardwareMap.get(ServoImplEx.class, Settings.Hardware.TUNG);
        }

        public void start() {
            close();
        }

        public void open() {
            servo.setPosition(Settings.Turret.TUNG_OPEN_POSITION);
        }

        public void close() {
            servo.setPosition(Settings.Turret.TUNG_CLOSED_POSITION);
        }
    }
}
