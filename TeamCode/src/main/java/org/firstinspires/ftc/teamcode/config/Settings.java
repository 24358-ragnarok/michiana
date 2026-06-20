package org.firstinspires.ftc.teamcode.config;

import static org.firstinspires.ftc.teamcode.config.Settings.Dimensions.LENGTH;
import static org.firstinspires.ftc.teamcode.config.Settings.Dimensions.WIDTH;

import com.bylazar.field.Style;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.shooter.PolynomialShooterModels;

/**
 * Global configuration settings for the robot.
 * <p>
 * This class contains constants for hardware mapping, PID coefficients,
 * pathing constraints, field dimensions, and other tunable parameters.
 */
public class Settings {

    /**
     * General flags for controlling robot behavior.
     */
    public static class Flags {
        public static final boolean DEBUG = false;

        public static final boolean SFX = true;
    }

    /**
     * PedroPathing configuration.
     * Holds the constants for both Tank and Mecanum followers.
     */
    public static class PedroPathing {
        public static class Follower {
            private static FollowerConstants common() {
                return new FollowerConstants()
                        .mass(13.8)
                        .automaticHoldEnd(true)
                        .useSecondaryDrivePIDF(true)
                        .useSecondaryHeadingPIDF(true)
                        .useSecondaryTranslationalPIDF(true);
            }

            public static final FollowerConstants MECANUM = common()
                    .forwardZeroPowerAcceleration(-73.7)
                    .lateralZeroPowerAcceleration(-91.8)
                    .translationalPIDFCoefficients(
                            new PIDFCoefficients(0.17, 0.001, 0.02, 0.025))
                    .secondaryTranslationalPIDFCoefficients(
                            new PIDFCoefficients(0.1, 0.0001, 0.02, 0.02))
                    .headingPIDFCoefficients(
                            new PIDFCoefficients(0.8, 0.001, 0.05, 0.032))
                    .secondaryHeadingPIDFCoefficients(
                            new PIDFCoefficients(1.8, 0.01, 0.015, 0.025))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.5, 0.0, 0.01, 0.6, 0.0))
                    .secondaryDrivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.1, 0.001, 0.001, 0.6, 0.01));
        }

        public static class Drive {
            /* ---- MECANUM ---- */
            private static MecanumConstants common() {
                return new MecanumConstants()
                        .maxPower(1)
                        .leftFrontMotorName(Hardware.LEFT_FRONT_MOTOR)
                        .leftRearMotorName(Hardware.LEFT_REAR_MOTOR)
                        .rightFrontMotorName(Hardware.RIGHT_FRONT_MOTOR)
                        .rightRearMotorName(Hardware.RIGHT_REAR_MOTOR)
                        .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                        .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                        .useBrakeModeInTeleOp(false);
            }

            public static final MecanumConstants MECANUM = common()
                    .xVelocity(73)
                    .yVelocity(55);
        }

        public static class Localizer {
            public static final PinpointConstants PINPOINT = new PinpointConstants()
                    .forwardPodY(-3.125)
                    .strafePodX(-3.5)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName(Hardware.PINPOINT)
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD)
                    .strafeEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.FORWARD);
        }

        public static class Path {
            public static PathConstraints MECANUM = new PathConstraints(
                    0.995,
                    0.01,
                    0.01,
                    0.001,
                    80,
                    1.2,
                    10,
                    0.8);
        }
    }

    public static class Turret {
        public static final double REV_THROUGH_BORE_V2_COUNTS_PER_REV = 8192.0;

        // Hood calibration: map 10-50 degrees into servo range 0.2-0.8.
        public static double HOOD_MIN_ANGLE_RAD = Math.toRadians(50.0);
        public static double HOOD_MAX_ANGLE_RAD = Math.toRadians(70.0);
        public static double HOOD_MIN_SERVO_POSITION = 0.0;
        public static double HOOD_MAX_SERVO_POSITION = 1.0;
        public static double GEAR_RATIO_MOTOR_TO_FLYWHEEL = (double) 2 / 3;
        public static double FLYWHEEL_TICKS_PER_REV = 28.0;

        // Geometry for hood "point at goal" behavior.
        public static double LAUNCHER_HEIGHT_INCHES = 9.0;
        public static double GOAL_HEIGHT_INCHES = 36.0;
        public static double HOOD_ANGLE_OFFSET_RAD = 0.0;
        public static double YAW_MIN_TICKS = 407;
        public static double YAW_MAX_TICKS = -386;
        public static double YAW_MIN_ANGLE_DEG = -90;
        public static double YAW_MAX_ANGLE_DEG = 90;

        public static double TUNG_OPEN_POSITION = 0.527;
        public static double TUNG_CLOSED_POSITION = 0.59;

        // Per-servo PIDF used by software loop for the shared yaw axle.
        public static com.qualcomm.robotcore.hardware.PIDFCoefficients YAW_PIDF = new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                1.8, 0.0, 0.06, 0.0);
        public static double YAW_MAX_POWER = 1.0;
        // Infinite-yaw tick conversion settings.
        // angleRad = ((ticks - YAW_ZERO_TICKS) / YAW_TICKS_PER_REV) * 2pi *
        // YAW_TICK_DIRECTION

        // Shoot-on-the-move compensation.
        // Converts flywheel RPM from model into projectile linear speed (inches/sec).
        public static boolean USE_MOTION_COMPENSATION = false;
        public static double FLYWHEEL_RPM_TO_EXIT_SPEED = 0.05;

        /**
         * Polynomial degree used by
         * {@link org.firstinspires.ftc.teamcode.opmodes.SolutionTuner}
         * and the default coefficient arrays below.
         * <p>
         * Model form: c0 + c1*d + c2*d^2 + ... where d is horizontal distance in
         * inches.
         */
        public static int SOLUTION_POLYNOMIAL_DEGREE = 2;

        /**
         * Hood angle model coefficients (radians). Paste updated values from
         * SolutionTuner.
         */
        public static double[] ANGLE_COEFFICIENTS = new double[]{-0.301161, 0.036121, -0.000213};

        /**
         * Flywheel RPM model coefficients. Paste updated values from SolutionTuner.
         */
        public static double[] RPM_COEFFICIENTS = new double[]{4527.650994, -44.645306, 0.191299};

        public static AngleSolutionModel ANGLE_SOLUTION_MODEL = PolynomialShooterModels
                .angleWithFarMax(ANGLE_COEFFICIENTS);

        public static RPMSolutionModel RPM_SOLUTION_MODEL = PolynomialShooterModels
                .rpmWithFarMax(RPM_COEFFICIENTS);

        public interface AngleSolutionModel {
            AngleSolution solve(double distanceInches);
        }

        public interface RPMSolutionModel {
            RPMSolution solve(double distanceInches);
        }

        public static class AngleSolution {
            public final double angleRadians;

            public AngleSolution(double angleRadians) {
                this.angleRadians = angleRadians;
            }
        }

        public static class RPMSolution {
            public final double flywheelRpm;

            public RPMSolution(double flywheelRpm) {
                this.flywheelRpm = flywheelRpm;
            }
        }
    }

    /**
     * Simple robot physical measurements.
     */
    public static class Dimensions {
        public static float WIDTH = 16;
        public static float LENGTH = 16;
    }

    /**
     * HardwareMap device names.
     */
    public static class Hardware {
        public static final String PINPOINT = "pinpoint";
        public static final String LEFT_FRONT_MOTOR = "leftFront";
        public static final String LEFT_REAR_MOTOR = "leftRear";
        public static final String RIGHT_FRONT_MOTOR = "rightFront";
        public static final String RIGHT_REAR_MOTOR = "rightRear";

        public static final String HOOD = "hood";
        public static final String FLYWHEEL_R = "flywheelRight";
        public static final String FLYWHEEL_L = "flywheelLeft";
        public static final String YAW = "yaw";
        public static final String TUNG = "tung";
        public static final String INTAKE_MOTOR = "intake";

    }

    public static class Autonomous {
        public static final double DURATION = 30;
        public static final long LAUNCH_CALIBRATION_MS = 250;
        public static final long LAUNCH_DURATION_MS = 2500;
    }

    public static class Positions {
        public static class Towers {
            public static final Pose RED_GOAL = new Pose(140, 140, Math.toRadians(225));
            public static final Pose BLUE_GOAL = new Pose(4, 140, Math.toRadians(315));
            public static final Pose OBELISK = new Pose(72.0, 150.0, Math.toRadians(0));
            public static final Pose CLASSIFIER_EXIT = new Pose(9.4, 50, Math.toRadians(97));
            public static final Pose CLOSE_SCAN = new Pose(60, 100.0, Math.toRadians(80));
            public static final Pose FAR_SCAN = new Pose(60, 12, Math.toRadians(80));
        }

        public static class TeleOp {
            public static final Pose CLOSE_SHOOT = new Pose(54.92, 95, Math.toRadians(130.6));
            public static final Pose CLOSE_SHOOT_AUTO = new Pose(58, 90, Math.toRadians(130.0));
            public static final Pose FAR_SHOOT = new Pose(60, 18, Math.toRadians(112.75));
            public static final Pose FAR_SHOOT_AUTO = new Pose(55, 18, Math.toRadians(111));
            public static final Pose HUMAN_PLAYER = new Pose(30, 30, Math.toRadians(225));
            public static final Pose GATE = new Pose(12.44, 62, Math.toRadians(150));
            public static final Pose PARK = new Pose(106, 32, Math.toRadians(180));
            public static final Pose RESET = new Pose(144 - WIDTH / 2, 0 + LENGTH / 2, Math.toRadians(90));
        }

        public static class BotPoses {
            public static final Pose START_FAR = new Pose(56.26, 9.0, Math.toRadians(90));
            public static final Pose START_CLOSE = new Pose(25.8, 132.4, Math.toRadians(55));
        }

        public static class Samples {
            public static class GateAndEating {
                public static final Pose EAT_FROM_EMPTY_DIRECTLY = new Pose(32, 16);
                public static final Pose EMPTY_GATE = new Pose(12, 60.6, Math.toRadians(147));
                public static final Pose EMPTY_GATE_MOVE_BACK = new Pose(12, 55.6, Math.toRadians(147));
            }

            public static class Preset1 {
                public static final Pose PREP = new Pose(44, 35.67, Math.toRadians(180));
                public static final Pose GRAB_1 = new Pose(36.0, 35.67, Math.toRadians(180));
                public static final Pose END = new Pose(20, 35.67, Math.toRadians(180));
            }

            public static class Preset2 {
                public static final Pose PREP = new Pose(44, 59.5, Math.toRadians(180));
                public static final Pose END = new Pose(25, 58, Math.toRadians(180));
                public static final Pose END_AND_EMPTY_GATE = new Pose(20, 63, Math.toRadians(180));
            }

            public static class Preset3 {
                public static final Pose PREP = new Pose(44, 84.6, Math.toRadians(180));
                public static final Pose END = new Pose(25, 84.6, Math.toRadians(180));
            }

            public static class HumanPlayerPreset {
                public static final Pose END = new Pose(7.0, 21, Math.toRadians(180));
                public static final Pose HUMAN_PLAYER_GRAB_1 = new Pose(5, 8.5, Math.toRadians(180));
                public static final Pose END_First_Secure_preload = new Pose(18, 10, Math.toRadians(180));
                public static final Pose END_First_Secure_preload1 = new Pose(10, 10, Math.toRadians(180));
            }
        }

        public static class ControlPoints {
            public static final Pose TURN_BOT = new Pose(53.4, 16.9, Math.toRadians(180));
            public static final Pose FROM_FAR_SHOOT_TO_HP = new Pose(70.4, 8);
            public static final Pose PRESET_1_APPROACH_FAR = new Pose(75, 38);
            public static final Pose PRESET_1_APPROACH_CLOSE = new Pose(67, 23);
            public static final Pose PRESET_1_END_TO_FAR_SHOOT = new Pose(50, 30);
            public static final Pose FROM_PRESET2_TO_CLOSE = new Pose(64, 56);
            public static final Pose PRESET_2_APPROACH_FAR = new Pose(65, 59);
            public static final Pose FROM_PRESET3_TO_CLOSE = new Pose(41, 81);
            public static final Pose FROM_PRESET3_TO_FAR = new Pose(52, 37);
            public static final Pose FROM_CLOSE_SHOOT_TO_PRESET2_END = new Pose(65.4, 58);
            public static final Pose FROM_CLOSE_SHOOT_TO_PRESET3_END = new Pose(81, 81);
            public static final Pose EMPTY_GATE_APPROACH = new Pose(42.5, 64.3);
        }

        public static class Park {
            public static final Pose FAR = new Pose(53, 21.5, Math.toRadians(135));
            public static final Pose CLOSE = new Pose(58.1, 101.7, Math.toRadians(145));
            public static final Pose CLOSE_SAFE_PARK_POSE = new Pose(48, 130, Math.toRadians(90));
        }
    }

    /**
     * Logging configuration.
     */
    public static class Logging {
        public static final int INTERVAL = Flags.DEBUG ? 50 : 1000;
        public static final boolean DRAW_FIELD = Flags.DEBUG;
        public static final Style followerLook = new Style(
                "", "#FFD40C", 0.75);
        public static final Style robotLook = new Style(
                "", "#4CAF50", 0.75);
    }
}
