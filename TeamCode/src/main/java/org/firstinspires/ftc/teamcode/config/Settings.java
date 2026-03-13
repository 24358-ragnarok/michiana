package org.firstinspires.ftc.teamcode.config;

import android.content.Context;

import com.bylazar.field.Style;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.ftccommon.FtcEventLoop;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLogSettings;

/**
 * Global configuration settings for the robot.
 * <p>
 * This class contains constants for hardware mapping, PID coefficients,
 * pathing constraints, field dimensions, and other tunable parameters.
 */
public class Settings {
    /**
     * Configures the PsiKit AutoLog settings when the event loop is created.
     */
    @OnCreateEventLoop
    public static void configure(Context context, FtcEventLoop ftcEventLoop) {
        System.setProperty(PsiKitAutoLogSettings.PROPERTY_RLOG_PORT, Flags.DEBUG ? "5900" : "0");
        PsiKitAutoLogSettings.enabledByDefault = Flags.DEBUG;
    }

    /**
     * General flags for controlling robot behavior.
     */
    public static class Flags {
        public static final boolean DEBUG = true;

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
                    .forwardZeroPowerAcceleration(-37.5)
                    .lateralZeroPowerAcceleration(-65.7)
                    .translationalPIDFCoefficients(
                            new PIDFCoefficients(0.13, 0.001, 0.02, 0.02))
                    .secondaryTranslationalPIDFCoefficients(
                            new PIDFCoefficients(0.1, 0.0001, 0.02, 0.02))
                    .headingPIDFCoefficients(
                            new PIDFCoefficients(0.7, 0.001, 0.05, 0.03))
                    .secondaryHeadingPIDFCoefficients(
                            new PIDFCoefficients(1.65, 0.001, 0.015, 0.02))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.5, 0.0, 0.01, 0.6, 0.0))
                    .secondaryDrivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.08, 0.001, 0.001, 0.6, 0.0));

            public static final FollowerConstants TANK = common()
                    .forwardZeroPowerAcceleration(-37.5)
                    .lateralZeroPowerAcceleration(-65.7)
                    .translationalPIDFCoefficients(
                            new PIDFCoefficients(0.13, 0.001, 0.02, 0.02))
                    .secondaryTranslationalPIDFCoefficients(
                            new PIDFCoefficients(0.1, 0.0001, 0.02, 0.02))
                    .headingPIDFCoefficients(
                            new PIDFCoefficients(0.7, 0.001, 0.05, 0.03))
                    .secondaryHeadingPIDFCoefficients(
                            new PIDFCoefficients(1.65, 0.001, 0.015, 0.02))
                    .drivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.5, 0.0, 0.01, 0.6, 0.0))
                    .secondaryDrivePIDFCoefficients(
                            new FilteredPIDFCoefficients(0.08, 0.001, 0.001, 0.6, 0.0));
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
                    .xVelocity(86)
                    .yVelocity(60);

            /* ---- TANK ---- */
            public static final MecanumConstants TANK = common()
                    .xVelocity(86)
                    .yVelocity(60);
        }

        public static class Localizer {
            public static final PinpointConstants PINPOINT = new PinpointConstants()
                    .forwardPodY(-4.75)
                    .strafePodX(-4.05)
                    .distanceUnit(DistanceUnit.INCH)
                    .hardwareMapName(Hardware.PINPOINT)
                    .encoderResolution(
                            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
                    .forwardEncoderDirection(
                            GoBildaPinpointDriver.EncoderDirection.REVERSED)
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
                    1.00,
                    10,
                    1);
            public static PathConstraints TANK = new PathConstraints(
                    0.995,
                    0.01,
                    0.01,
                    0.001,
                    80,
                    1.00,
                    10,
                    1);
        }
    }

    public static class Drivetrain {
        // Butterfly Drivetrain Servo Positions
        public static double RIGHT_MECANUM_DOWN_POSITION = 0.2;
        public static double RIGHT_TANK_DOWN_POSITION = 0.6;
        public static double LEFT_MECANUM_DOWN_POSITION = 0.6;
        public static double LEFT_TANK_DOWN_POSITION = 0.2;
    }

    public static class Launcher {
        // Hood calibration: map 10-50 degrees into servo range 0.2-0.8.
        public static double HOOD_MIN_ANGLE_RAD = Math.toRadians(10.0);
        public static double HOOD_MAX_ANGLE_RAD = Math.toRadians(50.0);
        public static double HOOD_MIN_SERVO_POSITION = 0.2;
        public static double HOOD_MAX_SERVO_POSITION = 0.8;
        // Flywheel target conversion:
        // flywheel velocity * motor-to-flywheel ratio = motor velocity target.
        public static double GEAR_RATIO_MOTOR_TO_FLYWHEEL = 1.0;
        // Per-servo PIDF used by software loop for the shared yaw axle.
        public static com.qualcomm.robotcore.hardware.PIDFCoefficients YAW_PIDF_R =
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        1.8, 0.0, 0.06, 0.0);
        public static com.qualcomm.robotcore.hardware.PIDFCoefficients YAW_PIDF_L =
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        1.8, 0.0, 0.06, 0.0);
        public static double YAW_MAX_POWER = 1.0;
        public static com.qualcomm.robotcore.hardware.PIDFCoefficients PIDF_R =
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        35, 0.02, 0, 0);
        public static com.qualcomm.robotcore.hardware.PIDFCoefficients PIDF_L =
                new com.qualcomm.robotcore.hardware.PIDFCoefficients(
                        35, 0.02, 0, 0);
        // Replace with your distance -> trajectory model when ready.
        public static ShotModel SHOT_MODEL = distanceInches ->
                new ShotSolution(HOOD_MIN_ANGLE_RAD, 0.0);

        public interface ShotModel {
            ShotSolution solve(double distanceInches);
        }

        public static class ShotSolution {
            public final double hoodAngleRadians;
            public final double flywheelVelocity;

            public ShotSolution(double hoodAngleRadians, double flywheelVelocity) {
                this.hoodAngleRadians = hoodAngleRadians;
                this.flywheelVelocity = flywheelVelocity;
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
        public static final String BUTTERFLY_LEFT = "butterflyLeft";
        public static final String BUTTERFLY_RIGHT = "butterflyRight";

        public static final String HOOD = "hood";
        public static final String FLYWHEEL_R = "flywheelRight";
        public static final String FLYWHEEL_L = "flywheelLeft";
        public static final String YAW_R = "yawRight";
        public static final String YAW_L = "yawLeft";

    }

    public static class Autonomous {
        public static final double DURATION = 30;
    }

    public static class Positions {
        public static class BotPoses {
            public static Pose START_FAR = new Pose();
            public static Pose START_CLOSE = new Pose();
        }

        public static class TeleopPresets {
            public static final Pose CLOSE_SHOOT = new Pose(54.92, 86.55, Math.toRadians(130.6));
            public static final Pose FAR_SHOOT = new Pose(60, 18, Math.toRadians(112.75));
            public static final Pose HUMAN_PLAYER = new Pose(30, 30, Math.toRadians(225));
            public static final Pose GATE = new Pose(12.44, 62, Math.toRadians(150));
            public static final Pose PARK = new Pose(106, 32, Math.toRadians(180));
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
