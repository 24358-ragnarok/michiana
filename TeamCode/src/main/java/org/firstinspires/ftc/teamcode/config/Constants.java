package org.firstinspires.ftc.teamcode.config;

import com.bylazar.field.Style;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.PinpointLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pathing.tank.TankDrivetrain;

public class Constants {
    public static class Flags {
        public static final boolean DEBUG = true;
    }
    /**
     * PedroPathing configuration.
     * Holds the constants for both Tank and Mecanum followers, using a common() set of parameters.
     **/
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

            public static final FollowerConstants MECANUM =
                    common()
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

            public static final FollowerConstants TANK =
                    common()
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

            public static final MecanumConstants MECANUM =
                    common()
                            .xVelocity(86)
                            .yVelocity(60);


            /* ---- TANK ---- */

            public static final MecanumConstants TANK =
                    common()
                            .xVelocity(86)
                            .yVelocity(60);
        }

        public static class Localizer {
            public static final PinpointConstants PINPOINT =
                    new PinpointConstants()
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
                    1
            );
            public static PathConstraints TANK = new PathConstraints(
                    0.995,
                    0.01,
                    0.01,
                    0.001,
                    80,
                    1.00,
                    10,
                    1
            );
        }

        public static class Builder {
            public static com.pedropathing.follower.Follower mecanum(HardwareMap hardwareMap) {
                return new FollowerBuilder(Follower.MECANUM, hardwareMap)
                        .mecanumDrivetrain(Drive.MECANUM)
                        .pathConstraints(Path.MECANUM)
                        .pinpointLocalizer(Localizer.PINPOINT)
                        .build();
            }

            public static com.pedropathing.follower.Follower tank(HardwareMap hardwareMap) {
                PinpointLocalizer l = new PinpointLocalizer(hardwareMap, Localizer.PINPOINT);
                TankDrivetrain t = new TankDrivetrain(hardwareMap, Drive.TANK);
                return new com.pedropathing.follower.Follower(
                        Follower.TANK, l, t, Path.TANK);
            }
        }
    }

    /**
     * Simple robot measurements
     **/
    public static class Dimensions {
        public static float WIDTH = 16;
        public static float LENGTH = 16;

    }

    /**
     * HardwareMap Names
     **/
    public static class Hardware {
        public static final String PINPOINT = "pinpoint";
        public static final String LEFT_FRONT_MOTOR = "leftFront";
        public static final String LEFT_REAR_MOTOR = "leftRear";
        public static final String RIGHT_FRONT_MOTOR = "rightFront";
        public static final String RIGHT_REAR_MOTOR = "rightRear";

    }

    public static class Logging {
        public static final int INTERVAL = Flags.DEBUG ? 50 : 1000;
        public static final Style followerLook = new Style(
                "", "#FFD40C", 0.75
        );
        public static final Style robotLook = new Style(
                "", "#4CAF50", 0.75
        );
    }
}