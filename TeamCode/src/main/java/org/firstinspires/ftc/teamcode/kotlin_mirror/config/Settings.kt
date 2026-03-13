package org.firstinspires.ftc.teamcode.kotlin_mirror.config

import android.content.Context
import com.bylazar.field.Style
import com.pedropathing.control.FilteredPIDFCoefficients
import com.pedropathing.control.PIDFCoefficients
import com.pedropathing.follower.FollowerConstants
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.ftc.localization.constants.PinpointConstants
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathConstraints
import com.qualcomm.ftccommon.FtcEventLoop
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.ftccommon.external.OnCreateEventLoop
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.psilynx.psikit.ftc.autolog.PsiKitAutoLogSettings

/**
 * Global configuration settings for the robot.
 *
 * This class contains constants for hardware mapping, PID coefficients,
 * pathing constraints, field dimensions, and other tunable parameters.
 */
object Settings {

    /**
     * Configures the PsiKit AutoLog settings when the event loop is created.
     */
    @JvmStatic
    @OnCreateEventLoop
    fun configure(context: Context, ftcEventLoop: FtcEventLoop) {
        System.setProperty(PsiKitAutoLogSettings.PROPERTY_RLOG_PORT, if (Flags.DEBUG) "5900" else "0")
        PsiKitAutoLogSettings.enabledByDefault = Flags.DEBUG
        PsiKitAutoLogSettings.enableLinearByDefault = Flags.DEBUG
    }

    /**
     * General flags for controlling robot behavior.
     */
    object Flags {
        const val DEBUG = true
        const val SFX = true
    }

    /**
     * PedroPathing configuration.
     * Holds the constants for both Tank and Mecanum followers.
     */
    object PedroPathing {
        object Follower {
            private fun common(): FollowerConstants {
                return FollowerConstants()
                    .mass(13.8)
                    .automaticHoldEnd(true)
                    .useSecondaryDrivePIDF(true)
                    .useSecondaryHeadingPIDF(true)
                    .useSecondaryTranslationalPIDF(true)
            }

            @JvmField
            val MECANUM: FollowerConstants = common()
                .forwardZeroPowerAcceleration(-37.5)
                .lateralZeroPowerAcceleration(-65.7)
                .translationalPIDFCoefficients(
                    PIDFCoefficients(0.13, 0.001, 0.02, 0.02)
                )
                .secondaryTranslationalPIDFCoefficients(
                    PIDFCoefficients(0.1, 0.0001, 0.02, 0.02)
                )
                .headingPIDFCoefficients(
                    PIDFCoefficients(0.7, 0.001, 0.05, 0.03)
                )
                .secondaryHeadingPIDFCoefficients(
                    PIDFCoefficients(1.65, 0.001, 0.015, 0.02)
                )
                .drivePIDFCoefficients(
                    FilteredPIDFCoefficients(0.5, 0.0, 0.01, 0.6, 0.0)
                )
                .secondaryDrivePIDFCoefficients(
                    FilteredPIDFCoefficients(0.08, 0.001, 0.001, 0.6, 0.0)
                )

            @JvmField
            val TANK: FollowerConstants = common()
                .forwardZeroPowerAcceleration(-37.5)
                .lateralZeroPowerAcceleration(-65.7)
                .translationalPIDFCoefficients(
                    PIDFCoefficients(0.13, 0.001, 0.02, 0.02)
                )
                .secondaryTranslationalPIDFCoefficients(
                    PIDFCoefficients(0.1, 0.0001, 0.02, 0.02)
                )
                .headingPIDFCoefficients(
                    PIDFCoefficients(0.7, 0.001, 0.05, 0.03)
                )
                .secondaryHeadingPIDFCoefficients(
                    PIDFCoefficients(1.65, 0.001, 0.015, 0.02)
                )
                .drivePIDFCoefficients(
                    FilteredPIDFCoefficients(0.5, 0.0, 0.01, 0.6, 0.0)
                )
                .secondaryDrivePIDFCoefficients(
                    FilteredPIDFCoefficients(0.08, 0.001, 0.001, 0.6, 0.0)
                )
        }

        object Drive {
            /* ---- MECANUM ---- */
            private fun common(): MecanumConstants {
                return MecanumConstants()
                    .maxPower(1.0)
                    .leftFrontMotorName(Hardware.LEFT_FRONT_MOTOR)
                    .leftRearMotorName(Hardware.LEFT_REAR_MOTOR)
                    .rightFrontMotorName(Hardware.RIGHT_FRONT_MOTOR)
                    .rightRearMotorName(Hardware.RIGHT_REAR_MOTOR)
                    .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
                    .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
                    .useBrakeModeInTeleOp(false)
            }

            @JvmField
            val MECANUM: MecanumConstants = common()
                .xVelocity(86.0)
                .yVelocity(60.0)

            /* ---- TANK ---- */
            @JvmField
            val TANK: MecanumConstants = common()
                .xVelocity(86.0)
                .yVelocity(60.0)
        }

        object Localizer {
            @JvmField
            val PINPOINT: PinpointConstants = PinpointConstants()
                .forwardPodY(-4.75)
                .strafePodX(-4.05)
                .distanceUnit(DistanceUnit.INCH)
                .hardwareMapName(Hardware.PINPOINT)
                .encoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
                )
                .forwardEncoderDirection(
                    GoBildaPinpointDriver.EncoderDirection.REVERSED
                )
                .strafeEncoderDirection(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD
                )
        }

        object Path {
            @JvmField
            var MECANUM = PathConstraints(
                0.995,
                0.01,
                0.01,
                0.001,
                80.0,
                1.00,
                10.0,
                1.0
            )

            @JvmField
            var TANK = PathConstraints(
                0.995,
                0.01,
                0.01,
                0.001,
                80.0,
                1.00,
                10.0,
                1.0
            )
        }
    }

    object Drivetrain {
        // Butterfly Drivetrain Servo Positions
        @JvmField
        var MECANUM_DOWN_POSITION = 0.2
        @JvmField
        var TANK_DOWN_POSITION = 0.6
    }

    /**
     * Simple robot physical measurements.
     */
    object Dimensions {
        const val WIDTH = 16f
        const val LENGTH = 16f
    }

    /**
     * HardwareMap device names.
     */
    object Hardware {
        const val PINPOINT = "pinpoint"
        const val LEFT_FRONT_MOTOR = "leftFront"
        const val LEFT_REAR_MOTOR = "leftRear"
        const val RIGHT_FRONT_MOTOR = "rightFront"
        const val RIGHT_REAR_MOTOR = "rightRear"
        const val BUTTERFLY = "butterfly"
    }

    object Autonomous {
        const val DURATION = 30.0
    }

    object Positions {
        object BotPoses {
            @JvmField
            var START_FAR = Pose()
            @JvmField
            var START_CLOSE = Pose()
        }

        object TeleopPresets {
            @JvmField
            val CLOSE_SHOOT = Pose(54.92, 86.55, Math.toRadians(130.6))
            @JvmField
            val FAR_SHOOT = Pose(60.0, 18.0, Math.toRadians(112.75))
            @JvmField
            val HUMAN_PLAYER = Pose(30.0, 30.0, Math.toRadians(225.0))
            @JvmField
            val GATE = Pose(12.44, 62.0, Math.toRadians(150.0))
            @JvmField
            val PARK = Pose(106.0, 32.0, Math.toRadians(180.0))
        }
    }

    /**
     * Logging configuration.
     */
    object Logging {
        val INTERVAL = if (Flags.DEBUG) 50 else 1000
        val DRAW_FIELD = Flags.DEBUG
        @JvmField
        val followerLook = Style("", "#FFD40C", 0.75)
        @JvmField
        val robotLook = Style("", "#4CAF50", 0.75)
    }
}
