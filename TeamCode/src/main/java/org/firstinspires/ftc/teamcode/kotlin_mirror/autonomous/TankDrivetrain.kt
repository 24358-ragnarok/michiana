package org.firstinspires.ftc.teamcode.kotlin_mirror.autonomous

import com.pedropathing.Drivetrain
import com.pedropathing.ftc.drivetrains.MecanumConstants
import com.pedropathing.math.Vector
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.sin

/**
 * A custom Drivetrain implementation for PedroPathing that supports Tank (Differential) drive.
 *
 * This class adapts the holonomic pathing requests from PedroPathing into differential
 * drive commands suitable for a tank drive robot.
 *
 * Note: This is an experimental implementation.
 */
class TankDrivetrain(hardwareMap: HardwareMap, var constants: MecanumConstants) : Drivetrain() {
    private val leftFront: DcMotorEx
    private val leftRear: DcMotorEx
    private val rightFront: DcMotorEx
    private val rightRear: DcMotorEx
    val motors: List<DcMotorEx>
    private val voltageSensor: VoltageSensor
    private var motorCachingThreshold: Double
    private var useBrakeModeInTeleOp: Boolean
    private var staticFrictionCoefficient: Double = 0.0

    // Tuning scalar for converting lateral cross-track error into rotational commands
    private val kLateral = 1.5

    init {
        this.maxPowerScaling = constants.maxPower
        this.motorCachingThreshold = constants.motorCachingThreshold
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp

        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        leftFront = hardwareMap.get(DcMotorEx::class.java, constants.leftFrontMotorName)
        leftRear = hardwareMap.get(DcMotorEx::class.java, constants.leftRearMotorName)
        rightFront = hardwareMap.get(DcMotorEx::class.java, constants.rightFrontMotorName)
        rightRear = hardwareMap.get(DcMotorEx::class.java, constants.rightRearMotorName)

        motors = listOf(leftFront, leftRear, rightFront, rightRear)

        for (motor in motors) {
            val motorConfigurationType = motor.motorType.clone()
            motorConfigurationType.achieveableMaxRPMFraction = 1.0
            motor.motorType = motorConfigurationType
        }

        setMotorsToFloat()
        breakFollowing()
    }

    fun updateConstants() {
        leftFront.direction = constants.leftFrontMotorDirection
        leftRear.direction = constants.leftRearMotorDirection
        rightFront.direction = constants.rightFrontMotorDirection
        rightRear.direction = constants.rightRearMotorDirection
        this.motorCachingThreshold = constants.motorCachingThreshold
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp
        this.voltageCompensation = constants.useVoltageCompensation
        this.nominalVoltage = constants.nominalVoltage
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient
    }

    /**
     * Calculates the motor powers required to follow the path.
     *
     * Converts the holonomic corrective, heading, and pathing vectors into
     * left and right wheel powers for a differential drive.
     *
     * @param correctivePower The power vector to correct position error.
     * @param headingPower    The power vector to correct heading error.
     * @param pathingPower    The power vector to follow the path.
     * @param robotHeading    The current robot heading.
     * @return An array of 4 motor powers [LF, LR, RF, RR].
     */
    override fun calculateDrive(
        correctivePower: Vector,
        headingPower: Vector,
        pathingPower: Vector,
        robotHeading: Double
    ): DoubleArray {
        if (correctivePower.magnitude > maxPowerScaling) correctivePower.magnitude = maxPowerScaling
        if (headingPower.magnitude > maxPowerScaling) headingPower.magnitude = maxPowerScaling
        if (pathingPower.magnitude > maxPowerScaling) pathingPower.magnitude = maxPowerScaling

        val globalTranslation = correctivePower.plus(pathingPower)
        val localTheta = globalTranslation.theta - robotHeading

        val vx = globalTranslation.magnitude * cos(localTheta)
        val vy = globalTranslation.magnitude * sin(localTheta)

        // In tank drive, lateral error (vy) cannot be corrected directly by strafing.
        // We convert it into a turning command to steer back to the path.
        val turnCrossTrack = vy * kLateral

        // Extract longitudinal differential demand from heading vector
        val turnHeading = headingPower.magnitude * cos(headingPower.theta - robotHeading)

        var leftPower = vx - turnHeading - turnCrossTrack
        var rightPower = vx + turnHeading + turnCrossTrack

        // Scale up since we have 4 motors but only 2 sides
        leftPower *= 2.0
        rightPower *= 2.0

        val wheelPowers = DoubleArray(4)
        wheelPowers[0] = leftPower  // leftFront
        wheelPowers[1] = leftPower  // leftRear
        wheelPowers[2] = rightPower // rightFront
        wheelPowers[3] = rightPower // rightRear

        if (voltageCompensation) {
            val voltageNormalized = voltageNormalized
            for (i in wheelPowers.indices) {
                wheelPowers[i] *= voltageNormalized
            }
        }

        val wheelPowerMax = max(
            max(abs(wheelPowers[0]), abs(wheelPowers[1])),
            max(abs(wheelPowers[2]), abs(wheelPowers[3]))
        )

        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] = (wheelPowers[0] / wheelPowerMax) * maxPowerScaling
            wheelPowers[1] = (wheelPowers[1] / wheelPowerMax) * maxPowerScaling
            wheelPowers[2] = (wheelPowers[2] / wheelPowerMax) * maxPowerScaling
            wheelPowers[3] = (wheelPowers[3] / wheelPowerMax) * maxPowerScaling
        }

        return wheelPowers
    }

    private fun setMotorsToBrake() {
        for (motor in motors) {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }
    }

    private fun setMotorsToFloat() {
        for (motor in motors) {
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        }
    }

    fun breakFollowing() {
        for (motor in motors) {
            motor.power = 0.0
        }
        setMotorsToFloat()
    }

    fun runDrive(drivePowers: DoubleArray) {
        for (i in motors.indices) {
            if (abs(motors[i].power - drivePowers[i]) > motorCachingThreshold) {
                motors[i].power = drivePowers[i]
            }
        }
    }

    override fun startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake()
        }
    }

    override fun startTeleopDrive(brakeMode: Boolean) {
        if (brakeMode) {
            setMotorsToBrake()
        } else {
            setMotorsToFloat()
        }
    }

    fun getAndRunDrivePowers(
        correctivePower: Vector,
        headingPower: Vector,
        pathingPower: Vector,
        robotHeading: Double
    ) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading))
    }

    var xVelocity: Double
        get() = constants.xVelocity
        set(xMovement) {
            constants.setXVelocity(xMovement)
        }

    var yVelocity: Double
        get() = constants.yVelocity
        set(yMovement) {
            constants.setYVelocity(yMovement)
        }

    fun getStaticFrictionCoefficient(): Double {
        return staticFrictionCoefficient
    }

    override fun getVoltage(): Double {
        return voltageSensor.voltage
    }

    private val voltageNormalized: Double
        get() {
            val voltage = voltage
            return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient))
        }

    fun debugString(): String {
        return "Tank{" +
                " leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                ", rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}'
    }
}
