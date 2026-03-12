package org.firstinspires.ftc.teamcode.autonomous;

import com.pedropathing.Drivetrain;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

/**
 * A custom Drivetrain implementation for PedroPathing that supports Tank (Differential) drive.
 * <p>
 * This class adapts the holonomic pathing requests from PedroPathing into differential
 * drive commands suitable for a tank drive robot.
 * <p>
 * Note: This is an experimental implementation.
 */
public class TankDrivetrain extends Drivetrain {
    public MecanumConstants constants;
    private final DcMotorEx leftFront;
    private final DcMotorEx leftRear;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightRear;
    private final List<DcMotorEx> motors;
    private final VoltageSensor voltageSensor;
    private double motorCachingThreshold;
    private boolean useBrakeModeInTeleOp;
    private double staticFrictionCoefficient;

    // Tuning scalar for converting lateral cross-track error into rotational commands
    private final double kLateral = 1.5;

    /**
     * Initializes the Tank Drivetrain.
     *
     * @param hardwareMap   The hardware map.
     * @param tankConstants The constants defining motor names and properties.
     */
    public TankDrivetrain(HardwareMap hardwareMap, MecanumConstants tankConstants) {
        constants = tankConstants;

        this.maxPowerScaling = tankConstants.maxPower;
        this.motorCachingThreshold = tankConstants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = tankConstants.useBrakeModeInTeleOp;

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareMap.get(DcMotorEx.class, tankConstants.leftFrontMotorName);
        leftRear = hardwareMap.get(DcMotorEx.class, tankConstants.leftRearMotorName);
        rightFront = hardwareMap.get(DcMotorEx.class, tankConstants.rightFrontMotorName);
        rightRear = hardwareMap.get(DcMotorEx.class, tankConstants.rightRearMotorName);

        motors = Arrays.asList(leftFront, leftRear, rightFront, rightRear);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }

        setMotorsToFloat();
        breakFollowing();
    }

    public void updateConstants() {
        leftFront.setDirection(constants.leftFrontMotorDirection);
        leftRear.setDirection(constants.leftRearMotorDirection);
        rightFront.setDirection(constants.rightFrontMotorDirection);
        rightRear.setDirection(constants.rightRearMotorDirection);
        this.motorCachingThreshold = constants.motorCachingThreshold;
        this.useBrakeModeInTeleOp = constants.useBrakeModeInTeleOp;
        this.voltageCompensation = constants.useVoltageCompensation;
        this.nominalVoltage = constants.nominalVoltage;
        this.staticFrictionCoefficient = constants.staticFrictionCoefficient;
    }

    /**
     * Calculates the motor powers required to follow the path.
     * <p>
     * Converts the holonomic corrective, heading, and pathing vectors into
     * left and right wheel powers for a differential drive.
     *
     * @param correctivePower The power vector to correct position error.
     * @param headingPower    The power vector to correct heading error.
     * @param pathingPower    The power vector to follow the path.
     * @param robotHeading    The current robot heading.
     * @return An array of 4 motor powers [LF, LR, RF, RR].
     */
    @Override
    public double[] calculateDrive(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        if (correctivePower.getMagnitude() > maxPowerScaling)
            correctivePower.setMagnitude(maxPowerScaling);
        if (headingPower.getMagnitude() > maxPowerScaling)
            headingPower.setMagnitude(maxPowerScaling);
        if (pathingPower.getMagnitude() > maxPowerScaling)
            pathingPower.setMagnitude(maxPowerScaling);

        Vector globalTranslation = correctivePower.plus(pathingPower);
        double localTheta = globalTranslation.getTheta() - robotHeading;

        double vx = globalTranslation.getMagnitude() * Math.cos(localTheta);
        double vy = globalTranslation.getMagnitude() * Math.sin(localTheta);

        // In tank drive, lateral error (vy) cannot be corrected directly by strafing.
        // We convert it into a turning command to steer back to the path.
        double turnCrossTrack = vy * kLateral;

        // Extract longitudinal differential demand from heading vector
        double turnHeading = headingPower.getMagnitude() * Math.cos(headingPower.getTheta() - robotHeading);

        double leftPower = vx - turnHeading - turnCrossTrack;
        double rightPower = vx + turnHeading + turnCrossTrack;

        // Scale up since we have 4 motors but only 2 sides
        leftPower *= 2.0;
        rightPower *= 2.0;

        double[] wheelPowers = new double[4];
        wheelPowers[0] = leftPower;  // leftFront
        wheelPowers[1] = leftPower;  // leftRear
        wheelPowers[2] = rightPower; // rightFront
        wheelPowers[3] = rightPower; // rightRear

        if (voltageCompensation) {
            double voltageNormalized = getVoltageNormalized();
            for (int i = 0; i < wheelPowers.length; i++) {
                wheelPowers[i] *= voltageNormalized;
            }
        }

        double wheelPowerMax = Math.max(
                Math.max(Math.abs(wheelPowers[0]), Math.abs(wheelPowers[1])),
                Math.max(Math.abs(wheelPowers[2]), Math.abs(wheelPowers[3]))
        );

        if (wheelPowerMax > maxPowerScaling) {
            wheelPowers[0] = (wheelPowers[0] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[1] = (wheelPowers[1] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[2] = (wheelPowers[2] / wheelPowerMax) * maxPowerScaling;
            wheelPowers[3] = (wheelPowers[3] / wheelPowerMax) * maxPowerScaling;
        }

        return wheelPowers;
    }

    private void setMotorsToBrake() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    private void setMotorsToFloat() {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
    }

    public void breakFollowing() {
        for (DcMotorEx motor : motors) {
            motor.setPower(0);
        }
        setMotorsToFloat();
    }

    public void runDrive(double[] drivePowers) {
        for (int i = 0; i < motors.size(); i++) {
            if (Math.abs(motors.get(i).getPower() - drivePowers[i]) > motorCachingThreshold) {
                motors.get(i).setPower(drivePowers[i]);
            }
        }
    }

    @Override
    public void startTeleopDrive() {
        if (useBrakeModeInTeleOp) {
            setMotorsToBrake();
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        if (brakeMode) {
            setMotorsToBrake();
        } else {
            setMotorsToFloat();
        }
    }

    public void getAndRunDrivePowers(Vector correctivePower, Vector headingPower, Vector pathingPower, double robotHeading) {
        runDrive(calculateDrive(correctivePower, headingPower, pathingPower, robotHeading));
    }

    public double xVelocity() {
        return constants.xVelocity;
    }

    public double yVelocity() {
        return constants.yVelocity;
    }

    public void setXVelocity(double xMovement) {
        constants.setXVelocity(xMovement);
    }

    public void setYVelocity(double yMovement) {
        constants.setYVelocity(yMovement);
    }

    public double getStaticFrictionCoefficient() {
        return staticFrictionCoefficient;
    }

    @Override
    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

    private double getVoltageNormalized() {
        double voltage = getVoltage();
        return (nominalVoltage - (nominalVoltage * staticFrictionCoefficient)) / (voltage - ((nominalVoltage * nominalVoltage / voltage) * staticFrictionCoefficient));
    }

    public String debugString() {
        return "Tank{" +
                " leftFront=" + leftFront +
                ", leftRear=" + leftRear +
                ", rightFront=" + rightFront +
                ", rightRear=" + rightRear +
                ", motors=" + motors +
                ", motorCachingThreshold=" + motorCachingThreshold +
                ", useBrakeModeInTeleOp=" + useBrakeModeInTeleOp +
                '}';
    }

    public List<DcMotorEx> getMotors() {
        return motors;
    }
}
