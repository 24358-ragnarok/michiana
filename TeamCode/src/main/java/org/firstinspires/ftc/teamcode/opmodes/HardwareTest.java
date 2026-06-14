package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * A comprehensive hardware testing OpMode.
 * <p>
 * Use Dpad Up/Down to cycle through hardware devices.
 * Use gamepad buttons/sticks to control the selected device.
 */
@TeleOp(name = "Hardware Test", group = "z: Utilities")
public class HardwareTest extends OpMode {

    private List<HardwareDevice> devices;
    private List<String> deviceNames;
    private int selectedIndex = 0;

    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastX = false;
    private boolean lastY = false;

    private double servoPosition = 0.5;
    private boolean motorModeRunToPosition = false;

    @Override
    public void init() {
        devices = new ArrayList<>();
        deviceNames = new ArrayList<>();

        // Collect all devices from the hardware map using allDeviceMappings
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> mapping : hardwareMap.allDeviceMappings) {
            for (Map.Entry<String, ? extends HardwareDevice> entry : mapping.entrySet()) {
                deviceNames.add(entry.getKey());
                devices.add((HardwareDevice) entry.getValue());
            }
        }

        if (devices.isEmpty()) {
            telemetry.addLine("NO DEVICES FOUND IN HARDWARE MAP");
        }
    }

    @Override
    public void loop() {
        if (devices.isEmpty())
            return;

        // Navigation
        if (gamepad1.dpad_up && !lastDpadUp) {
            selectedIndex = (selectedIndex - 1 + devices.size()) % devices.size();
            resetState();
        }
        if (gamepad1.dpad_down && !lastDpadDown) {
            selectedIndex = (selectedIndex + 1) % devices.size();
            resetState();
        }
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;

        HardwareDevice device = devices.get(selectedIndex);
        String name = deviceNames.get(selectedIndex);

        telemetry.addLine("--- HARDWARE TEST ---");
        telemetry.addData("Selected Device", "[%d/%d] %s", selectedIndex + 1, devices.size(), name);
        telemetry.addData("Type", device.getClass().getSimpleName());
        telemetry.addLine("Use Dpad Up/Down to cycle devices");
        telemetry.addLine();

        handleDevice(device);

        telemetry.update();
    }

    private void resetState() {
        servoPosition = 0.5;
        motorModeRunToPosition = false;
        // Stop any currently running motors if we switch
        for (HardwareDevice d : devices) {
            if (d instanceof DcMotor)
                ((DcMotor) d).setPower(0);
            if (d instanceof CRServo)
                ((CRServo) d).setPower(0);
        }
    }

    private void handleDevice(HardwareDevice device) {
        if (device instanceof DcMotor) {
            handleMotor((DcMotor) device);
        } else if (device instanceof Servo) {
            handleServo((Servo) device);
        } else if (device instanceof CRServo) {
            handleCRServo((CRServo) device);
        } else if (device instanceof TouchSensor) {
            telemetry.addData("Pressed", ((TouchSensor) device).isPressed());
        } else if (device instanceof ColorSensor) {
            ColorSensor color = (ColorSensor) device;
            telemetry.addData("RGBA", "%d, %d, %d, %d", color.red(), color.green(), color.blue(), color.alpha());
        } else if (device instanceof DistanceSensor) {
            telemetry.addData("Distance (in)", ((DistanceSensor) device).getDistance(DistanceUnit.INCH));
        } else {
            telemetry.addLine("No specific controls for this device type.");
        }
    }

    private void handleMotor(DcMotor motor) {
        telemetry.addLine("Stick Left Y: Power | A: Reset Encoder");
        telemetry.addLine("X: Toggle RunToPosition | B: Set Target (Current)");

        if (gamepad1.a) {
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        if (gamepad1.x && !lastX) {
            motorModeRunToPosition = !motorModeRunToPosition;
            motor.setMode(
                    motorModeRunToPosition ? DcMotor.RunMode.RUN_TO_POSITION : DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        lastX = gamepad1.x;

        if (motorModeRunToPosition) {
            if (gamepad1.b) {
                motor.setTargetPosition(motor.getCurrentPosition());
            }
            motor.setPower(Math.abs(gamepad1.left_stick_y));
            telemetry.addData("Mode", "RUN_TO_POSITION");
            telemetry.addData("Target", motor.getTargetPosition());
        } else {
            motor.setPower(-gamepad1.left_stick_y);
            telemetry.addData("Mode", motor.getMode());
        }

        telemetry.addData("Power", motor.getPower());
        telemetry.addData("Position", motor.getCurrentPosition());

        if (motor instanceof DcMotorEx) {
            telemetry.addData("Velocity", ((DcMotorEx) motor).getVelocity());
        }
    }

    private void handleServo(Servo servo) {
        telemetry.addLine("Stick Left Y: Adjust Position | X: 0 | Y: 1 | B: 0.5");

        if (Math.abs(gamepad1.left_stick_y) > 0.05) {
            servoPosition -= gamepad1.left_stick_y * 0.01;
        }
        if (gamepad1.x)
            servoPosition = 0;
        if (gamepad1.y)
            servoPosition = 1;
        if (gamepad1.b)
            servoPosition = 0.5;

        servoPosition = Math.max(0, Math.min(1, servoPosition));
        servo.setPosition(servoPosition);

        telemetry.addData("Target Position", "%.3f", servoPosition);
        telemetry.addData("Actual Position", "%.3f", servo.getPosition());
    }

    private void handleCRServo(CRServo servo) {
        telemetry.addLine("Stick Left Y: Power");
        servo.setPower(-gamepad1.left_stick_y);
        telemetry.addData("Power", servo.getPower());
    }
}
