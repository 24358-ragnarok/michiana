package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.Gamepad;


/**
 * A wrapper class for FTC Gamepads using the Dairy library.
 * <p>
 * This class provides enhanced gamepad functionality, such as rising/falling edge detection
 * and button state management, through the interface.
 * It also integrates with FTControl Panels for virtual gamepad support.
 */
public class Controller {
    /**
     * The primary driver gamepad.
     */
    public final Gamepad main;

    /**
     * The secondary operator gamepad.
     */
    public final Gamepad sub;

    /**
     * Initializes the controller wrapper.
     * <p>
     * Wraps the standard FTC gamepads with Dairy's enhanced features and Panels integration.
     *
     * @param gamepad1 The first gamepad from the OpMode.
     * @param gamepad2 The second gamepad from the OpMode.
     */
    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        main = gamepad1;
        sub = gamepad2;
    }
}
