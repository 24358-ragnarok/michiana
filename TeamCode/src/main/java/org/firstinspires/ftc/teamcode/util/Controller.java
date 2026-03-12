package org.firstinspires.ftc.teamcode.util;

import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

/**
 * A wrapper class for FTC Gamepads using the Dairy library.
 * <p>
 * This class provides enhanced gamepad functionality, such as rising/falling edge detection
 * and button state management, through the {@link PasteurizedGamepad} interface.
 * It also integrates with FTControl Panels for virtual gamepad support.
 */
public class Controller {
    /**
     * The primary driver gamepad.
     */
    public final PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> main;

    /**
     * The secondary operator gamepad.
     */
    public final PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> sub;

    /**
     * Initializes the controller wrapper.
     * <p>
     * Wraps the standard FTC gamepads with Dairy's enhanced features and Panels integration.
     *
     * @param gamepad1 The first gamepad from the OpMode.
     * @param gamepad2 The second gamepad from the OpMode.
     */
    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        main = new SDKGamepad(PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1));
        sub = new SDKGamepad(PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(gamepad2));
    }
}
