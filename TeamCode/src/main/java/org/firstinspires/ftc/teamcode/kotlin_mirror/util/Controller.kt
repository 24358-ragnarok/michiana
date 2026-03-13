package org.firstinspires.ftc.teamcode.kotlin_mirror.util

import com.bylazar.gamepad.PanelsGamepad
import com.qualcomm.robotcore.hardware.Gamepad
import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad
import dev.frozenmilk.dairy.pasteurized.SDKGamepad

/**
 * A wrapper class for FTC Gamepads using the Dairy library.
 *
 * This class provides enhanced gamepad functionality, such as rising/falling edge detection
 * and button state management, through the [PasteurizedGamepad] interface.
 * It also integrates with FTControl Panels for virtual gamepad support.
 */
class Controller(gamepad1: Gamepad, gamepad2: Gamepad) {
    /**
     * The primary driver gamepad.
     */
    val main: PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> =
        SDKGamepad(PanelsGamepad.INSTANCE.firstManager.asCombinedFTCGamepad(gamepad1))

    /**
     * The secondary operator gamepad.
     */
    val sub: PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> =
        SDKGamepad(PanelsGamepad.INSTANCE.secondManager.asCombinedFTCGamepad(gamepad2))
}
