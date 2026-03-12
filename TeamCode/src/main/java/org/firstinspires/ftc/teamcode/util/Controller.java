package org.firstinspires.ftc.teamcode.util;

import com.bylazar.gamepad.PanelsGamepad;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.frozenmilk.dairy.core.util.supplier.logical.EnhancedBooleanSupplier;
import dev.frozenmilk.dairy.core.util.supplier.numeric.EnhancedDoubleSupplier;
import dev.frozenmilk.dairy.pasteurized.PasteurizedGamepad;
import dev.frozenmilk.dairy.pasteurized.SDKGamepad;

public class Controller {
    public final PasteurizedGamepad<EnhancedDoubleSupplier, EnhancedBooleanSupplier> main, sub;

    public Controller(Gamepad gamepad1, Gamepad gamepad2) {
        main = new SDKGamepad(PanelsGamepad.INSTANCE.getFirstManager().asCombinedFTCGamepad(gamepad1));
        sub = new SDKGamepad(PanelsGamepad.INSTANCE.getSecondManager().asCombinedFTCGamepad(gamepad2));
    }
}
