package org.firstinspires.ftc.teamcode.util.shooter;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.config.Settings;

/**
 * Polynomial distance models for hood angle and flywheel RPM.
 */
public final class PolynomialShooterModels {
    private PolynomialShooterModels() {
    }

    public static Settings.Turret.AngleSolutionModel angleFromCoefficients(double[] coefficients) {
        double[] coeffs = copyCoefficients(coefficients);
        return distanceInches -> {
            double angleRadians = PolynomialRegression.evaluate(coeffs, distanceInches);
            return new Settings.Turret.AngleSolution(angleRadians);
        };
    }

    public static Settings.Turret.RPMSolutionModel rpmFromCoefficients(double[] coefficients) {
        double[] coeffs = copyCoefficients(coefficients);
        return distanceInches -> {
            double rpm = PolynomialRegression.evaluate(coeffs, distanceInches);
            return new Settings.Turret.RPMSolution(rpm);
        };
    }

    public static double predictAngleRadians(double[] coefficients, double distanceInches) {
        return PolynomialRegression.evaluate(coefficients, distanceInches);
    }

    public static double predictFlywheelRpm(double[] coefficients, double distanceInches) {
        return PolynomialRegression.evaluate(coefficients, distanceInches);
    }

    public static double[] defaultAngleCoefficients() {
        double verticalDelta = Settings.Turret.GOAL_HEIGHT_INCHES - Settings.Turret.LAUNCHER_HEIGHT_INCHES;
        double referenceDistance = 60.0;
        double referenceAngle = Math.atan2(verticalDelta, referenceDistance) + Settings.Turret.HOOD_ANGLE_OFFSET_RAD;
        return new double[] {referenceAngle, 0.0, 0.0};
    }

    public static double[] defaultRpmCoefficients() {
        return new double[]{100.0, 0.0, 0.0};
    }

    public static String formatSettingsSnippet(double[] angleCoefficients, double[] rpmCoefficients) {
        String builder = PolynomialRegression.formatCoefficients("ANGLE_COEFFICIENTS", angleCoefficients) +
                "\n" +
                PolynomialRegression.formatCoefficients("RPM_COEFFICIENTS", rpmCoefficients) +
                "\n" +
                "ANGLE_SOLUTION_MODEL = PolynomialShooterModels.angleFromCoefficients(ANGLE_COEFFICIENTS);" +
                "\n" +
                "RPM_SOLUTION_MODEL = PolynomialShooterModels.rpmFromCoefficients(RPM_COEFFICIENTS);";
        return builder;
    }

    public static double clipAngleRadians(double angleRadians) {
        return Range.clip(
                angleRadians,
                Settings.Turret.HOOD_MIN_ANGLE_RAD,
                Settings.Turret.HOOD_MAX_ANGLE_RAD);
    }

    private static double[] copyCoefficients(double[] coefficients) {
        if (coefficients == null || coefficients.length == 0) {
            return new double[] {0.0};
        }
        double[] copy = new double[coefficients.length];
        System.arraycopy(coefficients, 0, copy, 0, coefficients.length);
        return copy;
    }
}
