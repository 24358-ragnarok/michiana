package org.firstinspires.ftc.teamcode.util.shooter;

/**
 * Least-squares polynomial fit for shooter calibration.
 */
public final class PolynomialRegression {
    private PolynomialRegression() {
    }

    /**
     * Fits {@code degree}-order polynomial coefficients for y = c0 + c1*x + c2*x^2
     * + ...
     *
     * @return coefficients ordered from constant term upward, or null if
     *         underdetermined or singular
     */
    public static double[] fit(double[] x, double[] y, int degree) {
        if (x == null || y == null || x.length != y.length || x.length == 0) {
            return null;
        }

        int sampleCount = x.length;
        int termCount = degree + 1;
        if (sampleCount < termCount) {
            return null;
        }

        double[][] normalMatrix = new double[termCount][termCount];
        double[] normalVector = new double[termCount];

        for (int i = 0; i < sampleCount; i++) {
            double[] row = powers(x[i], termCount);
            for (int j = 0; j < termCount; j++) {
                normalVector[j] += row[j] * y[i];
                for (int k = 0; k < termCount; k++) {
                    normalMatrix[j][k] += row[j] * row[k];
                }
            }
        }

        return solveLinearSystem(normalMatrix, normalVector);
    }

    public static double evaluate(double[] coefficients, double x) {
        if (coefficients == null || coefficients.length == 0) {
            return 0.0;
        }

        double result = 0.0;
        double power = 1.0;
        for (double coefficient : coefficients) {
            result += coefficient * power;
            power *= x;
        }
        return result;
    }

    public static String formatCoefficients(String name, double[] coefficients) {
        if (coefficients == null) {
            return name + " = null;";
        }

        StringBuilder builder = new StringBuilder();
        builder.append("public static double[] ").append(name).append(" = new double[] {");
        for (int i = 0; i < coefficients.length; i++) {
            if (i > 0) {
                builder.append(", ");
            }
            builder.append(String.format(java.util.Locale.US, "%.6f", coefficients[i]));
        }
        builder.append("};");
        return builder.toString();
    }

    private static double[] powers(double x, int termCount) {
        double[] row = new double[termCount];
        row[0] = 1.0;
        for (int i = 1; i < termCount; i++) {
            row[i] = row[i - 1] * x;
        }
        return row;
    }

    private static double[] solveLinearSystem(double[][] matrix, double[] vector) {
        int size = vector.length;
        double[][] augmented = new double[size][size + 1];
        for (int row = 0; row < size; row++) {
            System.arraycopy(matrix[row], 0, augmented[row], 0, size);
            augmented[row][size] = vector[row];
        }

        for (int pivotRow = 0; pivotRow < size; pivotRow++) {
            int bestRow = pivotRow;
            for (int row = pivotRow + 1; row < size; row++) {
                if (Math.abs(augmented[row][pivotRow]) > Math.abs(augmented[bestRow][pivotRow])) {
                    bestRow = row;
                }
            }

            if (Math.abs(augmented[bestRow][pivotRow]) < 1e-9) {
                return null;
            }

            if (bestRow != pivotRow) {
                double[] swap = augmented[pivotRow];
                augmented[pivotRow] = augmented[bestRow];
                augmented[bestRow] = swap;
            }

            double pivot = augmented[pivotRow][pivotRow];
            for (int col = pivotRow; col <= size; col++) {
                augmented[pivotRow][col] /= pivot;
            }

            for (int row = 0; row < size; row++) {
                if (row == pivotRow) {
                    continue;
                }
                double factor = augmented[row][pivotRow];
                for (int col = pivotRow; col <= size; col++) {
                    augmented[row][col] -= factor * augmented[pivotRow][col];
                }
            }
        }

        double[] solution = new double[size];
        for (int row = 0; row < size; row++) {
            solution[row] = augmented[row][size];
        }
        return solution;
    }
}
