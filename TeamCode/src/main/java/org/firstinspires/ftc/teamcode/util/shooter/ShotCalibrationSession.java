package org.firstinspires.ftc.teamcode.util.shooter;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Collects successful shot samples and fits polynomial models from them.
 */
public class ShotCalibrationSession {
    public static final class Sample {
        public final double distanceInches;
        public final double hoodAngleRadians;
        public final double flywheelRpm;

        public Sample(double distanceInches, double hoodAngleRadians, double flywheelRpm) {
            this.distanceInches = distanceInches;
            this.hoodAngleRadians = hoodAngleRadians;
            this.flywheelRpm = flywheelRpm;
        }
    }

    private final List<Sample> samples = new ArrayList<>();
    private int polynomialDegree = 2;

    public void setPolynomialDegree(int polynomialDegree) {
        this.polynomialDegree = Math.max(1, Math.min(3, polynomialDegree));
    }

    public int getPolynomialDegree() {
        return polynomialDegree;
    }

    public int getMinimumSampleCount() {
        return polynomialDegree + 1;
    }

    public void addSample(double distanceInches, double hoodAngleRadians, double flywheelRpm) {
        samples.add(new Sample(distanceInches, hoodAngleRadians, flywheelRpm));
    }

    public boolean removeLastSample() {
        if (samples.isEmpty()) {
            return false;
        }
        samples.remove(samples.size() - 1);
        return true;
    }

    public void clearSamples() {
        samples.clear();
    }

    public int getSampleCount() {
        return samples.size();
    }

    public List<Sample> getSamples() {
        return Collections.unmodifiableList(samples);
    }

    public double[] fitAngleCoefficients() {
        return fit(samples, sample -> sample.hoodAngleRadians);
    }

    public double[] fitRpmCoefficients() {
        return fit(samples, sample -> sample.flywheelRpm);
    }

    public boolean hasValidFit() {
        return fitAngleCoefficients() != null && fitRpmCoefficients() != null;
    }

    public String formatSettingsSnippet() {
        double[] angleCoefficients = fitAngleCoefficients();
        double[] rpmCoefficients = fitRpmCoefficients();
        if (angleCoefficients == null || rpmCoefficients == null) {
            int remaining = getMinimumSampleCount() - getSampleCount();
            return "Need " + Math.max(remaining, 0) + " more successful sample(s) for degree "
                    + polynomialDegree + " fit.";
        }
        return PolynomialShooterModels.formatSettingsSnippet(angleCoefficients, rpmCoefficients);
    }

    private interface SampleValue {
        double get(Sample sample);
    }

    private double[] fit(List<Sample> source, SampleValue valueSelector) {
        if (source.size() < getMinimumSampleCount()) {
            return null;
        }

        double[] distances = new double[source.size()];
        double[] values = new double[source.size()];
        for (int i = 0; i < source.size(); i++) {
            Sample sample = source.get(i);
            distances[i] = sample.distanceInches;
            values[i] = valueSelector.get(sample);
        }
        return PolynomialRegression.fit(distances, values, polynomialDegree);
    }
}
