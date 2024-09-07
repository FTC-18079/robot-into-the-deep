package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    /**
     * Returns value clamped around the low and high boundaries
     * @param value Value to clamp
     * @param low Lower boundary to clamp value
     * @param max Upper boundary to clamp
     * @return The clamped value
     */
    public static int clamp(int value, int low, int max) {
        return Math.max(low, Math.min(value, max));
    }

    /**
     * Returns value clamped around the low and high boundaries
     * @param value Value to clamp
     * @param low Lower boundary to clamp value
     * @param max Upper boundary to clamp
     * @return The clamped value
     */
    public static double clamp(double value, double low, double max) {
        return Math.max(low, Math.min(value, max));
    }

    public static double inputModulus(double input, double min, double max) {
        double modulus = max - min;

        int numMax = (int) ((input - min) / modulus);
        input -= numMax * modulus;

        int numMin = (int) ((input - max) / modulus);
        input -= numMin * modulus;

        return input;
    }

    /**
     * Wraps an angle around -pi to pi radians
     * @param angleRadians Angle to wrap
     * @return The wrapped angle
     */
    public static double angleModulus(double angleRadians) {
        return inputModulus(angleRadians, -Math.PI, Math.PI);
    }
}
