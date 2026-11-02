package org.firstinspires.ftc.teamcode.mechanisms;

/** Camera calculations that do not need a Limelight or an Android device. */
public final class VisionMath {
    // Keep the conversion already used by Camera; this is not a tuning change.
    private static final double INCHES_PER_METER = 39.37;

    private VisionMath() { }

    /** Straight-line distance from a camera-space position measured in meters. */
    public static double distanceInchesFromMeters(double x, double y, double z) {
        return Math.sqrt(x * x + y * y + z * z) * INCHES_PER_METER;
    }
}
