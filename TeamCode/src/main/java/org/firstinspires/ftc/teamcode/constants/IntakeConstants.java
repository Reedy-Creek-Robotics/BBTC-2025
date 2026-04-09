package org.firstinspires.ftc.teamcode.constants;

/**
 * Centralized intake and transfer power values.
 * Previously hardcoded across BaseTeleOp and BaseAutonomus.
 */
public final class IntakeConstants {
    private IntakeConstants() {}

    // Normal intake (transfer belt pulls balls in, servo off)
    public static final double INTAKE_TRANSFER_POWER = 0.75;
    public static final double INTAKE_SERVO_POWER    = 0.0;

    // Reverse intake (eject jammed balls)
    public static final double REVERSE_TRANSFER_POWER = -0.3;
    public static final double REVERSE_SERVO_POWER    = -0.75;

    // Feeding balls to shooter (both at full power)
    public static final double FEED_TRANSFER_POWER = 1.0;
    public static final double FEED_SERVO_POWER    = 1.0;

    // Autonomous intake with servo
    public static final double AUTO_INTAKE_TRANSFER_POWER = 1.0;
    public static final double AUTO_INTAKE_SERVO_POWER    = -1.0;
}
