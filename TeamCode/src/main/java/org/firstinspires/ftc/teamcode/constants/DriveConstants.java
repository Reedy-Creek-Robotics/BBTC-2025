package org.firstinspires.ftc.teamcode.constants;

/**
 * Centralized drive constants shared between TeleOp and Autonomous.
 * Previously duplicated in BaseTeleOp and BaseAutonomus.
 */
public final class DriveConstants {
    private DriveConstants() {}

    // ---------- Encoder-based drive constants ----------
    public static final double COUNTS_PER_MOTOR_REV  = 537.7;
    public static final double DRIVE_GEAR_REDUCTION  = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.25;
    public static final double HALF_OF_BOT_LENGTH    = 8.5;

    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION)
                    / (WHEEL_DIAMETER_INCHES * Math.PI);

    // ---------- Speed presets ----------
    public static final double DRIVE_SPEED  = 0.9;
    public static final double TURN_SPEED   = 0.4;
    public static final double INTAKE_SPEED = 0.5;

    // ---------- TeleOp slew-rate limits ----------
    public static final double MAX_ACCEL = 0.08;
    public static final double MAX_DECEL = 0.12;

    // ---------- Reversal braking ----------
    public static final long REVERSAL_DELAY_MS = 120;

    // ---------- Turn geometry (encoder-based autonomous) ----------
    public static final double TURN_DIAMETER_INCHES = 23.5;
}
