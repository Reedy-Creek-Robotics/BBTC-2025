package org.firstinspires.ftc.teamcode.constants;

/**
 * Centralized shooter PIDF values and target velocities.
 * Previously scattered across BaseTeleOp, BaseAutonomus, MecanumDriveClose, MecanumDriveFar.
 *
 * Each ShotType holds its own PIDF coefficients and target TPS so you only
 * need to change values in ONE place.
 */
public final class ShooterConstants {
    private ShooterConstants() {}

    /** How close (in TPS) the motor must be to target before we consider it "ready". */
    public static final double VELOCITY_TOLERANCE = 50;

    /** How long (ms) to run the feed before stopping a shoot sequence. */
    public static final double SHOOT_DURATION_MS = 2500;

    /**
     * Shot presets. Each entry stores its own PIDF coefficients and target velocity.
     *
     * Usage:
     *   ShotType shot = ShotType.LONG;
     *   shooter.setVelocityPIDFCoefficients(shot.p, shot.i, shot.d, shot.f);
     *   shooter.setVelocity(shot.tps);
     */
    public enum ShotType {
        NONE      (0,   0, 0,    0,    0),
        LONG      (75,  0, 0,  6.5, 1000),
        SHORT     (28,  0, 0, 10.5,  900),
        MID       (28,  0, 0,   13,  900),
        EMERGENCY (80,  0, 0,   20, 1000);

        public final double p, i, d, f, tps;

        ShotType(double p, double i, double d, double f, double tps) {
            this.p   = p;
            this.i   = i;
            this.d   = d;
            this.f   = f;
            this.tps = tps;
        }
    }
}
