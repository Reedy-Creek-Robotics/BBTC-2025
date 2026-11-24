package org.firstinspires.ftc.teamcode.Subsystems;

/** Drive calculations that can be checked without connecting any motors. */
public final class DriveMath {
    private DriveMath() { }

    /**
     * Mixes the three driver commands into four wheel powers.
     * If any wheel needs more than full power, scale all four by the same amount
     * so the requested direction is preserved.
     */
    public static WheelPowers mecanumPowers(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));

        return new WheelPowers(fl / maxPower, fr / maxPower,
                bl / maxPower, br / maxPower);
    }

    public static final class WheelPowers {
        public final double frontLeft;
        public final double frontRight;
        public final double backLeft;
        public final double backRight;

        private WheelPowers(double frontLeft, double frontRight,
                            double backLeft, double backRight) {
            this.frontLeft = frontLeft;
            this.frontRight = frontRight;
            this.backLeft = backLeft;
            this.backRight = backRight;
        }
    }
}
