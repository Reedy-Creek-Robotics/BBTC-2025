package org.firstinspires.ftc.teamcode.mechanisms;

/** Drive calculations that can be checked without connecting any motors. */
public final class DriveMath {
    private DriveMath() { }

    /**
     * Mixes the three driver commands into four wheel powers.
     * If any wheel needs more than full power, scale all four by the same amount
     * so the requested direction is preserved.
     */
    public static WheelPowers mecanumPowers(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));

        return new WheelPowers(fl / maxPower, fr / maxPower,
                bl / maxPower, br / maxPower);
    }

    /**
     * Moves toward a target by at most one allowed step per call.
     * The limits describe increasing/decreasing numbers, including below zero;
     * they preserve the existing TeleOp ramp rather than retuning acceleration.
     */
    public static double limitChange(double current, double target,
                                     double positiveStep, double negativeStep) {
        double delta = target - current;
        if (delta > 0) delta = Math.min(delta, positiveStep);
        else delta = Math.max(delta, -negativeStep);
        return current + delta;
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
