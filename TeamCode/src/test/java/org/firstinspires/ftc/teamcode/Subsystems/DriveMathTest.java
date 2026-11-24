package org.firstinspires.ftc.teamcode.Subsystems;

import org.junit.Test;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

public class DriveMathTest {
    private static final double EPSILON = 0.000000001;

    @Test
    public void releasedSticksStopAllFourWheels() {
        assertPowers(DriveMath.mecanumPowers(0, 0, 0), 0, 0, 0, 0);
    }

    @Test
    public void forwardUsesEqualPositiveWheelPowers() {
        assertPowers(DriveMath.mecanumPowers(1, 0, 0), 1, 1, 1, 1);
    }

    @Test
    public void backwardUsesEqualNegativeWheelPowers() {
        assertPowers(DriveMath.mecanumPowers(-1, 0, 0), -1, -1, -1, -1);
    }

    @Test
    public void rightStrafeUsesOppositeDiagonals() {
        assertPowers(DriveMath.mecanumPowers(0, 1, 0), 1, -1, -1, 1);
    }

    @Test
    public void leftStrafeReversesBothDiagonals() {
        assertPowers(DriveMath.mecanumPowers(0, -1, 0), -1, 1, 1, -1);
    }

    @Test
    public void positiveRotationUsesOppositeWheelSides() {
        assertPowers(DriveMath.mecanumPowers(0, 0, 1), 1, -1, 1, -1);
    }

    @Test
    public void negativeRotationReversesBothSides() {
        assertPowers(DriveMath.mecanumPowers(0, 0, -1), -1, 1, -1, 1);
    }

    @Test
    public void forwardRightDiagonalUsesOnlyTheMatchingDiagonal() {
        assertPowers(DriveMath.mecanumPowers(1, 1, 0), 1, 0, 0, 1);
    }

    @Test
    public void combinedCommandsScaleAllWheelsTogetherInsteadOfClippingEachOne() {
        assertPowers(DriveMath.mecanumPowers(1, 1, 1),
                1, -1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
    }

    @Test
    public void smallCommandsKeepTheirRequestedPower() {
        assertPowers(DriveMath.mecanumPowers(0.2, 0.3, 0.4), 0.9, -0.5, 0.3, 0.1);
    }

    @Test
    public void validStickCombinationsNeverExceedFullMotorPower() {
        double[] stickPositions = {-1, -0.5, 0, 0.5, 1};
        for (double forward : stickPositions) {
            for (double strafe : stickPositions) {
                for (double rotate : stickPositions) {
                    DriveMath.WheelPowers powers = DriveMath.mecanumPowers(forward, strafe, rotate);
                    String input = "forward=" + forward + ", strafe=" + strafe + ", rotate=" + rotate;
                    assertTrue(input + " front left", Math.abs(powers.frontLeft) <= 1 + EPSILON);
                    assertTrue(input + " front right", Math.abs(powers.frontRight) <= 1 + EPSILON);
                    assertTrue(input + " back left", Math.abs(powers.backLeft) <= 1 + EPSILON);
                    assertTrue(input + " back right", Math.abs(powers.backRight) <= 1 + EPSILON);
                }
            }
        }
    }

    @Test
    public void reversingEveryCommandReversesEveryWheel() {
        DriveMath.WheelPowers powers = DriveMath.mecanumPowers(0.8, -0.4, 0.6);
        assertPowers(DriveMath.mecanumPowers(-0.8, 0.4, -0.6),
                -powers.frontLeft, -powers.frontRight, -powers.backLeft, -powers.backRight);
    }

    private static void assertPowers(DriveMath.WheelPowers actual,
                                     double frontLeft, double frontRight,
                                     double backLeft, double backRight) {
        assertEquals("Front left", frontLeft, actual.frontLeft, EPSILON);
        assertEquals("Front right", frontRight, actual.frontRight, EPSILON);
        assertEquals("Back left", backLeft, actual.backLeft, EPSILON);
        assertEquals("Back right", backRight, actual.backRight, EPSILON);
    }
}
