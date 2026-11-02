package org.firstinspires.ftc.teamcode.mechanisms;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

public class VisionMathTest {
    private static final double EPSILON = 0.000000001;

    @Test
    public void theOriginHasZeroDistance() {
        assertEquals(0, VisionMath.distanceInchesFromMeters(0, 0, 0), EPSILON);
    }

    @Test
    public void oneMeterOnAnyAxisIs39Point37Inches() {
        assertEquals(39.37, VisionMath.distanceInchesFromMeters(1, 0, 0), EPSILON);
        assertEquals(39.37, VisionMath.distanceInchesFromMeters(0, 1, 0), EPSILON);
        assertEquals(39.37, VisionMath.distanceInchesFromMeters(0, 0, 1), EPSILON);
    }

    @Test
    public void aThirtyByFortyCentimeterTriangleIsHalfAMeterAway() {
        assertEquals(19.685, VisionMath.distanceInchesFromMeters(0.3, 0.4, 0), EPSILON);
    }

    @Test
    public void negativeCoordinatesStillProduceAPositiveDistance() {
        assertEquals(19.685, VisionMath.distanceInchesFromMeters(-0.3, -0.4, 0), EPSILON);
    }

    @Test
    public void allThreeCoordinatesContributeToTheDistance() {
        // A 1-by-2-by-2 vector has length 3 meters.
        assertEquals(118.11, VisionMath.distanceInchesFromMeters(1, 2, 2), EPSILON);
    }
}
