package org.firstinspires.ftc.teamcode.mechanisms;

import org.junit.Test;
import static org.junit.Assert.fail;

/** Temporary integration probe in the draft pilot only. Remove after CI verification. */
public class CiFailureProbeTest {
    @Test
    public void demonstratesThatCiRejectsAFailingTest() {
        fail("Intentional CI verification probe; this is not a robot-code defect.");
    }
}
