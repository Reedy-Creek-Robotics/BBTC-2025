package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Autonomous : Straight No shoot (Blue)")
public class AutonomusStraightBlueNoShoot extends BaseAutonomus {


    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS STEPS
        // -------------------------------

        sleep(22);
        moveForward(5, 0.9);
        rotate(-90, TURN_SPEED);
        moveForward(17, 0.9);

        // End
        stopAll();
    }
}
