package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto : Blue Straight NO-shoot")
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
        moveForward(5, 0.6);
        rotate(-90, TURN_SPEED);
        moveForward(17, DRIVE_SPEED);

        // End
        stopAll();
    }
}
