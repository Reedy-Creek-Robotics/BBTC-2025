package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto : Blue Goal NO-shoot")
public class AutonomusBlueGoalNoShoot extends BaseAutonomus {


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

        sleep(22000);
        moveForward(5, 0.6);
        rotate(55, TURN_SPEED);
        moveForward(17, DRIVE_SPEED);

        // End
        stopAll();
    }
}
