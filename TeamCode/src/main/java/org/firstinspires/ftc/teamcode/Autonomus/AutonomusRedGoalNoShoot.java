package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto : Red Goal NO-shoot")
public class AutonomusRedGoalNoShoot extends BaseAutonomus {


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
        moveForward(3, 0.6);
        rotate(-55, TURN_SPEED);
        moveForward(8, DRIVE_SPEED);

        // End
        stopAll();
    }
}
