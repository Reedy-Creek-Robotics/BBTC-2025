package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;

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

       // sleep(22000);
        shooter_1.setVelocity(SHOOTER_TPS);
        moveForward(45,DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        sleep(500);
        intakeTransfer.setPower(1.0);
        sleep(4500);
        stopShootSequence();
        sleep(15000);

        moveForward(-30, 0.9);
        rotate(55, TURN_SPEED);
        moveForward(17, DRIVE_SPEED);

        // End
        stopAll();
    }
}
