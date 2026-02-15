package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;
//RIGHT ONE

@Autonomous(name = "Auto : Red Straight Shoot")
public class AutonomusStraightRedShoot extends BaseAutonomus {

    // Drive motors

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

        farShot();
        moveForward(-9,DRIVE_SPEED);
        rotate(30,DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1);
        intakeTransfer.setPower(1);
        sleep(2500);
        stopShootSequence();


        moveForward(-(10+HALF_OF_BOT_LENGTH),DRIVE_SPEED);
        sleep(100);
        rotate(-115,DRIVE_SPEED);
        intakeTransfer.setPower(0.8);
        intakeServo.setPower(-0.8);
        moveForward(43,0.35);
        sleep(100);
        intakeTransfer.setPower(0);
        intakeServo.setPower(0);
        moveForward(-43,DRIVE_SPEED);
        rotate(115,DRIVE_SPEED);
        sleep(100);
        farShot();
        moveForward((10+HALF_OF_BOT_LENGTH),DRIVE_SPEED);

        intakeServo.setPower(1);
        intakeTransfer.setPower(1);
        sleep(2500);

        moveForward(-10,DRIVE_SPEED);
        // End
        stopAll();
    }
}
