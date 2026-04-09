package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;
//RIGHT ONE

@Autonomous(name = "Auto : Blue Straight Shoot")
public class AutonomusStraightBlueShoot extends BaseAutonomus {

    // Drive motors

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("Ready!");
        telemetry.update();
        camera.setPipelineBlue();
        camera.update();


        waitForStart();

        if (!opModeIsActive()) return;
        camera.update();



        // -------------------------------
        // AUTONOMOUS STEPS

        // -------------------------------
        farShot();
        moveForward(-9,DRIVE_SPEED);
        sleep(100);
        rotate(-25,DRIVE_SPEED);
        sleep(100);
        rotation = turnCorrectionBlue();
        sleep(100);
        intakeServo.setPower(1);
        intakeTransfer.setPower(1);
        sleep(2500);
        stopShootSequence();


        moveForward(-(11+HALF_OF_BOT_LENGTH),DRIVE_SPEED);
        sleep(100);
        rotate(125+rotation,DRIVE_SPEED);
        intakeTransfer.setPower(1);
        intakeServo.setPower(-1);
        moveForward(45,0.25);
        intakeOnUntilDetected();
        moveForward(-45,DRIVE_SPEED);
        rotate(-125,DRIVE_SPEED);
        sleep(100);
        farShot();
        moveForward((12+HALF_OF_BOT_LENGTH),DRIVE_SPEED);
        turnCorrectionBlue();
        intakeServo.setPower(1);
        intakeTransfer.setPower(0.5);
        sleep(2500);
        stopShootSequence();
        camera.setPipelineUseless();
        camera.update();
        moveForward(-18,DRIVE_SPEED);

        // End
        stopAll();
    }
}
