package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;
//RIGHT ONE

@Autonomous(name = "Auto : Red Goal Shoot")
public class AutonomusRedGoal extends BaseAutonomus {


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

        closeShot();
        moveForward(56,0.7);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1.0);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(-140, DRIVE_SPEED);
        sleep(100);
        intakeTransfer.setPower(1.0);
        intakeServo.setPower(-0.8);

        moveForward((44),0.35);
        sleep(100);
        intakeTransfer.setPower(0.0);
        intakeServo.setPower(0);
        moveForward((-40), DRIVE_SPEED);

        closeShot();
        rotate(140, DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(-52,DRIVE_SPEED);
        moveForward((18+HALF_OF_BOT_LENGTH),DRIVE_SPEED);
        rotate(-92,DRIVE_SPEED);
        sleep(100);
        intakeTransfer.setPower(0.7);
        intakeServo.setPower(-0.8);
        moveForward(45,0.35);
        sleep(100);
        intakeTransfer.setPower(0);
        intakeServo.setPower(0);


        moveForward(-50,DRIVE_SPEED);
        rotate(92,DRIVE_SPEED);
        closeShot();
        moveForward(-(17+HALF_OF_BOT_LENGTH),DRIVE_SPEED);
        rotate(55,DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1);
        sleep(2500);

        // End
        stopAll();
    }
}
