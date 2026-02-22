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
        camera.setPipelineUseless();
        camera.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS STEPS
        // -------------------------------

        closeShot();
        moveForward(57,0.7);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1.0);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(-142, DRIVE_SPEED);
        sleep(100);
        intakeTransfer.setPower(1);
        intakeServo.setPower(-1);//-0.8

        moveForward((44),0.25);//0.35
        intakeOnUntilDetected();
        moveForward((-42), DRIVE_SPEED);

        closeShot();
        rotate(142, DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(-145,DRIVE_SPEED);
        strafe((19+HALF_OF_BOT_LENGTH),DRIVE_SPEED,StrafeDirection.RIGHT);
        sleep(100);
        intakeTransfer.setPower(1);
        intakeServo.setPower(-1);//-0.8
        moveForward(47,0.25);//45, 0.35
        intakeOnUntilDetected();



        moveForward(-47,DRIVE_SPEED);
        closeShot();
        strafe((19+HALF_OF_BOT_LENGTH+24),DRIVE_SPEED,StrafeDirection.LEFT);
        rotate(155,DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1);
        sleep(2500);

        // End
        stopAll();
    }
}
