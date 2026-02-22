package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;

@Autonomous(name = "Auto : Blue Goal Shoot")
public class AutonomusBlueGoal extends BaseAutonomus {

    // Drive motors

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
        moveForward(56,0.7);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1.0);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(142, DRIVE_SPEED);
        sleep(100);
        intakeTransfer.setPower(1);
        intakeServo.setPower(-1);

        moveForward((43),0.25);
        intakeOnUntilDetected();
        moveForward((-39), DRIVE_SPEED);

        closeShot();
        rotate(-140, DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1);
        intakeTransfer.setPower(1);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(140,DRIVE_SPEED);
        strafe((17+HALF_OF_BOT_LENGTH),0.9,StrafeDirection.LEFT);
        sleep(100);

        intakeTransfer.setPower(1);
        intakeServo.setPower(-1);
        moveForward(46,0.25);
        intakeOnUntilDetected();


        moveForward(-46,DRIVE_SPEED);
        closeShot();
        strafe((17+HALF_OF_BOT_LENGTH + 24),DRIVE_SPEED,StrafeDirection.RIGHT);
        rotate(-155,DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1);
        sleep(2500);
        // End
        stopAll();
    }
}
