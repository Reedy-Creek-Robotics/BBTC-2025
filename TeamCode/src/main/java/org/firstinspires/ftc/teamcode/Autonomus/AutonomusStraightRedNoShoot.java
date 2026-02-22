package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;

@Autonomous(name = "Auto : Red Straight No Shoot")
public class AutonomusStraightRedNoShoot extends BaseAutonomus {

    // Drive motors

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("Ready!");
        telemetry.update();
        camera.setPipelineRed();
        camera.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS STEPS
        // -------------------------------
        farShot();
        moveForward(-6,0.7);
        sleep(100);
        turnCorrectionRed();
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1.0);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();
        camera.setPipelineUseless();
        camera.update();

        rotate(-20,DRIVE_SPEED);
        strafe(20,0.9,StrafeDirection.LEFT);
        stopAll();
    }
}
