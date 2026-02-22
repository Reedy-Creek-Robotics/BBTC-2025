package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;

@Autonomous(name = "Auto : Blue Straight No Shoot")
public class AutonomusStraightBlueNoShoot extends BaseAutonomus {


    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("Ready!");
        telemetry.update();
        camera.setPipelineBlue();
        camera.update();
        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS STEPS
        // -------------------------------
        farShot();
        moveForward(-6,0.7);
        sleep(100);
        turnCorrectionBlue();
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1.0);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();
        camera.setPipelineUseless();
        camera.update();

        rotate(20,DRIVE_SPEED);

        strafe(20,0.9,StrafeDirection.RIGHT);
        stopAll();
    }
}
