package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.mechanisms.BaseAutonomus;

@Autonomous(name = "Auto : Blue Goal No Shoot")
public class AutonomusBlueGoalNoShoot extends BaseAutonomus {


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
        veryCloseShot();
        moveForward(35,0.4);
        sleep(100);
        intakeServo.setPower(1.0);
        intakeTransfer.setPower(1.0);
        sleep(2500);
        shooter_1.setPower(0);
        stopShootSequence();

        rotate(20, DRIVE_SPEED);
        moveForward((-10),0.4);
        // End
        stopAll();
    }
}
