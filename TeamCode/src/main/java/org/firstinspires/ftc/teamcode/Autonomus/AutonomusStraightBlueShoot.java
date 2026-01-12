package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//RIGHT ONE

@Autonomous(name = "Autonomous : Blue straight Shoot")
public class AutonomusStraightBlueShoot extends BaseAutonomus {

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

        moveForward((72+HALF_OF_BOT_LENGTH),DRIVE_SPEED);
        rotate(147,TURN_SPEED);
        moveForward(-8,0.6);
        shooter_1.setVelocity(2300);
        sleep(2500);
        intakeServo.setPower(1);
        sleep(500);
        intakeTransfer.setPower(1);
        sleep(5000);
        stopShootSequence();
        sleep(100);

        moveForward(8, 0.6);
        rotate(147, TURN_SPEED);
        sleep(100);
        intakeTransfer.setPower(1.0);

        moveForward((44),0.23);
        sleep(100);
        intakeTransfer.setPower(0.0);

        moveForward((-44), DRIVE_SPEED);
        rotate(-147, TURN_SPEED);
        moveForward(-8,0.6);
        shooter_1.setVelocity(2300);
        sleep(2500);
        intakeServo.setPower(1.0);
        sleep(2000);
        intakeTransfer.setPower(0.8);
        sleep(5000);

        rotate(90,TURN_SPEED);
        moveForward(10,DRIVE_SPEED);
        
        // End
        stopAll();
    }
}
