package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//RIGHT ONE

@Autonomous(name = "Auto : Blue Straight Shoot")
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
        shooter_1.setVelocity(SHOOTER_TPS);
        rotate(147,TURN_SPEED);
        sleep(1000);
        intakeServo.setPower(1);
        sleep(500);
        intakeTransfer.setPower(1);
        sleep(4500);
        stopShootSequence();
        sleep(100);

        moveForward(8, DRIVE_SPEED);
        rotate(147, TURN_SPEED);
        sleep(100);
        intakeTransfer.setPower(1.0);

        moveForward((44),0.23);
        sleep(100);
        intakeTransfer.setPower(0.0);

        moveForward((-44), DRIVE_SPEED);
        rotate(-147, TURN_SPEED);
        shooter_1.setVelocity(SHOOTER_TPS);
        moveForward(-12, DRIVE_SPEED);
        sleep(100);
        intakeServo.setPower(1.0);
        sleep(500);
        intakeTransfer.setPower(0.8);
        sleep(4500);

        rotate(90,TURN_SPEED);
        moveForward(10,DRIVE_SPEED);
        
        // End
        stopAll();
    }
}
