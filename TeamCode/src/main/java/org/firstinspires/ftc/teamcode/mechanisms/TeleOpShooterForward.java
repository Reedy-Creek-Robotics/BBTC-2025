package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp: Shooter Forward")
public class TeleOpShooterForward extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware(); // Calling the function from the Brain
        telemetry.update();
        waitForStart();
        intakeServo.setPosition(0.2);

        while (opModeIsActive()) {
            handleDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            handleMechanisms();
            telemetry.update();
        }
    }
}
