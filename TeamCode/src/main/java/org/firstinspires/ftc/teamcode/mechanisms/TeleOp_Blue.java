package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp: Blue")
public class TeleOp_Blue extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware(); // Calling the function from the Brain
        telemetry.update();
        blueChannel();
        waitForStart();
        cameraStart();

        while (opModeIsActive()) {
            handleDrive(-gamepad1.left_stick_y, gamepad1.left_stick_x);
            handleMechanisms();
            telemetry.update();
            camera.update();
        }
    }
}
