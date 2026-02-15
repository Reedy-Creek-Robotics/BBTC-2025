package org.firstinspires.ftc.teamcode.mechanisms;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware(); // Calling the function from the Brain
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            handleDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            handleMechanisms();
            telemetry.update();
            telemetry.addData("dist: ",camera.getDistance());
            telemetry.addData("area: ",camera.getArea());
        }
    }
}
