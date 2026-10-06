package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Motor Stop on Magnet", group = "Test")

public class rana2001 extends LinearOpMode {

    private DcMotor testMotor;
    private DigitalChannel magnetLimit;

    @Override
    public void runOpMode() {
        // 1. Initialize hardware (make sure names match your Robot Configuration)
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");
        magnetLimit = hardwareMap.get(DigitalChannel.class, "magnetLimit");

        // 2. Set sensor to INPUT mode
        magnetLimit.setMode(DigitalChannel.Mode.INPUT);

        // Optional: set motor behavior when power is 0 (Brake helps it stop instantly)
        testMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Initialized. Press PLAY to start turning.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // REV magnetic switches are active-LOW: getState() is false when magnet is near.
            // Using '!' makes 'isMagnetPresent' true when the magnet is detected.
            boolean isMagnetPresent = !magnetLimit.getState();

            if (isMagnetPresent) {
                // STOP the motor immediately if the magnet is detected
                testMotor.setPower(0);
                telemetry.addData("Status", "MAGNET DETECTED! Motor Stopped.");
            } else {
                // Keep turning (e.g., at 30% power) if no magnet is near
                testMotor.setPower(0.3);
                telemetry.addData("Status", "Turning... Searching for magnet.");
            }

            // Display raw telemetry for debugging
            telemetry.addData("Raw Sensor State", magnetLimit.getState());
            telemetry.update();
        }
    }
}