package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "Magnetic Switch Test", group = "Test")
public class rana2000 extends LinearOpMode {

    private DigitalChannel magnetLimit;

    @Override
    public void runOpMode() {
        // Initialize the sensor from the hardware map.
        // Make sure "magnetLimit" matches the exact name you used in your Robot Configuration!
        magnetLimit = hardwareMap.get(DigitalChannel.class, "magnetLimit");

        // Set the digital channel mode to INPUT so it can read the sensor state
        magnetLimit.setMode(DigitalChannel.Mode.INPUT);

        telemetry.addLine("Status: Initialized. Bring a magnet close to the sensor!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // REV magnetic limit switches are active-LOW:
            // - When a magnet is present, getState() returns false.
            // - When no magnet is present, getState() returns true.
            // We use the '!' (NOT) operator so 'isTriggered' is true when the magnet is near.
            boolean isTriggered = !magnetLimit.getState();

            // Send telemetry data to the Driver Station screen
            telemetry.addData("Raw Digital State", magnetLimit.getState());
            telemetry.addData("Magnet Detected", isTriggered ? "YES" : "NO");
            telemetry.update();
        }
    }
}