package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Test: REV 2m Distance Sensor", group = "Test")
public class rana extends LinearOpMode {

    // Declare the sensor object using the general DistanceSensor interface
    private DistanceSensor distanceSensor;

    @Override
    public void runOpMode() {
        // Retrieve the sensor from the hardware map.
        // "distanceSensor" must match the device name set in your Driver Station configuration.
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        telemetry.addData("Status", "Initialized. Press Play to start reading data.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Query distance in various measurement units
            double distInches = distanceSensor.getDistance(DistanceUnit.INCH);
            double distCm     = distanceSensor.getDistance(DistanceUnit.CM);
            double distMm     = distanceSensor.getDistance(DistanceUnit.MM);

            // Display distance readings on the Driver Station screen
            telemetry.addData("Status", "Running");
            telemetry.addData("Distance (in)", "%.2f in", distInches);
            telemetry.addData("Distance (cm)", "%.2f cm", distCm);
            telemetry.addData("Distance (mm)", "%.2f mm", distMm);

            telemetry.update();
        }
    }
}
