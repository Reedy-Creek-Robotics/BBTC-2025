package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import android.graphics.Color;

@TeleOp(name = "REV Color Sensor V3 Example", group = "Sensors")
public class color extends LinearOpMode {

    private RevColorSensorV3 colorSensor;

    @Override
    public void runOpMode() {
        // Get the color sensor from hardware map
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        waitForStart();

        while (opModeIsActive()) {

            // Read RGB values
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Convert RGB to HSV
            float[] hsv = new float[3];
            Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv);  // multiply by 8 to scale 0-255 properly

            String colorDetected = "Unknown";

            // Simple color detection by hue value
            if (hsv[0] >= 0 && hsv[0] <= 10 || hsv[0] >= 350 && hsv[0] <= 360) {
                colorDetected = "Red";
            } else if (hsv[0] >= 180 && hsv[0] <= 250) {
                colorDetected = "Blue";
            }

            // Telemetry
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.addData("Hue", hsv[0]);
            telemetry.addData("Detected Color", colorDetected);
            telemetry.update();

            sleep(100);
        }
    }
}
