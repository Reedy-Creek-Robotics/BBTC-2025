package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor Test", group = "Test")
public class sensortest extends LinearOpMode {

    private DistanceSensor distance1;
    private DistanceSensor distance2;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {

        // Match these names to your configuration in the FTC Robot Controller
        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");
        distance2 = hardwareMap.get(DistanceSensor.class, "distance2");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        waitForStart();

        while (opModeIsActive()) {

            // Distance readings (in inches)
            telemetry.addData("Distance 1 (in)", distance1.getDistance(DistanceUnit.INCH));
            telemetry.addData("Distance 2 (in)", distance2.getDistance(DistanceUnit.INCH));

            // Color readings
            telemetry.addData("Red", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue", colorSensor.blue());

            telemetry.update();
        }
    }
}
