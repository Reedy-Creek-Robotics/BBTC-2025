package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Testing", group = "Test")
public class servo_testing extends LinearOpMode {

    private Servo testServo;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize servo
        testServo = hardwareMap.get(Servo.class, "testServo");

        telemetry.addLine("Servo initialized. Waiting for start...");
        telemetry.update();

        waitForStart();

        // Set servo position to 1
        testServo.setPosition(1.0);

        telemetry.addLine("Servo set to position 1.0");
        telemetry.update();

        // Keep the opmode alive so we can see the servo
        while (opModeIsActive()) {
            idle();
        }
    }
}
