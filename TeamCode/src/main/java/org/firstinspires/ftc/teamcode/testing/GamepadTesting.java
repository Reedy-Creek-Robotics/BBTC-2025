package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Gamepad Test")
public class GamepadTesting extends LinearOpMode {
    @Override
    public void runOpMode() {
        waitForStart();

        // Keep running as long as the OpMode is active
        while (opModeIsActive()) {
            // Check if 'B' was pressed (using bWasPressed is great for single-press events)
            if (gamepad1.bWasPressed()) {
                gamepad1.rumble(2000); // Rumble for 2 seconds
            }

            // Allow the phone/Control Hub to process background tasks
            telemetry.update();
        }
    }
}