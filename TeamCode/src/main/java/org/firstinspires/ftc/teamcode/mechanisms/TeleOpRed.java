package org.firstinspires.ftc.teamcode.mechanisms;


import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp Red")
public class TeleOpRed extends BaseTeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware(); // Calling the function from the Brain
        telemetry.update();
        camera.setPipelineRed();
        camera.update();

        waitForStart();

        lastLedToggleTime = System.currentTimeMillis();
        while (opModeIsActive()) {
            handleDrive(gamepad1.left_stick_y, -gamepad1.left_stick_x);
            handleMechanisms();
            camera.update();
            currentTime = System.currentTimeMillis();
            // Capture the status once to avoid calculating it multiple times
            boolean isTargetLocked = redFarShotLED();
            if(killLED == true){
                led.setPosition(0);
            } else{
                if (isTargetLocked) {
                    // Force the color here to be sure it stays lit while locked
                    led.setPosition(0.666);
                    // Optional: Reset the blink timer so the pattern starts fresh when lock is lost
                    lastLedToggleTime = currentTime;
                } else {
                    long elapsed = currentTime - lastLedToggleTime;

                    if (elapsed >= 5000) {
                        lastLedToggleTime = currentTime;
                    } else if (elapsed >= 2000) {
                        led.setPosition(0);
                    } else {
                        // Only increment color at the very start of the 2-second window
                        if (elapsed < 30) {
                            led_color += 0.1;
                            if (led_color > 1.0) led_color = 0.1;
                            if (led_color > 0.55 && led_color < 0.69) led_color += 0.2;
                        }
                        led.setPosition(led_color);
                    }
                }

            }

            telemetry.update();
            telemetry.addData("rotation Tx: ", -(camera.getTx()));
            telemetry.addData("dist: ", camera.getDistance());
        }
    }
}