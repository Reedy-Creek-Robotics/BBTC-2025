package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Elevator Position Test")
public class ElevatorTesting extends LinearOpMode {

    private DcMotor Elevator;

    // --- CONFIGURATION CONSTANTS ---
    // Change these to match your actual mechanical build!
    static final double COUNTS_PER_MOTOR_REV = 537.7; // e.g., GoBILDA 5203 series 312 RPM
    static final double SPOOL_DIAMETER_INCHES = 1.5; // Diameter of the spool winding the string
    static final double COUNTS_PER_INCH = COUNTS_PER_MOTOR_REV / (SPOOL_DIAMETER_INCHES * Math.PI);

    // Variable to track the current target height in inches
    double targetInches = 0.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        Elevator = hardwareMap.get(DcMotor.class, "Elevator");
        Elevator.setDirection(DcMotor.Direction.FORWARD);
        Elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset encoders and prepare for run-to-position mode
        Elevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Variables for button edge detection (Y and A)
        boolean lastYState = false;
        boolean lastAState = false;

        telemetry.addData("Status", "Initialized. Reset encoders complete.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. Handle Y (Up 20 in) and A (Down 20 in) Button Presses ---
            boolean currentYState = gamepad1.y;
            boolean currentAState = gamepad1.a;

            // If Y is freshly pressed, add 20 inches
            if (currentYState && !lastYState) {
                targetInches += 20.0;
                // Optional: You can add a maximum height cap here if needed (e.g., max 40 inches)
            }

            // If A is freshly pressed, subtract 20 inches (won't go below 0)
            if (currentAState && !lastAState) {
                targetInches = Math.max(0.0, targetInches - 20.0);
            }

            lastYState = currentYState;
            lastAState = currentAState;

            // --- 2. Calculate Target Ticks and Command the Motor ---
            int targetTicks = (int) (targetInches * COUNTS_PER_INCH);

            // Set target position and switch motor mode
            Elevator.setTargetPosition(targetTicks);
            Elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // Set power to drive towards the target
            Elevator.setPower(0.8);

            // --- 3. Telemetry ---
            telemetry.addData("Target Height (Inches)", targetInches);
            telemetry.addData("Target Ticks", targetTicks);
            telemetry.addData("Current Encoder Ticks", Elevator.getCurrentPosition());
            telemetry.addData("Motor Busy", Elevator.isBusy());
            telemetry.update();
        }
    }
}