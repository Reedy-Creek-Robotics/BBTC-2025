package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "TeleOp Testing", group = "Linear OpMode")
public class TeleopTesting extends LinearOpMode {

    boolean intakeOn = false;
    boolean reverseOn = false;
    double intakePower = 0.0; // Default starting power

    // Variables for button edge detection
    boolean lastBState = false;
    boolean lastXState = false;
    boolean lastDpadUpState = false;
    boolean lastDpadDownState = false;

    @Override
    public void runOpMode() {
        // 1. Declare motors
        DcMotor leftFront  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        DcMotor leftRear   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        DcMotor rightFront = hardwareMap.get(DcMotor.class, "frontRightMotor");
        DcMotor rightRear  = hardwareMap.get(DcMotor.class, "backRightMotor");
        DcMotor IntakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        // 2. Set motor directions
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        // 3. Set motor zero power behavior to BRAKE
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Gamepad stick inputs (Y is inverted on gamepads, so negate it)
            double y  = -gamepad1.left_stick_y; // Forward/Back
            double x  =  gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing (FIXED)
            double rx =  gamepad1.right_stick_x; // Rotation

            // Calculate wheel power components
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower  = (y + x + rx) / denominator;
            double backLeftPower   = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower  = (y + x + rx) / denominator;

            // Set power to motors
            leftFront.setPower(frontLeftPower);
            leftRear.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightRear.setPower(backRightPower);

            // --- 1. Toggle Intake Forward (Button B) ---
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState) {
                intakeOn = !intakeOn;
                if (intakeOn) {
                    reverseOn = false; // Turn off reverse if forward is turned on
                }
            }
            lastBState = currentBState;

            // --- Toggle Intake Reverse (Button X) ---
            boolean currentXState = gamepad1.x;
            if (currentXState && !lastXState) {
                reverseOn = !reverseOn;
                if (reverseOn) {
                    intakeOn = false; // Turn off forward if reverse is turned on
                }
            }
            lastXState = currentXState; // (FIXED: added missing state update)

            // --- 2. Stepper for Power using DPad Up/Down ---
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentDpadUp && !lastDpadUpState) {
                intakePower = Math.min(1.0, intakePower + 0.1); // Increase power, max 1.0
            }
            if (currentDpadDown && !lastDpadDownState) {
                intakePower = Math.max(0.0, intakePower - 0.1); // Decrease power, min 0.0 (FIXED)
            }

            lastDpadUpState = currentDpadUp;
            lastDpadDownState = currentDpadDown;

            // --- 3. Apply Power to Motor (Combined cleanly) ---
            if (intakeOn) {
                IntakeMotor.setPower(intakePower);
            } else if (reverseOn) {
                IntakeMotor.setPower(-intakePower);
            } else {
                IntakeMotor.setPower(0.0);
            }

            // Send telemetry data to the Driver Station
            String intakeStatus = "OFF";
            if (intakeOn) intakeStatus = "FORWARD (Power: " + String.format("%.2f", intakePower) + ")";
            else if (reverseOn) intakeStatus = "REVERSE (Power: " + String.format("%.2f", intakePower) + ")";

            telemetry.addData("Intake Status", intakeStatus);
            telemetry.addData("Motors", "LF (%.2f), RF (%.2f), LR (%.2f), RR (%.2f)",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}