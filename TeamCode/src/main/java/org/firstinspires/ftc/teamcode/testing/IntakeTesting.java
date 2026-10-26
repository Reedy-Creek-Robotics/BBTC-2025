package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Intake Test")
public class IntakeTesting extends LinearOpMode {

    private DcMotor IntakeMotor;

    @Override
    public void runOpMode() {
        // Initialize hardware
        IntakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        IntakeMotor.setDirection(DcMotor.Direction.FORWARD);

        // State variables
        boolean intakeOn = false;
        double intakePower = 0.0; // Default starting power

        // Variables for button edge detection
        boolean lastBState = false;
        boolean lastDpadUpState = false;
        boolean lastDpadDownState = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // --- 1. Toggle Flywheel with Button B ---
            boolean currentBState = gamepad1.b;
            if (currentBState && !lastBState) {
                intakeOn = !intakeOn; // Flip the state (true -> false, or false -> true)
            }
            lastBState = currentBState;

            // --- 2. Stepper for Power using DPad Up/Down ---
            boolean currentDpadUp = gamepad1.dpad_up;
            boolean currentDpadDown = gamepad1.dpad_down;

            if (currentDpadUp && !lastDpadUpState) {
                intakePower = Math.min(1.0, intakePower + 0.1); // Increase by 5%, max 1.0
            }
            if (currentDpadDown && !lastDpadDownState) {
                intakePower = Math.max(0.0, intakePower - 0.2); // Decrease by 5%, min 0.0
            }

            lastDpadUpState = currentDpadUp;
            lastDpadDownState = currentDpadDown;

            // --- 3. Apply Power to Motor ---
            if (intakeOn) {
                IntakeMotor.setPower(intakePower);
            } else {
                IntakeMotor.setPower(0.0);
                telemetry.addLine("Press button B to start/stop shooter!");
            }

            // --- 4. Telemetry (to see what's happening on the Driver Station) ---
            telemetry.addData("Flywheel State", intakeOn ? "ON" : "OFF");
            telemetry.addData("Target Power", intakePower);
            telemetry.update();
        }
    }
}