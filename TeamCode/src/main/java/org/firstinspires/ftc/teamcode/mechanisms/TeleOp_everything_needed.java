/* Copyright (c) 2025 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode; // Changed to LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/**
 * This TeleOp OpMode combines Field-Relative Mecanum Drive, Mechanism Controls,
 * and Live AprilTag Vision Telemetry using a LinearOpMode structure.
 */
@TeleOp(name = "TeleOp: Mecanum+Vision", group = "Robot")

public class TeleOp_everything_needed extends LinearOpMode {
    // --- Drive & Mechanism Declarations ---
    DcMotor flmotor;
    DcMotor frmotor;
    DcMotor blmotor;
    DcMotor brmotor;
    DcMotor shooter_1;
    DcMotor shooter_2;
    DcMotor intake;
    float intakePower = 0;

    float shooterPower = 0;
    IMU imu;

    // --- State & Constants for AprilTag Alignment ---
    // These constants control the auto-align behavior.
    // P-controller gain. Determines how fast the robot will turn.
    final double ROTATION_P_GAIN = 0.02;
    // Smallest error in degrees before the robot stops aligning.
    final double ROTATION_ERROR_TOLERANCE = 1.0;
    // Maximum rotation speed to prevent overshoot.
    final double MAX_ROTATION_SPEED = 0.4;

    // --- State variables for toggles ---
    private boolean intakeOn = false;      // Tracks the state of the intake motor
    private boolean xWasPressed = false;   // Tracks the previous state of the 'X' button to detect a single press
    private boolean shooterOn = false;     // Tracks the state of the shooter motors
    private boolean bWasPressed = false;   // Tracks the previous state of the 'B' button


    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        initializeHardware();

        telemetry.addLine("Initialization Complete: Press Play!");
        telemetry.update();

        waitForStart();

        // --- Main TeleOp Loop (Replaces the OpMode.loop() method) ---
        while (opModeIsActive()) {

            // 1. Handle Drive Controls
            handleDrive();

            // 2. Handle Mechanism Controls
            handleMechanisms();

            // Push all new data to the driver station
            telemetry.update();
        }
    }


    /**
     * Initializes all motors and the IMU.
     */
    private void initializeHardware() {
        // Motor Hardware Map
        shooter_2 = hardwareMap.get(DcMotor.class, "shooter_2");
        shooter_1 = hardwareMap.get(DcMotor.class, "shooter_1");
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Motor Directions
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);
        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor Modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Handles all drive-related gamepad input.
     */
    private void handleDrive() {
        telemetry.addLine("\n--- Drive Controls ---");
        telemetry.addLine("Press A to reset Yaw");
        telemetry.addLine("Hold left bumper for robot-relative drive");

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Reset Yaw
        if (gamepad1.a) {
            imu.resetYaw();
        }

        // Drive Mode Selection
        if (gamepad1.left_bumper) {
            drive(forward, right, rotate); // Robot Relative
        } else {
            driveFieldRelative(forward, right, rotate); // Field Relative
        }
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
    }

    /**
     * Handles all mechanism-related gamepad input.
     */
    private void handleMechanisms() {
        // telemetry.addLine("\n--- Mechanism Controls ---");
        // telemetry.addLine("Press 'B' for Shooter (0.8 power)");
        // telemetry.addLine("Press 'X' to toggle Intake");
        // telemetry.addLine("Press right bumper to align to AprilTag");

        // Intake toggle controls (X)
        // Check if the 'X' button is currently pressed and wasn't pressed on the previous loop iteration.
        // This makes it a "rising edge" detector, so the code inside only runs once per button press.
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn; // Flip the state of the intake
        }
        // Update the button state for the next loop iteration.
        xWasPressed = gamepad1.x;

        if (intakeOn) {
            intake.setPower(1.0); // Set to full power when on
        } else {
            intake.setPower(0);
        }

        // Shooter toggle controls (B)
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn; // Flip the state of the shooter
        }
        bWasPressed = gamepad1.b;

        if (shooterOn) {
            shooter_1.setPower(1.0); // Run shooter forward
            shooter_2.setPower(1.0); // Run shooter forward
        } else {
            shooter_1.setPower(0); // Stop shooter
            shooter_2.setPower(0); // Stop shooter
        }
    }

    // Thanks to FTC16072 for sharing this code!!
    public void drive(double forward, double right, double rotate) {
        // This calculates the power needed for each wheel based on the amount of forward,
        // strafe right, and rotate
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double maxPower = 1.0;
        double maxSpeed = 1.0;

        // This is needed to make sure we don't pass > 1.0 to any wheel
        maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
        maxPower = Math.max(maxPower, Math.abs(frontRightPower));
        maxPower = Math.max(maxPower, Math.abs(backRightPower));
        maxPower = Math.max(maxPower, Math.abs(backLeftPower));

        // We multiply by maxSpeed so that it can be set lower for outreaches
        flmotor.setPower(maxSpeed * (frontLeftPower / maxPower));
        frmotor.setPower(maxSpeed * (frontRightPower / maxPower));
        blmotor.setPower(maxSpeed * (backLeftPower / maxPower));
        brmotor.setPower(maxSpeed * (backRightPower / maxPower));
    }

}