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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName; // Added for Webcam
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal; // Added for Vision
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection; // Added for AprilTag
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor; // Added for AprilTag

import java.util.List; // Added for List

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
    DcMotor intake;
    DcMotor outtake;
    float intakePower = 0;
    IMU imu;

    // --- AprilTag Vision Declarations (from first code block) ---
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialization ---
        initializeHardware();
        initializeVision();

        telemetry.addLine("Initialization Complete: Press Play!");
        telemetry.update();

        waitForStart();

        // Ensure the vision portal is closed when the OpMode ends
        try {
            // --- Main TeleOp Loop (Replaces the OpMode.loop() method) ---
            while (opModeIsActive()) {

                // 1. Handle Drive Controls
                handleDrive();

                // 2. Handle Mechanism Controls
                handleMechanisms();

                // 3. Update AprilTag Telemetry
                telemetryAprilTag();

                // Push all new data to the driver station
                telemetry.update();
            }
        } finally {
            if (visionPortal != null) {
                visionPortal.close();
            }
        }
    }

    /**
     * Initializes all motors and the IMU.
     */
    private void initializeHardware() {
        // Motor Hardware Map
        shooter_1 = hardwareMap.get(DcMotor.class, "shooter_1");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        // Motor Directions
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);
        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor Modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    /**
     * Initializes the AprilTag Vision Processor (from first code block).
     */
    private void initializeVision() {
        // Create the AprilTag processor (from first code block)
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        // Create the Vision Portal (from first code block)
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
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

    /**
     * Handles all mechanism-related gamepad input.
     */
    private void handleMechanisms() {
        telemetry.addLine("\n--- Mechanism Controls ---");
        telemetry.addLine("Press 'B' for Shooter (0.8 power)");
        telemetry.addLine("Press 'X' to toggle Intake");
        telemetry.addLine("Press 'Y' for Outtake (1.0 power)");

        // Shooter controls (B)
        if (gamepad1.b) {
            shooter_1.setPower(0.8);
        } else {
            shooter_1.setPower(0);
        }

        // Intake controls (X) - Original Toggle Logic kept, but may misfire in a fast loop.
        if (gamepad1.x) {
            // This is the original *toggle* logic:
            if (intakePower == 0) {
                intake.setPower(0.8);
                intakePower = 0.8F;
            } else {
                intake.setPower(0);
                intakePower = 0;
            }
            // To prevent multiple toggles per single press, a delay or debounce logic is usually needed.
            /*sleep(250);*/ // if needed uncomment this to have a bit of wait time before intake// Small sleep to help debounce the toggle
        }

        // Outtake controls (Y)
        if (gamepad1.y) {
            outtake.setPower(1);
        } else {
            outtake.setPower(0);
        }
    }


    // This routine drives the robot field relative
    private void driveFieldRelative(double forward, double right, double rotate) {
        // First, convert direction being asked to drive to polar coordinates
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        // Second, rotate angle by the angle the robot is pointing
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));

        // Third, convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        // Finally, call the drive method with robot relative forward and right amounts
        drive(newForward, newRight, rotate);
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


    /**
     * Updates telemetry with AprilTag detection data (from first code block).
     */
    private void telemetryAprilTag() {
        telemetry.addLine("\n--- AprilTag Vision ---");

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f    (pixels)", detection.center.x, detection.center.y));
            }
        }

        // Add "key" information to telemetry
        telemetry.addLine("\nKey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}