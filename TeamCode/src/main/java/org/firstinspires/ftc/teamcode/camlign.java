package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "godbole3000_fixed_2cam_ContainerID", group = "Practice")
public class camlign extends OpMode {

    // --- Configuration ---
    private static final int TARGET_TAG_ID = 10;
    private final double kP_DRIVE = 0.05;
    private final double kP_STRAFE = 0.05;
    private final double kP_TURN = 0.03;
    private final double MAX_ALIGN_SPEED = 0.5;

    // --- Motors and IMU ---
    private DcMotor fldrive, frdrive, bldrive, brdrive;
    private IMU imu;
    private double yawOffset = 0;

    // --- Vision and State Tracking ---
    private AprilTagProcessor aprilTagProcessor1;
    private AprilTagProcessor aprilTagProcessor2;
    private VisionPortal visionPortal1;
    private VisionPortal visionPortal2;

    private boolean isAligning = false;
    private boolean bPressedLastLoop = false;
    private boolean isCam1Active = true;     // Tracks which camera's processor is ENABLED

    // --- Initialization ---
    @Override
    public void init() {
        // Drive Motor Setup
        fldrive = hardwareMap.get(DcMotor.class, "flmotor");
        frdrive = hardwareMap.get(DcMotor.class, "frmotor");
        bldrive = hardwareMap.get(DcMotor.class, "blmotor");
        brdrive = hardwareMap.get(DcMotor.class, "brmotor");

        bldrive.setDirection(DcMotor.Direction.REVERSE);
        fldrive.setDirection(DcMotor.Direction.REVERSE);

        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU Setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Vision Setup: Initialize Two Processors and Two Portals

        // 1. Create TWO separate processors
        aprilTagProcessor1 = AprilTagProcessor.easyCreateWithDefaults();
        aprilTagProcessor2 = AprilTagProcessor.easyCreateWithDefaults();

        // **IMPORTANT:** Use 0 as the container ID placeholder for the standard Driver Station view.
        int LIVE_VIEW_CONTAINER_ID = 0;

        // 2. Attach Processor 1 to Portal 1 using setLiveViewContainerId
        visionPortal1 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor1)
                .setLiveViewContainerId(LIVE_VIEW_CONTAINER_ID)
                .build();

        // 3. Attach Processor 2 to Portal 2 using setLiveViewContainerId
        visionPortal2 = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(aprilTagProcessor2)
                .setLiveViewContainerId(LIVE_VIEW_CONTAINER_ID)
                .build();

        // 4. Start with Webcam 1 active, Webcam 2 disabled
        visionPortal2.setProcessorEnabled(aprilTagProcessor2, false);
        isCam1Active = true;

        telemetry.addData("Status", "Initialization Complete. Press Start.");
        telemetry.addData("Camera Status", "Webcam 1 Active");
        telemetry.update();
    }

// ----------------------------------------------------------------------------------

    // --- Main Loop ---
    @Override
    public void loop() {
        // 1. Toggle Alignment Mode with B button
        boolean bPressedNow = gamepad1.b;
        if (bPressedNow && !bPressedLastLoop) {
            isAligning = !isAligning; // Toggle the state
        }
        bPressedLastLoop = bPressedNow;

        // 2. Core Control Logic: Alignment or Manual Drive
        if (isAligning) {
            alignToTag();
            telemetry.addData("Alignment Status", "✅ ACTIVE (Target ID %d)", TARGET_TAG_ID);
            telemetry.addData("Press B", "To Disable Alignment");
        } else {
            // Manual TeleOp Drive
            manualDrive();
            telemetry.addData("Alignment Status", "❌ INACTIVE (Press B to enable)");
        }

        // 3. Telemetry Update
        telemetryAprilTag();
        telemetry.update();
    }

// ----------------------------------------------------------------------------------

    // --- Alignment Logic ---

    private void alignToTag() {
        AprilTagDetection targetDetection = getTargetDetection();

        if (targetDetection != null) {
            // Get the Pose data from the detection
            double x_error = targetDetection.ftcPose.x;        // Strafe error (target: 0)
            double z_error = targetDetection.ftcPose.z;        // Drive error (target: 0)
            double bearing_error = targetDetection.ftcPose.bearing; // Turn error (target: 0)

            // P-Controller to determine drive powers
            double drivePower = -z_error * kP_DRIVE;
            double strafePower = -x_error * kP_STRAFE;
            double turnPower = bearing_error * kP_TURN;

            // Clip power
            drivePower = Range.clip(drivePower, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);
            strafePower = Range.clip(strafePower, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);
            turnPower = Range.clip(turnPower, -MAX_ALIGN_SPEED, MAX_ALIGN_SPEED);

            // Apply the calculated powers
            drive(drivePower, strafePower, turnPower);

            // Telemetry
            telemetry.addData("Tag Found (Cam %s)", isCam1Active ? "1" : "2");
            telemetry.addData("Error X/Z/Turn", "X:%.2f, Z:%.2f, B:%.2f", x_error, z_error, bearing_error);
            telemetry.addData("Drive Power", "D:%.2f, S:%.2f, T:%.2f", drivePower, strafePower, turnPower);

        } else {
            // No target tag found. Stop the robot.
            drive(0, 0, 0);
            telemetry.addData("Status", "Target Tag ID %d Not Found. Robot Stopped.", TARGET_TAG_ID);
            telemetry.addData("Last Active Camera", isCam1Active ? "Webcam 1" : "Webcam 2");
        }
    }

    /**
     * Finds the target tag. Manages which camera is active and swaps them if the tag is not visible.
     */
    private AprilTagDetection getTargetDetection() {
        List<AprilTagDetection> detections;

        // Use the list from the currently active processor
        if (isCam1Active) {
            detections = aprilTagProcessor1.getDetections();
        } else {
            detections = aprilTagProcessor2.getDetections();
        }

        AprilTagDetection target = null;

        // 1. Check for the target tag in the current set of detections
        for (AprilTagDetection detection : detections) {
            if (detection.id == TARGET_TAG_ID) {
                target = detection;
                break;
            }
        }

        if (target != null) {
            return target;
        }

        // 2. If not found, swap the active camera/processor.
        if (detections.isEmpty()) {
            if (isCam1Active) {
                // Was Camera 1 (Processor 1), Switch to Camera 2 (Processor 2)
                visionPortal1.setProcessorEnabled(aprilTagProcessor1, false);
                visionPortal2.setProcessorEnabled(aprilTagProcessor2, true);
                isCam1Active = false; // Update tracker
                telemetry.addData("Camera Status", "Swapping to Webcam 2...");
            } else {
                // Was Camera 2 (Processor 2), Switch to Camera 1 (Processor 1)
                visionPortal2.setProcessorEnabled(aprilTagProcessor2, false);
                visionPortal1.setProcessorEnabled(aprilTagProcessor1, true);
                isCam1Active = true; // Update tracker
                telemetry.addData("Camera Status", "Swapping to Webcam 1...");
            }
        }

        return null; // No target tag found this cycle
    }

// ----------------------------------------------------------------------------------

    // --- Manual Drive & Utilities ---

    private void manualDrive() {
        telemetry.addLine("Press A to reset heading (logical yaw)");
        telemetry.addLine("Hold left bumper for robot-relative drive");
        telemetry.addLine("Right bumper for slow mode (0.3 speed)");

        // Logical yaw reset
        if (gamepad1.a) {
            yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            drive(forward, right, rotate);
            telemetry.addData("Drive Mode", "Robot-Relative");
        } else {
            driveFieldRelative(forward, right, rotate);
            telemetry.addData("Drive Mode", "Field-Relative");
        }
    }


    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - yawOffset;
        theta = AngleUnit.normalizeRadians(theta - robotYaw);

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    public void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        // Set maximum speed multiplier
        double maxSpeed = 1.0;
        if (isAligning) {
            maxSpeed = MAX_ALIGN_SPEED; // Limited speed during alignment
        } else if (gamepad1.right_bumper) {
            maxSpeed = 0.3; // Manual slow mode
        }

        // Normalize the powers
        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        fldrive.setPower(maxSpeed * frontLeftPower / max);
        frdrive.setPower(maxSpeed * frontRightPower / max);
        bldrive.setPower(maxSpeed * backLeftPower / max);
        brdrive.setPower(maxSpeed * backRightPower / max);
    }

    // --- Telemetry ---

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections;

        // Get detections from the currently active processor
        if (isCam1Active) {
            currentDetections = aprilTagProcessor1.getDetections();
        } else {
            currentDetections = aprilTagProcessor2.getDetections();
        }

        telemetry.addData("# AprilTags Detected (Total)", currentDetections.size());
    }
}