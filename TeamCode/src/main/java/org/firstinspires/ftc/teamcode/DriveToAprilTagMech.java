package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name="Mecanum DualCam AutoDrive", group="Drive")
public class DriveToAprilTagMech extends LinearOpMode {

    // --- Constants to tune ---
    final double DESIRED_DISTANCE = 12.0;        // desired inches from tag
    final double SPEED_GAIN        = 0.02;       // forward/back gain
    final double STRAFE_GAIN       = 0.015;      // strafe gain
    final double TURN_GAIN         = 0.01;       // rotation gain

    final double MAX_AUTO_SPEED   = 0.5;
    final double MAX_AUTO_STRAFE  = 0.4;
    final double MAX_AUTO_TURN    = 0.25;

    // --- Hardware ---
    private DcMotor flMotor, frMotor, blMotor, brMotor;
    private IMU imu;

    // --- Vision ---
    private VisionPortal frontPortal, rearPortal;
    private AprilTagProcessor frontTagProc, rearTagProc;

    private AprilTagDetection desiredTag = null;

    private double yawOffset = 0;

    // Toggle auto drive mode
    private boolean autoMode = false;
    private boolean lastToggleButton = false;

    @Override
    public void runOpMode() {
        // hardware map
        flMotor = hardwareMap.get(DcMotor.class, "flmotor");
        frMotor = hardwareMap.get(DcMotor.class, "frmotor");
        blMotor = hardwareMap.get(DcMotor.class, "blmotor");
        brMotor = hardwareMap.get(DcMotor.class, "brmotor");

        // reverse directions as needed
        flMotor.setDirection(DcMotor.Direction.REVERSE);
        blMotor.setDirection(DcMotor.Direction.REVERSE);

        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU init
        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Vision init for two cams
        // Front camera
        frontTagProc = new AprilTagProcessor.Builder().build();
        frontPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(frontTagProc)
                .build();

        // Rear camera
        rearTagProc = new AprilTagProcessor.Builder().build();
        rearPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .addProcessor(rearTagProc)
                .build();

        telemetry.addData("Status","Init complete");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {

            // toggle auto mode via gamepad1.b (for example)
            boolean toggleButton = gamepad1.b;
            if (toggleButton && !lastToggleButton) {
                autoMode = !autoMode;
            }
            lastToggleButton = toggleButton;
            telemetry.addData("Auto mode", autoMode);

            // yaw reset
            if (gamepad1.a) {
                yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // Decide which camera to use (for simplicity: front by default, if not seeing tag maybe rear)
            boolean tagFound = false;
            List<AprilTagDetection> frontDetections = frontTagProc.getDetections();
            List<AprilTagDetection> rearDetections  = rearTagProc.getDetections();

            // Use front camera first
            if (frontDetections.size() > 0) {
                for (AprilTagDetection d : frontDetections) {
                    if (d.metadata != null) {
                        desiredTag = d;
                        tagFound = true;
                        break;
                    }
                }
            }
            // If no front tag and autoMode is ON, check rear camera
            if (!tagFound && autoMode) {
                if (rearDetections.size() > 0) {
                    for (AprilTagDetection d : rearDetections) {
                        if (d.metadata != null) {
                            desiredTag = d;
                            tagFound = true;
                            break;
                        }
                    }
                }
            }

            double forward = 0;
            double strafe  = 0;
            double rotate  = 0;

            if (autoMode && tagFound && desiredTag != null) {
                // automatic drive
                double rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double bearingError = desiredTag.ftcPose.bearing;
                double lateralError = desiredTag.ftcPose.y;

                forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe  = Range.clip(- lateralError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                rotate  = Range.clip(bearingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                telemetry.addData("AUTO drive", "fwd=%.2f strafe=%.2f rot=%.2f", forward, strafe, rotate);
            } else {
                // manual control
                double y = -gamepad1.left_stick_y;   // forward/back
                double x =  gamepad1.left_stick_x;   // strafe
                double r =  gamepad1.right_stick_x;  // rotate

                // Choose field-relative or robot-relative via left bumper
                if (gamepad1.left_bumper) {
                    driveFieldRelative(y, x, r);
                } else {
                    driveRobotRelative(y, x, r);
                }

                telemetry.addData("Manual drive", "y=%.2f x=%.2f r=%.2f", y, x, r);
                // then loop
                telemetry.update();
                continue;
            }

            // If here: we computed forward, strafe, rotate in auto mode
            moveMecanum(forward, strafe, rotate);

            // Telemetry tag info
            if (tagFound) {
                telemetry.addData("Tag ID", desiredTag.id);
                telemetry.addData("Range (in)", "%.1f", desiredTag.ftcPose.range);
                telemetry.addData("Bearing (deg)", "%.1f", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw (deg)", "%.1f", desiredTag.ftcPose.yaw);
            } else {
                telemetry.addLine("No tag detected");
            }

            telemetry.update();
            sleep(20);
        }
    }

    // --- Drive methods ---
    private void driveRobotRelative(double forward, double right, double rotate) {
        moveMecanum(forward, right, rotate);
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - yawOffset;
        double tempF   = forward * Math.cos(heading) + right * Math.sin(heading);
        double tempR   = -forward * Math.sin(heading) + right * Math.cos(heading);
        moveMecanum(tempF, tempR, rotate);
    }

    private void moveMecanum(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        flMotor.setPower(fl / max);
        frMotor.setPower(fr / max);
        blMotor.setPower(bl / max);
        brMotor.setPower(br / max);
    }

}
