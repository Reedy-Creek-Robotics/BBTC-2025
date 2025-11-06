package org.firstinspires.ftc.teamcode;

import android.widget.LinearLayout;

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

    // ---------- CONFIGURATION ----------
    final double DESIRED_DISTANCE = 12.0;   // inches from tag
    final double SPEED_GAIN  = 0.02;
    final double STRAFE_GAIN = 0.015;
    final double TURN_GAIN   = 0.01;

    final double MAX_AUTO_SPEED  = 0.5;
    final double MAX_AUTO_STRAFE = 0.4;
    final double MAX_AUTO_TURN   = 0.25;

    // ---------- HARDWARE ----------
    private DcMotor flMotor, frMotor, blMotor, brMotor;
    private IMU imu;
    private double yawOffset = 0;

    // ---------- VISION ----------
    private VisionPortal frontPortal, rearPortal;
    private AprilTagProcessor frontTagProc, rearTagProc;
    private AprilTagDetection desiredTag = null;

    private boolean autoMode = false;
    private boolean lastToggleButton = false;

    @Override
    public void runOpMode() {

        // -------------- DRIVE SETUP --------------
        flMotor = hardwareMap.get(DcMotor.class, "flmotor");
        frMotor = hardwareMap.get(DcMotor.class, "frmotor");
        blMotor = hardwareMap.get(DcMotor.class, "blmotor");
        brMotor = hardwareMap.get(DcMotor.class, "brmotor");

        flMotor.setDirection(DcMotor.Direction.FORWARD); // directly connected
        frMotor.setDirection(DcMotor.Direction.REVERSE); // 90 deg + gear
        blMotor.setDirection(DcMotor.Direction.REVERSE); // 90 deg + gear
        brMotor.setDirection(DcMotor.Direction.FORWARD); // directly connected


        flMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // -------------- IMU SETUP --------------
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // -------------- CAMERA PREVIEW LAYOUT --------------
        // Get the main camera monitor layout from the RC app
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        LinearLayout cameraMonitorView =
                (LinearLayout) hardwareMap.appContext.getResources().getLayout(cameraMonitorViewId);

        // Create two camera preview containers
        LinearLayout.LayoutParams params = new LinearLayout.LayoutParams(
                LinearLayout.LayoutParams.MATCH_PARENT,
                LinearLayout.LayoutParams.MATCH_PARENT, 1.0f);

        LinearLayout frontLayout = new LinearLayout(hardwareMap.appContext);
        LinearLayout rearLayout  = new LinearLayout(hardwareMap.appContext);
        frontLayout.setLayoutParams(params);
        rearLayout.setLayoutParams(params);

        cameraMonitorView.setOrientation(LinearLayout.HORIZONTAL);
        cameraMonitorView.addView(frontLayout);
        cameraMonitorView.addView(rearLayout);

        // Get unique container IDs for each
        int frontViewId = frontLayout.getId();
        if (frontViewId == -1) frontViewId = ViewIdGenerator.generateViewId();
        frontLayout.setId(frontViewId);

        int rearViewId = rearLayout.getId();
        if (rearViewId == -1) rearViewId = ViewIdGenerator.generateViewId();
        rearLayout.setId(rearViewId);

        // -------------- VISION PORTALS --------------
        frontTagProc = new AprilTagProcessor.Builder().build();
        frontPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setLiveViewContainerId(frontViewId)
                .addProcessor(frontTagProc)
                .build();

        rearTagProc = new AprilTagProcessor.Builder().build();
        rearPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setLiveViewContainerId(rearViewId)
                .addProcessor(rearTagProc)
                .build();

        telemetry.addLine("Dual Cameras Initialized.");
        telemetry.addLine("Press [A] to zero IMU yaw.");
        telemetry.addLine("Press [B] to toggle AUTO mode.");
        telemetry.update();

        waitForStart();

        // -------------- MAIN LOOP --------------
        while (opModeIsActive()) {

            // --- Toggle AUTO mode ---
            boolean toggleButton = gamepad1.b;
            if (toggleButton && !lastToggleButton) {
                autoMode = !autoMode;
            }
            lastToggleButton = toggleButton;
            telemetry.addData("AUTO MODE", autoMode);

            // --- Reset heading offset ---
            if (gamepad1.a) {
                yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }

            // --- Detect Tags from front, fallback to rear ---
            desiredTag = getFirstValidTag(frontTagProc);
            if (desiredTag == null)
                desiredTag = getFirstValidTag(rearTagProc);

            if (autoMode && desiredTag != null) {
                // Auto align to tag
                double rangeError   = desiredTag.ftcPose.range - DESIRED_DISTANCE;
                double bearingError = desiredTag.ftcPose.bearing;
                double lateralError = desiredTag.ftcPose.y;

                double forward = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                double strafe  = Range.clip(-lateralError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                double rotate  = Range.clip(bearingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN);

                moveMecanum(forward, strafe, rotate);

                telemetry.addData("AUTO", "Fwd %.2f  Str %.2f  Rot %.2f", forward, strafe, rotate);
                telemetry.addData("Range (in)", "%.1f", desiredTag.ftcPose.range);
                telemetry.addData("Bearing (deg)", "%.1f", desiredTag.ftcPose.bearing);
            }
            else {
                // Manual control
                double y = -gamepad1.left_stick_y;
                double x =  gamepad1.left_stick_x;
                double r =  gamepad1.right_stick_x;

                if (gamepad1.left_bumper)
                    driveFieldRelative(y, x, r);
                else
                    driveRobotRelative(y, x, r);
            }

            telemetry.update();
            sleep(20);
        }
    }

    // ---------- UTILITIES ----------

    private AprilTagDetection getFirstValidTag(AprilTagProcessor proc) {
        List<AprilTagDetection> detections = proc.getDetections();
        for (AprilTagDetection d : detections) {
            if (d.metadata != null) return d;
        }
        return null;
    }

    private void driveRobotRelative(double fwd, double str, double rot) {
        moveMecanum(fwd, str, rot);
    }

    private void driveFieldRelative(double fwd, double str, double rot) {
        double heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - yawOffset;
        double tempF = fwd * Math.cos(heading) + str * Math.sin(heading);
        double tempS = -fwd * Math.sin(heading) + str * Math.cos(heading);
        moveMecanum(tempF, tempS, rot);
    }

    private void moveMecanum(double fwd, double str, double rot) {
        double fl = fwd + str + rot;
        double fr = fwd - str - rot;
        double bl = fwd - str + rot;
        double br = fwd + str - rot;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        flMotor.setPower(fl / max);
        frMotor.setPower(fr / max);
        blMotor.setPower(bl / max);
        brMotor.setPower(br / max);
    }

    // Generates unique IDs for preview containers
    private static class ViewIdGenerator {
        private static int nextId = 1;
        public static synchronized int generateViewId() {
            return nextId++;
        }
    }
}
