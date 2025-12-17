package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "godbole3000_align_clean", group = "Practice")
public class CamAlign extends OpMode {

    // === Motors ===
    private DcMotor fldrive, frdrive, bldrive, brdrive;

    // === IMU ===
    private IMU imu;
    private double yawOffset = 0;  // radians

    // === Vision ===
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // === Auto Align Variables ===
    private boolean autoAlign = false;
    private static final double kX = 0.05;     // Strafe gain
    private static final double kY = 0.05;     // Forward/back gain
    private static final double kTurn = 0.03;  // Rotation gain
    private static final double MAX_SPEED = 0.4;

    private static final double TOL_X = 0.5;   // inches
    private static final double TOL_Y = 0.5;   // inches
    private static final double TOL_YAW = Math.toRadians(1.0); // radians
    private static final long TAG_LOSS_TIMEOUT_MS = 500;

    private double lastX = 0, lastY = 0, lastYaw = 0;  // last tag pose
    private long lastSeenTime = 0;
    private boolean aligned = false;

    @Override
    public void init() {
        // --- Motor setup ---
        fldrive = hardwareMap.get(DcMotor.class, "flmotor");
        frdrive = hardwareMap.get(DcMotor.class, "frmotor");
        bldrive = hardwareMap.get(DcMotor.class, "blmotor");
        brdrive = hardwareMap.get(DcMotor.class, "brmotor");

        fldrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.REVERSE);

        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- IMU setup ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        // --- Vision setup ---
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        telemetry.addLine("Initialized. Press PLAY to start.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Toggle auto alignment
        if (gamepad1.x) autoAlign = true;
        else if (gamepad1.b) autoAlign = false;

        // Reset IMU heading
        if (gamepad1.a) yawOffset = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw());

        // === Drive modes ===
        if (autoAlign) autoAlignToAprilTag();
        else {
            if (gamepad1.left_bumper) drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            else driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        telemetryAprilTag();
        telemetry.addData("Auto Align Mode", autoAlign ? "ON" : "OFF");
        telemetry.addData("Aligned", aligned ? "✅" : "❌");
        telemetry.update();
    }

    // === AUTO ALIGNMENT LOGIC ===
    private void autoAlignToAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean tagVisible = false;

        for (AprilTagDetection tag : detections) {
            if (tag.ftcPose != null) {
                lastX = tag.ftcPose.x;
                lastY = tag.ftcPose.y;
                lastYaw = Math.toRadians(tag.ftcPose.yaw); // convert to radians
                lastSeenTime = System.currentTimeMillis();
                tagVisible = true;
                break;
            }
        }

        long sinceSeen = System.currentTimeMillis() - lastSeenTime;
        if (!tagVisible && sinceSeen > TAG_LOSS_TIMEOUT_MS) {
            drive(0, 0, 0);
            telemetry.addLine("Tag lost - stopping.");
            return;
        }

        // Use last known pose
        double x = lastX;
        double y = lastY;
        double yaw = lastYaw;

        if (Math.abs(x) < TOL_X && Math.abs(y) < TOL_Y && Math.abs(yaw) < TOL_YAW) {
            drive(0, 0, 0);
            aligned = true;
            telemetry.addLine("Aligned ✅");
            return;
        }

        aligned = false;

        // Auto corrections
        double strafeAuto = Range.clip(-x * kX, -MAX_SPEED, MAX_SPEED);
        double forwardAuto = Range.clip(y * kY, -MAX_SPEED, MAX_SPEED);
        double rotateAuto = Range.clip(-yaw * kTurn, -MAX_SPEED, MAX_SPEED);

        // Driver assist
        double forwardDriver = -gamepad1.left_stick_y * 0.5;
        double strafeDriver = gamepad1.left_stick_x * 0.5;
        double rotateDriver = gamepad1.right_stick_x * 0.5;

        double forward = Range.clip(forwardAuto + forwardDriver, -MAX_SPEED, MAX_SPEED);
        double strafe = Range.clip(strafeAuto + strafeDriver, -MAX_SPEED, MAX_SPEED);
        double rotate = Range.clip(rotateAuto + rotateDriver, -MAX_SPEED, MAX_SPEED);

        drive(forward, strafe, rotate);

        telemetry.addLine("Aligning + Driver Assist...");
        telemetry.addData("Tag Visible", tagVisible);
        telemetry.addData("x (Right+)", x);
        telemetry.addData("y (Forward+)", y);
        telemetry.addData("yaw (rad)", yaw);
        telemetry.addData("Forward", forward);
        telemetry.addData("Strafe", strafe);
        telemetry.addData("Rotate", rotate);
    }

    // === FIELD RELATIVE DRIVE ===
    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);

        double robotYaw = Math.toRadians(imu.getRobotYawPitchRollAngles().getYaw()) - yawOffset;

        // Normalize angle between -pi and pi
        theta = theta - robotYaw;
        theta = (theta + Math.PI) % (2 * Math.PI) - Math.PI;

        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        drive(newForward, newRight, rotate);
    }

    // === MECANUM DRIVE ===
    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double br = forward + right - rotate;
        double bl = forward - right + rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        fldrive.setPower(fl / max);
        frdrive.setPower(fr / max);
        bldrive.setPower(bl / max);
        brdrive.setPower(br / max);
    }

    // === TELEMETRY APRILTAG ===
    private void telemetryAprilTag() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null) {
                telemetry.addLine(String.format("\nID %d (%s)", detection.id,
                        detection.metadata != null ? detection.metadata.name : "Unknown"));
                telemetry.addLine(String.format("XYZ (in): %6.1f %6.1f %6.1f",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("Yaw (deg): %6.1f", detection.ftcPose.yaw));
            } else {
                telemetry.addLine(String.format("\nID %d (No pose data)", detection.id));
            }
        }
    }
}
