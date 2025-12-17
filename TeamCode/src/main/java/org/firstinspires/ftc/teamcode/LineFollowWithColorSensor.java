package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "godbole3000_SlowLineFollow", group = "Practice")
public class LineFollowWithColorSensor extends OpMode {

    // ---- Drive Motors ----
    private DcMotor fldrive, frdrive, bldrive, brdrive;

    // ---- Sensors ----
    private ColorSensor leftColor, rightColor;
    private IMU imu;

    // ---- Vision ----
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ---- Line Following Parameters ----
    private boolean lineFollowingActive = false;
    private final double basePower = 0.12;
    private final double turnPower = 0.08;
    private final double searchPower = 0.06;
    private final int whiteThreshold = 7000; // Alpha value threshold for white

    @Override
    public void init() {
        // ----- Drive Motors -----
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

        // ----- Color Sensors -----
        leftColor = hardwareMap.get(ColorSensor.class, "leftColor");
        rightColor = hardwareMap.get(ColorSensor.class, "rightColor");

        // ----- IMU -----
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // ----- AprilTag Vision -----
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        telemetry.addLine("Initialized. Press B to follow white line. Press A to stop.");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Gamepad Controls ===
        if (gamepad1.b) lineFollowingActive = true;
        if (gamepad1.a) {
            lineFollowingActive = false;
            stopDriveMotors();
        }

        if (lineFollowingActive) {
            followWhiteLine();
        } else {
            // Manual control
            if (gamepad1.left_bumper) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            }
        }

        // === Telemetry ===
        telemetry.addData("Mode", lineFollowingActive ? "Line Follow ACTIVE" : "Driver Control");

        telemetry.addLine("\n--- Left Sensor ---");
        telemetry.addData("R", leftColor.red());
        telemetry.addData("G", leftColor.green());
        telemetry.addData("B", leftColor.blue());
        telemetry.addData("W (alpha)", leftColor.alpha());
        telemetry.addData("Detected", getColorName(leftColor));

        telemetry.addLine("\n--- Right Sensor ---");
        telemetry.addData("R", rightColor.red());
        telemetry.addData("G", rightColor.green());
        telemetry.addData("B", rightColor.blue());
        telemetry.addData("W (alpha)", rightColor.alpha());
        telemetry.addData("Detected", getColorName(rightColor));

        telemetryAprilTag();
        telemetry.update();
    }

    // ------------------------------------------------------------------
    // ------------------ Line Following Logic --------------------------
    // ------------------------------------------------------------------
    private void followWhiteLine() {
        int leftWhite = leftColor.alpha();
        int rightWhite = rightColor.alpha();

        boolean leftOnWhite = leftWhite > whiteThreshold;
        boolean rightOnWhite = rightWhite > whiteThreshold;

        double leftPower, rightPower;

        if (leftOnWhite && rightOnWhite) {
            // Both sensors see the line → go straight
            leftPower = basePower;
            rightPower = basePower;
        } else if (leftOnWhite && !rightOnWhite) {
            // Turn gently right
            leftPower = basePower - turnPower;
            rightPower = basePower + turnPower;
        } else if (!leftOnWhite && rightOnWhite) {
            // Turn gently left
            leftPower = basePower + turnPower;
            rightPower = basePower - turnPower;
        } else {
            // Lost line → move slowly and bias toward brighter side
            double diff = (leftWhite - rightWhite) / 10000.0; // Normalize
            diff = Math.max(-0.5, Math.min(0.5, diff));
            leftPower = searchPower + diff;
            rightPower = searchPower - diff;
        }

        setAllDrivePower(leftPower, rightPower, leftPower, rightPower);
    }

    // ------------------------------------------------------------------
    // ------------------ Color Name Helper -----------------------------
    // ------------------------------------------------------------------
    private String getColorName(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();
        int w = sensor.alpha();

        if (w > 7000) return "White";
        if (r > g && r > b) return "Red";
        if (g > r && g > b) return "Green";
        if (b > r && b > g) return "Blue";
        if (w < 3000) return "Black";
        return "Unknown";
    }

    // ------------------------------------------------------------------
    // ------------------ Drive Control Methods -------------------------
    // ------------------------------------------------------------------
    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        theta = AngleUnit.normalizeRadians(theta -
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);
        drive(newForward, newRight, rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backRightPower = forward + right - rotate;
        double backLeftPower = forward - right + rotate;

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        fldrive.setPower(frontLeftPower / max);
        frdrive.setPower(frontRightPower / max);
        bldrive.setPower(backLeftPower / max);
        brdrive.setPower(backRightPower / max);
    }

    private void setAllDrivePower(double fl, double fr, double bl, double br) {
        fldrive.setPower(fl);
        frdrive.setPower(fr);
        bldrive.setPower(bl);
        brdrive.setPower(br);
    }

    private void stopDriveMotors() {
        setAllDrivePower(0, 0, 0, 0);
    }

    // ------------------------------------------------------------------
    // ------------------ AprilTag Telemetry ----------------------------
    // ------------------------------------------------------------------
    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
            }
        }
    }
}
