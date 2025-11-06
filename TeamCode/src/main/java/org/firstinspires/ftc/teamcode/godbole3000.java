package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "godbole3000_fixed", group = "Practice")
public class godbole3000 extends OpMode {

    // Motors
    private DcMotor fldrive, frdrive, bldrive, brdrive;

    // IMU
    private IMU imu;
    private double yawOffset = 0;  // <-- Logical yaw zero offset

    // Vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

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

        // Vision Setup
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
    }

    @Override
    public void loop() {
        telemetry.addLine("Press A to reset heading (logical yaw)");
        telemetry.addLine("Hold left bumper for robot-relative drive");
        telemetry.addLine("Left stick = move | Right stick = rotate");

        // Logical yaw reset
        if (gamepad1.a) {
            yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        }

        // Drive Modes
        if (gamepad1.left_bumper) {
            drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        } else {
            driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // AprilTag telemetry
        telemetryAprilTag();
        telemetry.update();
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

        double max = Math.max(1.0, Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)))));

        double maxSpeed = 1.0;

        fldrive.setPower(maxSpeed * frontLeftPower / max);
        frdrive.setPower(maxSpeed * frontRightPower / max);
        bldrive.setPower(maxSpeed * backLeftPower / max);
        brdrive.setPower(maxSpeed * backRightPower / max);
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        double outtakeSpeed = 0;

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                if (detection.metadata.name.contains("Target")) {
                    outtakeSpeed = detection.ftcPose.range;
                }

                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)",
                        detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)",
                        detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)",
                        detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)",
                        detection.center.x, detection.center.y));
            }
        }

        telemetry.addData("Outtake Speed = ", outtakeSpeed);
        telemetry.addLine("\nKey:\nXYZ = X (Right), Y (Forward), Z (Up)\nPRY = Pitch, Roll, Yaw\nRBE = Range, Bearing, Elevation");
    }
}
