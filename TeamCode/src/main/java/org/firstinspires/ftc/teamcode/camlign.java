package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "DualCamMecanumAprilTag", group = "Practice")
public class camlign extends OpMode {

    // Drive motors
    private DcMotor fldrive, frdrive, bldrive, brdrive;

    // IMU
    private IMU imu;
    private double yawOffset = 0;

    // Vision
    private VisionPortal visionPortal1, visionPortal2;
    private AprilTagProcessor aprilTag1, aprilTag2;

    @Override
    public void init() {
        // Drive motors
        fldrive = hardwareMap.get(DcMotor.class, "flmotor");
        frdrive = hardwareMap.get(DcMotor.class, "frmotor");
        bldrive = hardwareMap.get(DcMotor.class, "blmotor");
        brdrive = hardwareMap.get(DcMotor.class, "brmotor");

        fldrive.setDirection(DcMotor.Direction.FORWARD);
        frdrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.FORWARD);
        brdrive.setDirection(DcMotor.Direction.REVERSE);

        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientation));

        // AprilTag processors
        aprilTag1 = AprilTagProcessor.easyCreateWithDefaults();
        aprilTag2 = AprilTagProcessor.easyCreateWithDefaults();

        // Vision portals (live view disabled with ID 0)
        visionPortal1 = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag1
        ).//setLiveViewContainerId(0);

        visionPortal2 = VisionPortal.easyCreateWithDefaults(
               hardwareMap.get(WebcamName.class, "Webcam 2"), aprilTag2
        );//.setLiveViewContainerId(0);

        telemetry.addLine("Init complete");
        telemetry.update();
    }

    @Override
    public void loop() {
        // Reset yaw
        if (gamepad1.a) yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        if (gamepad1.left_bumper) {
            drive(forward, right, rotate);
        } else {
            driveFieldRelative(forward, right, rotate);
        }

        telemetry.addLine("Camera 1 AprilTags:");
        telemetryAprilTag(aprilTag1);
        telemetry.addLine("Camera 2 AprilTags:");
        telemetryAprilTag(aprilTag2);

        telemetry.update();
    }

    private void driveFieldRelative(double forward, double right, double rotate) {
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(right, forward);
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - yawOffset;
        theta -= robotYaw;
        drive(r * Math.sin(theta), r * Math.cos(theta), rotate);
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        fldrive.setPower(fl / max);
        frdrive.setPower(fr / max);
        bldrive.setPower(bl / max);
        brdrive.setPower(br / max);
    }

    private void telemetryAprilTag(AprilTagProcessor processor) {
        List<AprilTagDetection> detections = processor.getDetections();
        telemetry.addData("# detected", detections.size());
        for (AprilTagDetection d : detections) {
            telemetry.addData("ID " + d.id, "XYZ (%.1f, %.1f, %.1f)", d.ftcPose.x, d.ftcPose.y, d.ftcPose.z);
        }
    }
}
