package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous(name = "cam")
public class joshuacam extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() throws InterruptedException {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        waitForStart();
        while (opModeIsActive()) {
            telemetryAprilTag();
            telemetry.update();
        }
        // end method telemetryAprilTag()

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // 1. Declare the variable here, in the method's scope, with a default value.
        double outtakeSpeed = 0; // A value of 0 indicates no target has been seen yet.

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                // Check if the detected tag is a "Target"
                if (detection.metadata.name.contains("Target")) {
                    // 2. Assign the range value to our variable. Do not use 'double' here.
                    outtakeSpeed = detection.ftcPose.range;
                }

                // This block will now run for ALL tags that have metadata, which is cleaner.
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));

            } else {
                // This block runs only for tags with no metadata (e.g., unknown IDs)
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // 3. Now, print the final value of the variable AFTER the loop is done.
        // This gives a summary of the most recently seen target's range.
        telemetry.addData("Outtake Speed = ", outtakeSpeed);


        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    @TeleOp(name = "DualCamMecanumCompatibleIMU", group = "Practice")
    public static class MecanumTwoCams extends OpMode {

        private DcMotor fldrive, frdrive, bldrive, brdrive;
        private IMU imu;
        private double yawOffset = 0;

        private boolean autoEnabled = false;
        private boolean prevButtonB = false;

        @Override
        public void init() {
            // Motors
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
            IMU.Parameters parameters = new IMU.Parameters(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                    )
            );
            imu.initialize(parameters);

            telemetry.addLine("Init complete");
        }

        @Override
        public void loop() {

            // Toggle autonomous mode with B
            boolean buttonB = gamepad1.b;
            if (buttonB && !prevButtonB) autoEnabled = !autoEnabled;
            prevButtonB = buttonB;

            // Reset yaw
            if (gamepad1.a) yawOffset = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double forward = -gamepad1.left_stick_y;
            double right = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            if (!autoEnabled) {
                if (gamepad1.left_bumper) drive(forward, right, rotate);
                else driveFieldRelative(forward, right, rotate);
            } else {
                drive(0.3, 0, 0);
            }

            telemetry.addData("Auto Enabled", autoEnabled);
            telemetry.addData("Yaw Offset", yawOffset);
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
    }
}
