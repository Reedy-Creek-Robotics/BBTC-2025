package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

    /*
     * This OpMode illustrates how to program your robot to drive field relative.  This means
     * that the robot drives the direction you push the joystick regardless of the current orientation
     * of the robot.
     *
     * This OpMode assumes that you have four mecanum wheels each on its own motor named:
     *   front_left_motor, front_right_motor, back_left_motor, back_right_motor
     *
     *   and that the left motors are flipped such that when they turn clockwise the wheel moves backwards
     *
     * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
     * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
     *
     */
    @TeleOp(name = "Robot: Field Relative Mecanum Drive", group = "Robot")
    @Disabled
    public static class outreachbot extends OpMode {
        // This declares the four motors needed
        DcMotor flmotor;
        DcMotor frmotor;
        DcMotor blmotor;
        DcMotor brmotor;

        // This declares the IMU needed to get the current direction the robot is facing
        IMU imu;

        @Override
        public void init() {
            flmotor = hardwareMap.get(DcMotor.class, "flmotor");
            frmotor = hardwareMap.get(DcMotor.class, "frmotor");
            blmotor = hardwareMap.get(DcMotor.class, "blmotor");
            brmotor = hardwareMap.get(DcMotor.class, "brmotor");

            // We set the left motors in reverse which is needed for drive trains where the left
            // motors are opposite to the right ones.
            blmotor.setDirection(DcMotor.Direction.FORWARD);
            flmotor.setDirection(DcMotor.Direction.FORWARD);
            frmotor.setDirection(DcMotor.Direction.REVERSE);
            brmotor.setDirection(DcMotor.Direction.REVERSE);

            // This uses RUN_USING_ENCODER to be more accurate.   If you don't have the encoder
            // wires, you should remove these
            flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            imu = hardwareMap.get(IMU.class, "imu");
            // This needs to be changed to match the orientation on your robot
            RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                    RevHubOrientationOnRobot.LogoFacingDirection.UP;
            RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

            RevHubOrientationOnRobot orientationOnRobot = new
                    RevHubOrientationOnRobot(logoDirection, usbDirection);
            imu.initialize(new IMU.Parameters(orientationOnRobot));
        }

        @Override
        public void loop() {
            telemetry.addLine("Press A to reset Yaw");
            telemetry.addLine("Hold left bumper to drive in robot relative");
            telemetry.addLine("The left joystick sets the robot direction");
            telemetry.addLine("Moving the right joystick left and right turns the robot");

            // If you press the A button, then you reset the Yaw to be zero from the way
            // the robot is currently pointing
            if (gamepad1.a) {
                imu.resetYaw();
            }
            // If you press the left bumper, you get a drive from the point of view of the robot
            // (much like driving an RC vehicle)
            if (gamepad1.left_bumper) {
                drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
            } else {
                driveFieldRelative(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
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
            double maxSpeed = 1.0;  // make this slower for outreaches

            // This is needed to make sure we don't pass > 1.0 to any wheel
            // It allows us to keep all of the motors in proportion to what they should
            // be and not get clipped
            maxPower = Math.max(maxPower, Math.abs(frontLeftPower));
            maxPower = Math.max(maxPower, Math.abs(frontRightPower));
            maxPower = Math.max(maxPower, Math.abs(backRightPower));
            maxPower = Math.max(maxPower, Math.abs(backLeftPower));

            // We multiply by maxSpeed so that it can be set lower for outreaches
            // When a young child is driving the robot, we may not want to allow full
            // speed.
            flmotor.setPower(maxSpeed * (frontLeftPower / maxPower));
            frmotor.setPower(maxSpeed * (frontRightPower / maxPower));
            blmotor.setPower(maxSpeed * (backLeftPower / maxPower));
            brmotor.setPower(maxSpeed * (backRightPower / maxPower));
        }
    }
}
