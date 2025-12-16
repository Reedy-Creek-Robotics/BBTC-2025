package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "AprilTag Auto Align Full FTC", group = "Vision")
public class AprilTagAutoAlignFTC extends OpMode {

    private DcMotor fldrive, frdrive, bldrive, brdrive;
    private IMU imu;
    private Limelight3A limelight;

    // --- TUNING CONSTANTS ---
    private final double STRAFE_MULTIPLIER = 1.35;   // boosts strafing (not used here)
    private final double ROTATE_MULTIPLIER = 0.55;   // reduces rotation sensitivity

    private static final double CAMERA_X = 0.28; // meters forward
    private static final double CAMERA_Y = 0.15; // meters left/right

    private static final double TARGET_X = 2.5;   // meters forward
    private static final double TARGET_YAW = 0.0; // radians (forward)

    private static final double KP_POS = 0.5; // proportional gain for position
    private static final double KP_YAW = 1.0; // proportional gain for yaw

    @Override
    public void init() {
        // Drive motors
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

        // IMU initialization
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Limelight3A
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();
        limelight.setPollRateHz(50);

        telemetry.addLine("AprilTag Auto Align Full Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            telemetry.addData("Vision", "No valid AprilTag");
            drive(0, 0, 0);
            telemetry.update();
            return;
        }

        // Get robot pose
        Pose3D botpose = result.getBotpose();
        Position pos = botpose.getPosition();
        YawPitchRollAngles orient = botpose.getOrientation();

        double robotX   = pos.x;
        double robotYaw = orient.getYaw();

        // Compensate camera offset
        double[] corrected = compensateCameraPose(robotX, robotYaw);
        double currentX   = corrected[0];
        double currentYaw = corrected[1];

        // Compute errors
        double errorX   = TARGET_X - currentX;
        double errorYaw = angleDifference(TARGET_YAW, currentYaw);

        // Smooth blending
        double forward = KP_POS * errorX;
        double rotate  = KP_YAW * errorYaw;

        // Clamp values to prevent skidding
        forward = clamp(forward, -0.3, 0.3);
        rotate  = clamp(rotate, -0.2, 0.2);

        // Reduce forward if rotation is large
        forward *= 1.0 - Math.min(1.0, Math.abs(rotate) / 0.2);

        // Apply TeleOp rotation multiplier
        rotate *= ROTATE_MULTIPLIER;

        // Use TeleOpOutreach mecanum drive, right/strafe = 0
        drive(forward, 0, rotate);

        telemetry.addData("Robot X (m)", currentX);
        telemetry.addData("Yaw (deg)", Math.toDegrees(currentYaw));
        telemetry.addData("Error X", errorX);
        telemetry.addData("Error Yaw", Math.toDegrees(errorYaw));
        telemetry.addData("Forward", forward);
        telemetry.addData("Rotate", rotate);
        telemetry.update();
    }

    // TeleOpOutreach mecanum drive
    public void drive(double forward, double right, double rotate) {
        right *= STRAFE_MULTIPLIER;  // boost strafing (though we use 0)
        rotate *= ROTATE_MULTIPLIER; // reduce rotation sensitivity again

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(fr),
                                Math.max(Math.abs(bl), Math.abs(br)))));

        fldrive.setPower(fl / max);
        frdrive.setPower(fr / max);
        bldrive.setPower(bl / max);
        brdrive.setPower(br / max);
    }

    private double[] compensateCameraPose(double robotX, double robotYaw) {
        double cosYaw = Math.cos(robotYaw);
        double sinYaw = Math.sin(robotYaw);
        double camOffsetX = CAMERA_X * cosYaw - CAMERA_Y * sinYaw;
        double trueX = robotX - camOffsetX;
        return new double[]{trueX, robotYaw};
    }

    private double angleDifference(double target, double current) {
        double diff = target - current;
        while (diff > Math.PI)  diff -= 2*Math.PI;
        while (diff < -Math.PI) diff += 2*Math.PI;
        return diff;
    }

    private double clamp(double val, double min, double max) {
        return Math.max(min, Math.min(max, val));
    }
}
