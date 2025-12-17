package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Outreach Robot: Robot Relative Drive", group = "Robot")

public class TeleOpOutreach extends OpMode {

    private DcMotor fldrive;
    private DcMotor frdrive;
    private DcMotor bldrive;
    private DcMotor brdrive;

    private IMU imu;  // Added IMU

    // --- TUNING CONSTANTS ---
    private final double STRAFE_MULTIPLIER = 1.35;   // boosts left/right strafing
    private final double ROTATE_MULTIPLIER = 0.55;   // reduces rotation sensitivity

    @Override
    public void init() {

        fldrive = hardwareMap.get(DcMotor.class, "flmotor");
        frdrive = hardwareMap.get(DcMotor.class, "frmotor");
        bldrive = hardwareMap.get(DcMotor.class, "blmotor");
        brdrive = hardwareMap.get(DcMotor.class, "brmotor");

        // reverse left motors
        fldrive.setDirection(DcMotor.Direction.REVERSE);
        bldrive.setDirection(DcMotor.Direction.REVERSE);

        fldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bldrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brdrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addLine("Robot-relative drive ONLY");
    }

    @Override
    public void loop() {

        // Reset yaw if A is pressed
        if (gamepad1.a) {
            imu.resetYaw();
        }

        double forward = -gamepad1.left_stick_x;
        double right   =  gamepad1.left_stick_y;
        double rotate  =  gamepad1.right_stick_x;

        // Apply precision tuning
        right  *= STRAFE_MULTIPLIER;
        rotate *= ROTATE_MULTIPLIER;

        drive(forward, right, rotate);
    }

    // Robot-relative mecanum drive
    public void drive(double forward, double right, double rotate) {

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
}
