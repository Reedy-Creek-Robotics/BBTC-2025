/* Copyright (c) 2025 FIRST. */
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "TeleOp: Mecanum (Robot-Relative)", group = "Main")
class TeleOp_RobotRelative extends LinearOpMode {

    // --- Drive & Mechanism Declarations ---
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotor shooter_1, shooter_2, intake;
    private IMU imu;

    // --- State variables ---
    private boolean intakeOn = false;
    private boolean xWasPressed = false;
    private boolean shooterOn = false;
    private boolean bWasPressed = false;

    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();

        telemetry.addLine("Initialization Complete: Press Play!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            handleDrive();
            handleMechanisms();
            telemetry.update();
        }
    }

    /** Initialize motors and IMU */
    private void initializeHardware() {
        shooter_1 = hardwareMap.get(DcMotor.class, "shooter_1");
        shooter_2 = hardwareMap.get(DcMotor.class, "shooter_2");
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        intake = hardwareMap.get(DcMotor.class, "intake");

        // Motor directions (adjust as needed for your robot)
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);
        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Motor modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // IMU setup (not used for drive direction anymore)
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }

    /** Handles all driving logic (Robot-Relative Only) */
    private void handleDrive() {
        telemetry.addLine("\n--- Drive Controls ---");
        telemetry.addLine("Mode: Robot-Relative");

        double forward = -gamepad1.left_stick_y;  // forward/backward
        double right = gamepad1.left_stick_x;     // strafing
        double rotate = gamepad1.right_stick_x;   // rotation

        drive(forward, right, rotate);
    }

    /** Standard mecanum drive kinematics (Robot-Relative) */
    private void drive(double forward, double right, double rotate) {
        // Calculate each wheel’s power
        double frontLeftPower  = forward + right + rotate;
        double frontRightPower = forward - right - rotate;
        double backLeftPower   = forward - right + rotate;
        double backRightPower  = forward + right - rotate;

        // Normalize to keep values between -1.0 and 1.0
        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
        ));

        // Apply power
        flmotor.setPower(frontLeftPower / maxPower);
        frmotor.setPower(frontRightPower / maxPower);
        blmotor.setPower(backLeftPower / maxPower);
        brmotor.setPower(backRightPower / maxPower);
    }

    /** Handles intake and shooter toggle logic */
    private void handleMechanisms() {
        // Intake toggle (X)
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn;
        }
        xWasPressed = gamepad1.x;
        intake.setPower(intakeOn ? 1.0 : 0.0);

        // Shooter toggle (B)
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;
        }
        bWasPressed = gamepad1.b;
        shooter_1.setPower(shooterOn ? 1.0 : 0.0);
        shooter_2.setPower(shooterOn ? 1.0 : 0.0);
    }
}
