/* Copyright (c) 2025 FIRST. */

package org.firstinspires.ftc.teamcode.mechanisms;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp joshua")
public class TeleOp_everything_needed_servo_copy extends LinearOpMode {

    // --- Drive & Mechanism Declarations ---
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotor shooter_1, shooter_2, intake;
    private IMU imu;
    private Servo intakeServo;

    // --- State variables ---
    private boolean intakeOn = false;
    private boolean xWasPressed = false;
    private float shooterOn = 0;
    private double flmod = 1;
    private double frmod = 1;
    private double blmod = 1;
    private double brmod = 1;
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
        intake = hardwareMap.get(DcMotor.class, "intakeTransfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
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

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(logoDirection, usbDirection)));

        telemetry.addLine("Hardware initialized");
    }

    /** Handles all driving logic */
    private void handleDrive() {
        telemetry.addLine("\n--- Drive Controls ---");
        telemetry.addLine("Hold left bumper for robot-relative drive");
        telemetry.addLine("Press A to reset heading");

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Reset yaw if requested
        if (gamepad1.a) imu.resetYaw();

        // Field-relative by default
    }

    /** Toggle intake and shooter mechanisms */
    @SuppressLint("SuspiciousIndentation")
    private void handleMechanisms() {
        // Intake toggle (X)
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn;
        }
        xWasPressed = gamepad1.x;
        intake.setPower(intakeOn ? 1.0 : 0.0);

        if (gamepad1.right_bumper) {
            for(int i = 0; i < 3; i++ ); {
                shooterOn = 1;
                sleep(1310);
                intakeServo.setPosition(1);
                sleep(2000);
                intakeServo.setPosition(0);
                shooterOn = 0;
                sleep(2000);
            }
        }
        if (gamepad1.right_trigger >= 0.5) {
            for(int i = 0; i < 3; i++ ); {
                shooterOn = 2;
                sleep(1310);
                intakeServo.setPosition(1);
                sleep(2000);
                intakeServo.setPosition(0);
                shooterOn = 0;
                sleep(2000);
            }
        }

        if (shooterOn == 0) {
            shooter_1.setPower(0);
            shooter_2.setPower(0);
        }
        else if (shooterOn == 2) {
            shooter_1.setPower(0.65);
            shooter_2.setPower(0.65);
        }
        if (shooterOn == 1 ){
            shooter_1.setPower(0.35);
            shooter_2.setPower(0.35);

        }
    }

    /** Standard mecanum drive kinematics */
    private void drive(double forward, double right, double rotate) {
        double frontLeftPower = forward + right + rotate + flmod;
        double frontRightPower = forward - right - rotate + frmod;
        double backRightPower = forward + right - rotate + blmod;
        double backLeftPower = forward - right + rotate + brmod;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backRightPower), Math.abs(backLeftPower))
        ));

        // Normalize and apply
        flmotor.setPower(frontLeftPower / maxPower);
        frmotor.setPower(frontRightPower / maxPower);
        blmotor.setPower(backLeftPower / maxPower);
        brmotor.setPower(backRightPower / maxPower);
    }
}
