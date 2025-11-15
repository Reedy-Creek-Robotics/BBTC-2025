/* Copyright (c) 2025 FIRST. */
package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TeleOp: Mecanum (Robot-Relative)", group = "Main")
public class TeleOp_everything_needed extends LinearOpMode {

    // --- Drive & Mechanisms ---
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotor shooter_1, shooter_2, intake, transfer;
    private Servo gateServo;
    private IMU imu;

    // --- Toggle States ---
    private boolean intakeOn = false;
    private boolean xWasPressed = false;

    private boolean fullPowerToggle = false;   // RB = 1.0
    private boolean halfPowerToggle = false;   // LB = 0.5
    private boolean rbWasPressed = false;
    private boolean lbWasPressed = false;

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

    /** Initialize motors, servos and IMU */
    private void initializeHardware() {

        shooter_1 = hardwareMap.get(DcMotor.class, "shooter_1");
        shooter_2 = hardwareMap.get(DcMotor.class, "shooter_2");
        intake    = hardwareMap.get(DcMotor.class, "intake");
        transfer  = hardwareMap.get(DcMotor.class, "transfer");

        gateServo = hardwareMap.get(Servo.class, "gateServo");

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        // Drive motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Shooter directions
        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake & transfer
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer.setDirection(DcMotorSimple.Direction.FORWARD);

        // Drive modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Mechanisms run freely
        shooter_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Servo initial position
        gateServo.setPosition(0.0); // Closed

        // IMU (not used for driving anymore)
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }

    /** Drive Control */
    private void handleDrive() {

        double forward = -gamepad1.left_stick_y;
        double right   = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(fl),
                Math.max(Math.abs(fr),
                        Math.max(Math.abs(bl), Math.abs(br)))));

        flmotor.setPower(fl / max);
        frmotor.setPower(fr / max);
        blmotor.setPower(bl / max);
        brmotor.setPower(br / max);
    }

    /** Intake, Shooter, Transfer & Servo Logic */
    private void handleMechanisms() {

        // --------------------------
        //  INTAKE TOGGLE (X)
        // --------------------------
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn;

            if (intakeOn) {
                gateServo.setPosition(1.0); // open for collecting
            }
        }
        xWasPressed = gamepad1.x;

        intake.setPower(intakeOn ? 0.1 : 0.0);


        // --------------------------
        //  SHOOTER TOGGLES
        //  RB = 1.0
        //  LB = 0.5
        // --------------------------

        // Full power toggle (RB)
        if (gamepad1.right_bumper && !rbWasPressed) {
            fullPowerToggle = !fullPowerToggle;
            if (fullPowerToggle) halfPowerToggle = false;

            gateServo.setPosition(0.0); // close when shooting
        }
        rbWasPressed = gamepad1.right_bumper;

        // Half power toggle (LB)
        if (gamepad1.left_bumper && !lbWasPressed) {
            halfPowerToggle = !halfPowerToggle;
            if (halfPowerToggle) fullPowerToggle = false;

            gateServo.setPosition(0.0);
        }
        lbWasPressed = gamepad1.left_bumper;

        double shooterPower = 0.0;
        if (fullPowerToggle) shooterPower = 1.0;
        else if (halfPowerToggle) shooterPower = 0.5;

        shooter_1.setPower(shooterPower);
        shooter_2.setPower(shooterPower);


        // --------------------------
        //  TRANSFER MOTOR
        //  Runs if intake OR shooter active
        // --------------------------
        boolean transferActive = intakeOn || shooterPower > 0.0;

        transfer.setPower(transferActive ? 0.1 : 0.0);
    }
}
