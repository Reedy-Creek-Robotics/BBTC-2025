/* Copyright (c) 2025 FIRST */
package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.mechanisms.TestBench.TICKS_PER_REVOLUTION;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;

@TeleOp(name = "TeleOp: Mecanum (Robot-Relative intake forward)_3")
public class TeleOp_intake_forward extends LinearOpMode {

    // --- Drive & Mechanism Declarations ---
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotorEx shooter_1, shooter_2, intakeTransfer;
    private Servo intakeServo;
    private IMU imu;

    // --- State variables ---
    private boolean shooterOn = false;
    private boolean bWasPressed = false;

    private boolean yWasPressed = false;
    private boolean intakeOn = false;

    private boolean servoUp = false;
    private boolean xWasPressed = false;

    // --- Shooter speed control ---
    private static final double SHOOTER_TARGET_RPM = 5400;
    private boolean shooterReady = false;

    // --- Reversal detection ---
    private double lastForward = 0;
    private double lastRight = 0;
    private long lastDirectionChangeTime = 0;
    private static final long REVERSAL_DELAY_MS = 120;

    // --- Slew rate limiting ---
    private double limitedForward = 0;
    private double limitedRight = 0;
    private double limitedRotate = 0;

    private static final double MAX_ACCEL = 0.08;
    private static final double MAX_DECEL = 0.12;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.update();
        waitForStart();
        intakeServo.setPosition(0.8);

        while (opModeIsActive()) {
            handleDrive();
            handleMechanisms();
            telemetry.update();
        }
    }

    /** Initialize motors, servo, and IMU */
    private void initializeHardware() {
        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter_2");

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);

        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter PIDF (recommended starting values)
        shooter_1.setVelocityPIDFCoefficients(0.1, 0.0, 0.0, 12.0);
        shooter_2.setVelocityPIDFCoefficients(0.1, 0.0, 0.0, 12.0);

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }

    /** Handles driving + reversal protection + slew rate limiting */
    private void handleDrive() {
        telemetry.addLine("\n--- Drive Controls ---");
        telemetry.addLine("Mode: Robot-Relative");

        double forward = -gamepad1.left_stick_y; // Intake side is forward
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        long now = System.currentTimeMillis();

        // Detect movement reversals
        boolean forwardReversal =
                (Math.signum(forward) == -Math.signum(lastForward)) &&
                        Math.abs(forward) > 0.3 &&
                        Math.abs(lastForward) > 0.3;

        boolean strafeReversal =
                (Math.signum(right) == -Math.signum(lastRight)) &&
                        Math.abs(right) > 0.3 &&
                        Math.abs(lastRight) > 0.3;

        if (forwardReversal || strafeReversal)
            lastDirectionChangeTime = now;

        // Apply reversal pause
        if (now - lastDirectionChangeTime < REVERSAL_DELAY_MS) {
            forward = 0;
            right = 0;
        }

        // Apply slew rate limiter
        limitedForward = applySlewRate(limitedForward, forward);
        limitedRight = applySlewRate(limitedRight, right);
        limitedRotate = applySlewRate(limitedRotate, rotate);

        lastForward = forward;
        lastRight = right;

        drive(limitedForward, limitedRight, limitedRotate);
    }

    /** Slew rate limiter */
    private double applySlewRate(double current, double target) {
        double delta = target - current;

        if (delta > 0) delta = Math.min(delta, MAX_ACCEL);
        else           delta = Math.max(delta, -MAX_DECEL);

        return current + delta;
    }

    /** Standard robot-relative mecanum drive */
    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))
        ));

        flmotor.setPower(fl / maxPower);
        frmotor.setPower(fr / maxPower);
        blmotor.setPower(bl / maxPower);
        brmotor.setPower(br / maxPower);
    }

    /** RPM calculation */
    private double getRpm(DcMotorEx motor) {
        return (motor.getVelocity() * 60) / TICKS_PER_REVOLUTION;
    }

    /** Mechanism controls: shooter, intake, servo */
    private void handleMechanisms() {

        // Intake toggle (X)
        if (gamepad1.x && !xWasPressed) intakeOn = !intakeOn;
        xWasPressed = gamepad1.x;

        // Shooter toggle (B)
        if (gamepad1.b && !bWasPressed) shooterOn = !shooterOn;
        bWasPressed = gamepad1.b;

        // Blocker servo toggle (Y)
        if (gamepad1.y && !yWasPressed) servoUp = !servoUp;
        yWasPressed = gamepad1.y;

        intakeServo.setPosition(servoUp ? 0.8 : 0.38);

        // Shooter target in ticks/sec
        double targetVelocity = (SHOOTER_TARGET_RPM / 60.0) * TICKS_PER_REVOLUTION;

        if (shooterOn) {
            // PIDF shooter control
            shooter_1.setVelocity(targetVelocity);
            shooter_2.setVelocity(targetVelocity);

            // Actual RPM
            double rpm1 = getRpm(shooter_1);
            double rpm2 = getRpm(shooter_2);
            double avgRpm = (rpm1 + rpm2) / 2.0;

            shooterReady = Math.abs(avgRpm - SHOOTER_TARGET_RPM) < 75;

            telemetry.addData("Shooter RPM Avg", avgRpm);
            telemetry.addData("Shooter Ready", shooterReady);
        } else {
            shooter_1.setPower(0);
            shooter_2.setPower(0);
            shooterReady = false;
        }

        // Intake
        if (intakeOn) intakeTransfer.setPower(1.0);
        else          intakeTransfer.setPower(0.0);

        telemetry.addLine(servoUp ? "Gate: UP" : "Gate: DOWN");
    }
}
