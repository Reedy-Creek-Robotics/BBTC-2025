        /* Copyright (c) 2025 FIRST. */
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

@TeleOp(name = "TeleOp: Mecanum (Robot-Relative)_3")
public class TeleOp_everything_needed extends LinearOpMode {

    // --- Drive & Mechanism Declarations ---
    private DcMotor flmotor, frmotor, blmotor, brmotor;
    private DcMotorEx shooter_1, shooter_2, intakeTransfer;
    private Servo intakeServo;
    // This is the blocker servo
    private IMU imu;

    // --- State variables ---
    private boolean shooterOn = false;
    private boolean bWasPressed = false;
    private boolean intakeOn = false;
    private boolean xWasPressed = false;

    // --- Reversal detection state ---
    private double lastForward = 0;
    private double lastRight = 0;

    private long lastDirectionChangeTime = 0;
    private static final long REVERSAL_DELAY_MS = 120;

    // --- Slew rate limiter state ---
    private double limitedForward = 0;
    private double limitedRight = 0;
    private double limitedRotate = 0;

    // Slew rate settings (tune as needed)
    private static final double MAX_ACCEL = 0.08;   // acceleration per loop
    private static final double MAX_DECEL = 0.12;   // deceleration per loop

    private double servoCounter;


    @Override
    public void runOpMode() throws InterruptedException {
        initializeHardware();
        telemetry.update();
        waitForStart();
        while (opModeIsActive()) {
            handleDrive();
            handleMechanisms();
            telemetry.update();
        }
    }


    /**
     * Initialize motors, servo, and IMU
     */
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
        shooter_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        servoCounter = 0;

        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }

    /** Driving logic with reversal protection + slew rate limiting */
    /**
     * Handles all driving logic (Robot-Relative Only)
     */
    private void handleDrive() {
        telemetry.addLine("\n--- Drive Controls ---");
        telemetry.addLine("Mode: Robot-Relative");

        double forward = -gamepad1.left_stick_y;
        double right = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        long now = System.currentTimeMillis();

        // --- Detect reversals ---
        boolean forwardReversal =
                (Math.signum(forward) == -Math.signum(lastForward)) &&
                        Math.abs(forward) > 0.3 && Math.abs(lastForward) > 0.3;

        boolean strafeReversal =
                (Math.signum(right) == -Math.signum(lastRight)) &&
                        Math.abs(right) > 0.3 && Math.abs(lastRight) > 0.3;

        if (forwardReversal || strafeReversal) {
            lastDirectionChangeTime = now;
        }

        // --- Apply reversal pause ---
        if (now - lastDirectionChangeTime < REVERSAL_DELAY_MS) {
            forward = 0;
            right = 0;
        }

        // --- Apply slew rate limiting ---
        limitedForward = applySlewRate(limitedForward, forward);
        limitedRight = applySlewRate(limitedRight, right);
        limitedRotate = applySlewRate(limitedRotate, rotate);

        // Save for next-loop reversal detection
        lastForward = forward;
        lastRight = right;

        drive(limitedForward, limitedRight, limitedRotate);
    }

    /**
     * Slew rate limiter function
     */
    private double applySlewRate(double current, double target) {
        double delta = target - current;

        if (delta > 0) {
            delta = Math.min(delta, MAX_ACCEL);
        } else {
            delta = Math.max(delta, -MAX_DECEL);
        }
        double forward = -gamepad1.left_stick_y;  // forward/backward
        double right = gamepad1.left_stick_x;     // strafing
        double rotate = gamepad1.right_stick_x;   // rotation

        return current + delta;
    }

    /** Standard mecanum drive */
    /**
     * Standard mecanum drive kinematics (Robot-Relative)
     */
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

    /** Mechanisms (unchanged from your original) */
    /**
     * Mechanisms
     */
    private void printRpm() {
        telemetry.addData("Shooter 1 RPM", "%.2f", getRpm(shooter_1));
        telemetry.addData("Shooter 2 RPM", "%.2f", getRpm(shooter_2));
        telemetry.update();
    }

    private double getRpm(DcMotorEx motor) {
        // getVelocity() returns ticks per second.
        // To convert ticks/sec to revolutions/minute:
        // (ticks/sec) * (60 sec/min) / (ticks/rev) = rev/min
        return (motor.getVelocity() * 60) / TICKS_PER_REVOLUTION;
    }

    /**
     * Handles intake, transfer, shooter, and blocker servo logic
     */
    private void handleMechanisms() {
        // Intake toggle (X)
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn;
        }
        xWasPressed = gamepad1.x;

        // Shooter toggle (B) - full power
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;
        }
        bWasPressed = gamepad1.b;

        // --- SHOOTER POWER ---
        if (shooterOn) {
            shooter_1.setPower(0.8);
            shooter_2.setPower(0.8);
        } else{
            shooter_2.setPower(0.0);
            shooter_1.setPower(0.0);
        }

        // --- INTAKE POWER ---
        if(intakeOn || shooterOn){
            intakeTransfer.setPower(1.0);
        } else{
            intakeTransfer.setPower(0.0);
        }

        // --- SERVO LOGIC (clean & reliable) ---
        if (gamepad1.y) {
            servoCounter = servoCounter + 1;
        }
        if (servoCounter == 1) {
            intakeServo.setPosition(0.8);
        } else if (servoCounter == 2) {
            intakeServo.setPosition(0.38);
        } else if (servoCounter > 2) {
            servoCounter = 0;
        }
        if(intakeServo.getPosition() == 0.8){
            telemetry.addLine("servo position is: UP");
        } else{
            telemetry.addLine("servo position is: DOWN");
        }
    }
}