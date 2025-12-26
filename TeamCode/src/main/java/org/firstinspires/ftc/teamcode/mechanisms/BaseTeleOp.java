package org.firstinspires.ftc.teamcode.mechanisms;

import static org.firstinspires.ftc.teamcode.mechanisms.TestBench.TICKS_PER_REVOLUTION;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

public abstract class BaseTeleOp extends LinearOpMode {

    protected DcMotor flmotor, frmotor, blmotor, brmotor;
    protected DcMotorEx shooter_1, intakeTransfer;
    protected CRServo intakeServo;
    protected double limitedForward = 0, limitedRight = 0, limitedRotate = 0;

    private IMU imu;

    protected static final double MAX_ACCEL = 0.08, MAX_DECEL = 0.12;

    private boolean shooterOn = false;
    private boolean bWasPressed = false;

    private boolean yWasPressed = false;
    private boolean intakeOn = false;

    private boolean servoUp = false;
    private boolean xWasPressed = false;

    // --- Shooter speed control ---
    private static final double SHOOTER_TARGET_RPM = 4800;
    private boolean shooterReady = false;

    // --- Reversal detection ---
    private double lastForward = 0;
    private double lastRight = 0;
    private long lastDirectionChangeTime = 0;
    private static final long REVERSAL_DELAY_MS = 120;


    public void initializeHardware() {
        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        /*shooter_2 = hardwareMap.get(DcMotorEx.class, "shooter_2");*/

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        shooter_1.setDirection(DcMotorSimple.Direction.REVERSE);
        /* shooter_2.setDirection(DcMotorSimple.Direction.REVERSE);*/

        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);

        // Motor modes
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        /* shooter_2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Shooter PIDF (recommended starting values)
        shooter_1.setVelocityPIDFCoefficients(0.14, 0.0, 0.0, 11.7);
        /*shooter_2.setVelocityPIDFCoefficients(0.1, 0.0, 0.0, 11.7);*/
        intakeServo.setPower(0.0);
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        // IMU setup
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        telemetry.addLine("Hardware initialized");
    }

    protected double applySlewRate(double current, double target) {
        double delta = target - current;
        if (delta > 0) delta = Math.min(delta, MAX_ACCEL);
        else           delta = Math.max(delta, -MAX_DECEL);
        return current + delta;
    }

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

    // --- Slew rate limiting ---

    protected void handleDrive(double forward, double right) {
        telemetry.addLine("\n--- Drive Controls ---");
        telemetry.addLine("Mode: Robot-Relative");

        //double forward = gamepad1.left_stick_y; // Shooter side is forward
        //double right = -gamepad1.left_stick_x;
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

    private double getRpm(DcMotorEx motor) {
        return (motor.getVelocity() * 60) / TICKS_PER_REVOLUTION;
    }

    protected void handleMechanisms() {

        // --- Intake toggle (X button) ---
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn; // Toggle intake
        }
        xWasPressed = gamepad1.x;

        // --- Shooter toggle (B button) ---
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn; // Toggle shooter
        }
        bWasPressed = gamepad1.b;

        // --- Blocker/servo toggle (Y button) ---
        if (gamepad1.y && !yWasPressed) {
            servoUp = !servoUp; // Toggle servo ON/OFF
        }
        yWasPressed = gamepad1.y;

        // --- Set CRServo power (ON/OFF) ---
        intakeServo.setPower(servoUp ? 1.0 : 0.0);

        // --- Shooter control ---
        // --- Shooter control ---
        double targetVelocity =
                (SHOOTER_TARGET_RPM / 60.0) * TICKS_PER_REVOLUTION;

        if (shooterOn) {
            shooter_1.setVelocity(targetVelocity);

            double rpm = getRpm(shooter_1);
            shooterReady = Math.abs(rpm - SHOOTER_TARGET_RPM) < 75;

            telemetry.addData("Shooter RPM", rpm);
            telemetry.addData("Shooter Ready", shooterReady);
        } else {
            shooter_1.setVelocity(0);
            shooterReady = false;
        }

        // --- Intake motor control ---
        intakeTransfer.setPower(intakeOn ? 1.0 : 0.0);

        // --- Telemetry ---
        telemetry.addLine(servoUp ? "Gate: ON" : "Gate: OFF");
        telemetry.addLine(intakeOn ? "Intake: ON" : "Intake: OFF");
        telemetry.addLine(shooterOn ? "Shooter: ON" : "Shooter: OFF");
    }

}
