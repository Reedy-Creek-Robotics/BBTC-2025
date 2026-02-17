package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

public abstract class BaseTeleOp extends LinearOpMode {

    protected DcMotor flmotor, frmotor, blmotor, brmotor;
    protected DcMotorEx shooter_1, intakeTransfer;
    protected CRServo intakeServo;


    protected Camera camera;
    private boolean intakeReverseOn = false; // Added missing declaration
    private boolean aWasPressed = false; // Added missing declaration

    protected double limitedForward = 0, limitedRight = 0, limitedRotate = 0;

    private IMU imu;

    protected static final double MAX_ACCEL = 0.08, MAX_DECEL = 0.12;

    private boolean shooterOn = false;
    private boolean lastShooterOn = false;
    private boolean bWasPressed = false;

    private boolean EmergencyShootOn = false;

    private boolean intakeOn = false;
    private boolean xWasPressed = false;

    private boolean servoOn = false;
    private boolean yWasPressed = false;

    private boolean longShotOn = false;
    private boolean midShotOn = false;

    private boolean shortShotOn = false;

    private double lastForward = 0;
    private double lastRight = 0;
    //private double dist = 0;
    private long lastDirectionChangeTime = 0;
    private static final long REVERSAL_DELAY_MS = 120;
    private double tps = 0;

    public void initializeHardware() {

        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        camera = new Camera(hardwareMap);

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");


        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo.setPower(0.0);
        camera.update();
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        telemetry.addLine("Hardware initialized");
    }

    protected double applySlewRate(double current, double target) {
        double delta = target - current;
        if (delta > 0) delta = Math.min(delta, MAX_ACCEL);
        else delta = Math.max(delta, -MAX_DECEL);
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

    protected void handleDrive(double forward, double right) {
        double rotate = -gamepad1.right_stick_x;
        long now = System.currentTimeMillis();

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

        if (now - lastDirectionChangeTime < REVERSAL_DELAY_MS) {
            forward = 0;
            right = 0;
        }

        limitedForward = applySlewRate(limitedForward, forward);
        limitedRight = applySlewRate(limitedRight, right);
        limitedRotate = applySlewRate(limitedRotate, rotate);

        lastForward = forward;
        lastRight = right;

        drive(limitedForward, limitedRight, limitedRotate);
    }

    protected void handleMechanisms() {
        camera.update();

        // 1. Determine State from Inputs
        if (gamepad1.x && !xWasPressed) intakeOn = !intakeOn;
        xWasPressed = gamepad1.x;

        if (gamepad1.a && !aWasPressed) intakeReverseOn = !intakeReverseOn;
        aWasPressed = gamepad1.a;

        if (gamepad1.b && !bWasPressed) shooterOn = !shooterOn;
        bWasPressed = gamepad1.b;

        if (gamepad1.y && !yWasPressed) servoOn = !servoOn;
        yWasPressed = gamepad1.y;

        // Shot Type Selection (Gamepad 2)
        if (gamepad2.b) { EmergencyShootOn = true; longShotOn = midShotOn = shortShotOn = false; }
        if (gamepad2.y) { longShotOn = true; midShotOn = shortShotOn = EmergencyShootOn = false; }
        if (gamepad2.x) { midShotOn = true; longShotOn = shortShotOn = EmergencyShootOn = false; }
        if (gamepad2.a) { shortShotOn = true; longShotOn = midShotOn = EmergencyShootOn = false; }

        // 2. Set Shooter PIDF and Velocity (Logic only, no setPower yet)
        if (longShotOn) {
            shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);// Long shot
            tps = 1000;
            telemetry.addLine("LONG SHOT ON");
        } else if (shortShotOn) {
            shooter_1.setVelocityPIDFCoefficients(28, 0.0, 0, 10.5);// Short shot
            tps = 900;
            telemetry.addLine("SHORT SHOT ON");
        } else if (midShotOn) {
            shooter_1.setVelocityPIDFCoefficients(28,0,0,13);// Mid shot
            tps = 900;
            telemetry.addLine("MID SHOT ON");
        } else if (EmergencyShootOn) {
            shooter_1.setVelocityPIDFCoefficients(80,0,0,20);// Emergency Long shot
            tps = 1000;
            telemetry.addLine("Emergency Long Shot On");
        } else {
            tps = 900;
        }

        // 3. Final Power Calculations
        double finalShooterVel = 0;
        double finalServoPower = 0;
        double finalTransferPower = 0;

        if (shooterOn) {
            finalShooterVel = tps;
            finalServoPower = servoOn ? 1.0 : 0.0;
            // If shooter is on, transfer should probably feed it if intake is on
            finalTransferPower = intakeOn ? 1.0 : 0.0;
        } else {
            finalShooterVel = 0;
            servoOn = false;
            if (lastShooterOn) {
                intakeOn = false;
                shortShotOn = midShotOn = longShotOn = EmergencyShootOn = false;
            }
        }
        lastShooterOn = shooterOn;

        // Override Intake/Servo logic for Reversal or standard Intake
        // Only if the shooter isn't already "using" the servo
        if (intakeReverseOn && !shooterOn) {
            finalTransferPower = -0.3;
            finalServoPower = -0.75;
            intakeOn = false;
        } else if (intakeOn && !shooterOn) {
            finalTransferPower = 0.75;
            finalServoPower = 0.0;
        }

        // 4. Hardware Write (THE ONLY PLACE setPower/setVelocity IS CALLED)
        shooter_1.setVelocity(finalShooterVel);
        intakeServo.setPower(finalServoPower);
        intakeTransfer.setPower(finalTransferPower);

        telemetry.update();
    }
}
