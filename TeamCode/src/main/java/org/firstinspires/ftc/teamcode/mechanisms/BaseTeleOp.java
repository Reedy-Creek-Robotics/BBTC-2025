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
    protected double limitedForward = 0, limitedRight = 0, limitedRotate = 0;

    protected Camera camera;
    private IMU imu;

    protected static final double MAX_ACCEL = 0.08, MAX_DECEL = 0.12;

    private boolean shooterOn = false;
    private boolean lastShooterOn = false;
    private boolean bWasPressed = false;

    private boolean intakeOn = false;
    private boolean xWasPressed = false;

    private boolean servoOn = false;
    //private boolean cameraOn = false;
//    private boolean yWasPressed = false;

//    private boolean lastYWasPressed = false;

    private boolean aWasPressed = false;

    private boolean servoAllowed = false;

    private double shootervelocity = 1900;

    private double lastForward = 0;
    private double lastRight = 0;
    private long lastDirectionChangeTime = 0;
    private static final long REVERSAL_DELAY_MS = 120;
    private static final double SERVO_START_TICKS = 1050;

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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        shooter_1.setVelocityPIDFCoefficients(1.5, 0.0, 1.2, 15.0);

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
        double rotate = gamepad1.right_stick_x;
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

        // --- Intake toggle (X button) ---
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn;
        }
        xWasPressed = gamepad1.x;

        // --- Camera toggle (Y button) ---
       /* if (gamepad1.y && !yWasPressed) {
            cameraOn = !cameraOn;
        }
        yWasPressed = gamepad1.y;*/

        // --- Shooter toggle (B button) ---
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;
        }
        bWasPressed = gamepad1.b;

        if(servoOn){
            intakeServo.setPower(1);
            servoAllowed = false;
        } else {
            intakeServo.setPower(0);
        }

        double tps = 0;
        double rpm = 0;

        if (shooterOn) {
            /*if (yWasPressed && !lastYWasPressed) {
                shootervelocity += 100;
            }
            yWasPressed = gamepad1.y;
            lastYWasPressed = yWasPressed;*/
            shooter_1.setVelocity(shootervelocity);

            if (gamepad1.a && !aWasPressed) {
                servoOn = !servoOn;
            }
            aWasPressed = gamepad1.a;

            tps = shooter_1.getVelocity();
            rpm = (tps * 60) / 28;
            //rpm reached or manual override
            if (!servoAllowed && tps > SERVO_START_TICKS || aWasPressed) {
                servoAllowed = true;
            } else if (servoAllowed && tps < SERVO_START_TICKS) {
                servoAllowed = false;
            }
        } else {
            shooter_1.setVelocity(0);
            servoOn = false;
            servoAllowed = false;
            aWasPressed = false;
            // FORCE intake OFF when shooter turns OFF
            if (lastShooterOn) {
                intakeOn = false;
            }
        }
        lastShooterOn = shooterOn;

      /*  if (cameraOn) {
            camera.enable();
        } else {
            camera.disable();
        }*/

//        double distance = camera.getDistance();
//        double tagid = camera.getTid();

        intakeServo.setPower(servoAllowed ? 1.0 : 0.0);
        intakeTransfer.setPower(intakeOn ? 1.0 : 0.0);

        telemetry.addData("Shooter ON", shooterOn ? "YES" : "NO");
        telemetry.addData("Raw ticks/sec", tps);
        telemetry.addData("RPM", rpm);
        telemetry.addData("Intake ON", intakeOn ? "YES" : "NO");
        telemetry.addData("Servo Allowed", servoAllowed ? "YES" : "NO");
//        telemetry.addData("Distance", distance);
//        telemetry.addData("Tag ID", tagid);

        telemetry.update();
    }
}
