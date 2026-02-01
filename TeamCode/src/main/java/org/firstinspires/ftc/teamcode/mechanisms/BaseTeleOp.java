package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;

public abstract class BaseTeleOp extends LinearOpMode {

    protected DcMotor flmotor, frmotor, blmotor, brmotor;
    protected DcMotorEx shooter_1, intakeTransfer;
    protected CRServo intakeServo;

    protected Camera camera;
    private boolean intakeReverseOn = false; // Added missing declaration
    private boolean aWasPressed = false; // Added missing declaration

    private boolean apriltagSeen = false;
    protected double limitedForward = 0, limitedRight = 0, limitedRotate = 0;

    private IMU imu;

    protected static final double MAX_ACCEL = 0.08, MAX_DECEL = 0.12;

    private boolean shooterOn = false;
    private boolean lastShooterOn = false;
    private boolean bWasPressed = false;

    private boolean intakeOn = false;
    private boolean xWasPressed = false;

    private boolean servoOn = false;
    //private boolean cameraOn = false;
    private boolean yWasPressed = false;
    private boolean dpadUpWasPressed = false;
    private boolean dpadDownWasPressed = false;
    private boolean midShotOn = true;
    private boolean longShotOn = false;
    private boolean manualShotOverrideOn = true;
    private boolean cameraOn = false;
    private double distance = -1;
    private double tagID = -1;

    private double lastForward = 0;
    private double lastRight = 0;
    private long lastDirectionChangeTime = 0;
    private static final long REVERSAL_DELAY_MS = 120;
//    private static final double SERVO_START_TICKS = 1050;

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

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        cameraOn = true;
       // shooter_1.setVelocityPIDFCoefficients(P,0.0,0.5,F);
        //PIDF values tuned for 1000 TPS and shooting distance of ~45inches
       // shooter_1.setVelocityPIDFCoefficients(900, 0.0, 0.5, 25.5);//P = 550 F = 25 // older
      //  shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.5, 25); //tps=1000 (longest shot)
        //shooter_1.setVelocityPIDFCoefficients(40, 0.0, 0, 25); //tps=900 (mid shot)
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
    protected void blueChannel(){
        camera.switchToBlue();
    }
    protected void redChannel(){
        camera.switchToRed();
    }
    protected void cameraStart(){
        camera.startCamera();
    }
    protected void handleMechanisms() {
        camera.update();

        // --- Intake toggle (X button) ---
        if (gamepad1.x && !xWasPressed) {
            intakeOn = !intakeOn;
        }
        xWasPressed = gamepad1.x;

        // Intake reversal (toggle) (A button)
        if (gamepad1.a && !aWasPressed) {
            intakeReverseOn = !intakeReverseOn;
        }
        aWasPressed = gamepad1.a;

        // --- Shooter toggle (B button) ---
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;
        }
        bWasPressed = gamepad1.b;

        // Servo toggle (Y button)
        if (gamepad1.y && !yWasPressed) {
            servoOn = !servoOn;
        }
        yWasPressed = gamepad1.y;

        if (gamepad1.dpad_up && !dpadUpWasPressed) {
            longShotOn = !longShotOn;
        }
        dpadUpWasPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownWasPressed) {
            midShotOn = !midShotOn;
        }
        dpadDownWasPressed = gamepad1.dpad_down;

        if(longShotOn){
            shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.5, 25); //tps=1000 (longest shot)
        } else {
            shooter_1.setVelocityPIDFCoefficients(40, 0.0, 0, 25); //tps=1000 (mid shot)
        }

        if (shooterOn) {
            shooter_1.setVelocity(1000);//900
            intakeServo.setPower(servoOn ? 1.0 : 0.0);
            tagID = camera.getTid();
            distance = camera.getDistance();
            apriltagSeen = (distance > 0) ? true : false;
            if (!manualShotOverrideOn) {
                if (distance > 90) {
                    shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.5, 25); //tps=1000 (longest shot)
                } else if (distance < 90) {
                    shooter_1.setVelocityPIDFCoefficients(40, 0.0, 0, 25); //tps=1000 (mid shot)
                } else {
                    apriltagSeen = false;
                }
            }
        } else {
            shooter_1.setVelocity(0);
            servoOn = false;
            yWasPressed = false;
            apriltagSeen = false;
            distance = 0;
            tagID = 0;
            // FORCE intake OFF when shooter turns OFF
            if (lastShooterOn) {
                intakeOn = false;
            }
        }
        lastShooterOn = shooterOn;

        if (intakeReverseOn) {
            if (shooterOn) {
                intakeReverseOn = false;
            } else {
                intakeTransfer.setPower(-0.5);
            }
        }
        // Priority 2: Standard Intake
        else if (intakeOn) {
            // If the shooter is on, we want more speed to feed the shooter faster
            // While collecting the balls, we want less speed to avoid first ball jamming up the shooter.
            intakeTransfer.setPower( shooterOn ? 1.0 : 0.75);
        }
        // Priority 3: Default
        else {
            intakeTransfer.setPower(0.0);
        }

        telemetry.addData("Camera Active", cameraOn);
        telemetry.addData("April Tag seen: ",apriltagSeen);
        telemetry.addData("Distance", "%.2f", distance);
        telemetry.addData("Tag ID", tagID);

        telemetry.update();
    }
}
