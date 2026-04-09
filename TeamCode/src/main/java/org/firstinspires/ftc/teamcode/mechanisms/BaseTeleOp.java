package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants.ShotType;
import org.firstinspires.ftc.teamcode.statemachines.IntakeStateMachine;
import org.firstinspires.ftc.teamcode.statemachines.LEDController;
import org.firstinspires.ftc.teamcode.statemachines.ShooterStateMachine;

/**
 * Base class for TeleOp modes. Provides:
 * - Mecanum drive with slew-rate limiting and reversal braking
 * - State-machine-based shooter, intake, and LED control
 * - Threaded camera processing (50Hz on background thread)
 *
 * Subclasses implement:
 *   configureCameraPipeline() — set the appropriate Limelight pipeline
 *   isTargetLocked()          — check whether the camera sees a valid target
 */
public abstract class BaseTeleOp extends LinearOpMode {

    // -------------------- Hardware --------------------
    protected DcMotor flmotor, frmotor, blmotor, brmotor;
    protected DcMotorEx shooter_1, intakeTransfer;
    protected CRServo intakeServo;
    protected Servo led;
    private IMU imu;

    // -------------------- State Machines --------------------
    protected ShooterStateMachine shooterSM;
    protected IntakeStateMachine  intakeSM;
    protected LEDController       ledController;

    // -------------------- Threaded Camera --------------------
    protected ThreadedCamera threadedCamera;
    private Camera camera;

    // -------------------- Drive State --------------------
    private double limitedForward = 0, limitedRight = 0, limitedRotate = 0;
    private double lastForward = 0, lastRight = 0;
    private long lastDirectionChangeTime = 0;

    // -------------------- Mechanism State --------------------
    private boolean shooterOn = false;
    private ShotType selectedShotType = ShotType.NONE;

    // Button edge detection
    private boolean xWasPressed = false;
    private boolean aWasPressed = false;
    private boolean bWasPressed = false;
    private boolean yWasPressed = false;

    // ================================================================
    //  INITIALIZATION
    // ================================================================

    public void initializeHardware() {
        // --- Drive motors ---
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Shooter ---
        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // --- Intake ---
        intakeTransfer = hardwareMap.get(DcMotorEx.class, "intakeTransfer");
        intakeTransfer.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeServo.setPower(0.0);

        // --- LED ---
        led = hardwareMap.get(Servo.class, "led");

        // --- Camera ---
        camera = new Camera(hardwareMap);
        threadedCamera = new ThreadedCamera(camera);

        // --- IMU ---
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));

        // --- State Machines ---
        shooterSM     = new ShooterStateMachine(shooter_1);
        intakeSM      = new IntakeStateMachine(intakeTransfer, intakeServo);
        ledController = new LEDController(led);

        telemetry.addLine("Hardware initialized");
    }

    // ================================================================
    //  ABSTRACT METHODS — Subclasses must implement
    // ================================================================

    /** Set the Limelight pipeline (0=Blue, 1=Red, 2=Off). */
    protected abstract void configureCameraPipeline();

    /** Return true if the camera currently sees a valid target for shooting. */
    protected abstract boolean isTargetLocked();

    // ================================================================
    //  DRIVE — Slew-rate limited mecanum with reversal braking
    // ================================================================

    private double applySlewRate(double current, double target) {
        double delta = target - current;
        if (delta > 0) delta = Math.min(delta, DriveConstants.MAX_ACCEL);
        else           delta = Math.max(delta, -DriveConstants.MAX_DECEL);
        return current + delta;
    }

    private void drive(double forward, double right, double rotate) {
        double fl = forward + right + rotate;
        double fr = forward - right - rotate;
        double bl = forward - right + rotate;
        double br = forward + right - rotate;

        double maxPower = Math.max(1.0, Math.max(
                Math.max(Math.abs(fl), Math.abs(fr)),
                Math.max(Math.abs(bl), Math.abs(br))));

        flmotor.setPower(fl / maxPower);
        frmotor.setPower(fr / maxPower);
        blmotor.setPower(bl / maxPower);
        brmotor.setPower(br / maxPower);
    }

    protected void handleDrive(double forward, double right) {
        double rotate = -gamepad1.right_stick_x;
        long now = System.currentTimeMillis();

        boolean forwardReversal =
                (Math.signum(forward) == -Math.signum(lastForward))
                        && Math.abs(forward) > 0.3
                        && Math.abs(lastForward) > 0.3;
        boolean strafeReversal =
                (Math.signum(right) == -Math.signum(lastRight))
                        && Math.abs(right) > 0.3
                        && Math.abs(lastRight) > 0.3;

        if (forwardReversal || strafeReversal) lastDirectionChangeTime = now;

        if (now - lastDirectionChangeTime < DriveConstants.REVERSAL_DELAY_MS) {
            forward = 0;
            right   = 0;
        }

        limitedForward = applySlewRate(limitedForward, forward);
        limitedRight   = applySlewRate(limitedRight, right);
        limitedRotate  = applySlewRate(limitedRotate, rotate);

        lastForward = forward;
        lastRight   = right;

        drive(limitedForward, limitedRight, limitedRotate);
    }

    // ================================================================
    //  MECHANISM INPUT — reads gamepad buttons, drives state transitions
    // ================================================================

    protected void handleMechanismInputs() {
        // --- Gamepad 1 ---

        // X button: toggle intake (only when shooter is off)
        if (gamepad1.x && !xWasPressed && !shooterOn) {
            intakeSM.toggle(IntakeStateMachine.State.RUNNING);
        }
        xWasPressed = gamepad1.x;

        // A button: toggle reverse intake (only when shooter is off)
        if (gamepad1.a && !aWasPressed && !shooterOn) {
            intakeSM.toggle(IntakeStateMachine.State.REVERSE);
        }
        aWasPressed = gamepad1.a;

        // B button: toggle shooter on / off
        if (gamepad1.b && !bWasPressed) {
            shooterOn = !shooterOn;
            if (shooterOn) {
                // Default to SHORT if no shot type selected yet
                if (selectedShotType == ShotType.NONE) {
                    selectedShotType = ShotType.SHORT;
                }
                shooterSM.start(selectedShotType);
            } else {
                // Shooter off → stop everything, reset shot type
                shooterSM.stop();
                intakeSM.stop();
                selectedShotType = ShotType.NONE;
            }
        }
        bWasPressed = gamepad1.b;

        // Y button: toggle feeding (only when shooter is on)
        if (gamepad1.y && !yWasPressed && shooterOn) {
            intakeSM.toggle(IntakeStateMachine.State.FEEDING);
        }
        yWasPressed = gamepad1.y;

        // --- Gamepad 2: Shot Type Selection ---

        if (gamepad2.y) {
            selectedShotType = ShotType.LONG;
            if (shooterOn) shooterSM.start(selectedShotType);
            telemetry.addLine("LONG SHOT");
        }
        if (gamepad2.x) {
            selectedShotType = ShotType.MID;
            if (shooterOn) shooterSM.start(selectedShotType);
            telemetry.addLine("MID SHOT");
        }
        if (gamepad2.a) {
            selectedShotType = ShotType.SHORT;
            if (shooterOn) shooterSM.start(selectedShotType);
            telemetry.addLine("SHORT SHOT");
        }
        if (gamepad2.b) {
            selectedShotType = ShotType.EMERGENCY;
            if (shooterOn) shooterSM.start(selectedShotType);
            threadedCamera.setPipelineUseless();
            telemetry.addLine("EMERGENCY SHOT");
        }
        if (gamepad2.right_bumper || gamepad2.left_bumper) {
            threadedCamera.setPipelineUseless();
            telemetry.addLine("Camera Off");
        }
    }

    // ================================================================
    //  STATE MACHINE UPDATES — call every loop, never blocks
    // ================================================================

    protected void updateMechanisms() {
        shooterSM.update();
        intakeSM.update();

        ledController.setTargetLocked(isTargetLocked());
        ledController.update();
    }

    // ================================================================
    //  TELEMETRY
    // ================================================================

    protected void addTelemetry() {
        telemetry.addData("Shooter",   shooterSM.getState());
        telemetry.addData("Shot Type", selectedShotType);
        telemetry.addData("Intake",    intakeSM.getState());
        telemetry.addData("Target Tx", "%.2f", threadedCamera.getTx());
        telemetry.addData("Dist",      "%.1f", threadedCamera.getDistance());

        if (shooterSM.isRunning()) {
            telemetry.addData("Shooter Vel", "%.0f / %.0f",
                    shooterSM.getCurrentVelocity(),
                    shooterSM.getTargetVelocity());
        }

        telemetry.update();
    }

    // ================================================================
    //  HELPERS
    // ================================================================

    /** Accessor for subclasses that need the shooter state. */
    protected boolean isShooterOn() {
        return shooterOn;
    }

    /** Accessor for subclasses that need the selected shot type. */
    protected ShotType getSelectedShotType() {
        return selectedShotType;
    }
}
