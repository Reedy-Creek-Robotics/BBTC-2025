package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous 1: Move and Shoot")
public class AutonomusRedwall extends LinearOpMode {
    // Drive motors
    private DcMotor flmotor, frmotor, blmotor, brmotor;

    // Shooter motors (twin wheels)
    private DcMotor leftShooterWheel, rightShooterWheel;

    // Additional mechanism motors
    private DcMotor intake, transfer;

    // Servo
    private Servo intakeServo;

    // Constants for movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // REV HD Hex
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double SHOOTER_POWER = 0.85; // Power for shooter wheels
    private static final double MECHANISM_POWER = 0.1; // Intake/Transfer motor speed

    @Override
    public void runOpMode() {
        // Initialize hardware
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        leftShooterWheel = hardwareMap.get(DcMotor.class, "leftShooterWheel");
        rightShooterWheel = hardwareMap.get(DcMotor.class, "rightShooterWheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);
        leftShooterWheel.setDirection(DcMotor.Direction.FORWARD);
        rightShooterWheel.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        transfer.setDirection(DcMotor.Direction.FORWARD);

        // Motor modes
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized and ready to run");
        telemetry.update();

        waitForStart();

        // Step 1: Move forward by 50 inches
        activateIntake(true); // Turn on intake and transfer, servo to position 1
        moveForward(50, DRIVE_SPEED);
        activateIntake(false); // Stop intake/transfer, servo to 0

        // Step 2: Rotate 180 degrees
        rotate(180, TURN_SPEED);

        // Step 3: Spin up shooter wheels and shoot
        activateShooter(true); // Set shooter and transfer/servo as needed
        shoot();
        activateShooter(false); // Turn off shooter and transfer, servo to 0
    }

    // Intake and transfer motor + servo helper
    private void activateIntake(boolean on) {
        if (on) {
            intake.setPower(MECHANISM_POWER);
            transfer.setPower(MECHANISM_POWER);
            intakeServo.setPosition(1.0); // Extend servo
        } else {
            intake.setPower(0);
            transfer.setPower(0);
            intakeServo.setPosition(0.0); // Retract servo
        }
    }

    // Shooter helper with transfer and servo
    private void activateShooter(boolean on) {
        if (on) {
            leftShooterWheel.setPower(SHOOTER_POWER);
            rightShooterWheel.setPower(SHOOTER_POWER);
            transfer.setPower(MECHANISM_POWER);
            intakeServo.setPosition(0.0); // Retract servo when shooting
        } else {
            leftShooterWheel.setPower(0);
            rightShooterWheel.setPower(0);
            transfer.setPower(0);
            intakeServo.setPosition(0.0);
        }
    }

    private void rotate(double degrees, double speed) {
        final double TURN_DIAMETER_INCHES = 15.0;
        double inchesToTurn = (degrees / 360.0) * (TURN_DIAMETER_INCHES * 3.1415);
        int target = (int)(inchesToTurn * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Right turn
        flmotor.setTargetPosition(target);
        blmotor.setTargetPosition(target);
        frmotor.setTargetPosition(-target);
        brmotor.setTargetPosition(-target);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(speed);

        while (opModeIsActive() && flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()) {
            telemetry.addData("Path", "Rotating %d degrees", (int)degrees);
            telemetry.update();
        }

        stopAndResetDrive();
    }

    private void moveForward(double inches, double speed) {
        int target = (int)(inches * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flmotor.setTargetPosition(target);
        frmotor.setTargetPosition(target);
        blmotor.setTargetPosition(target);
        brmotor.setTargetPosition(target);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        setDrivePower(speed);

        while (opModeIsActive() && flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()) {
            telemetry.addData("Path", "Moving forward %f inches", inches);
            telemetry.update();
        }

        stopAndResetDrive();
    }

    private void shoot() {
        telemetry.addData("Status", "Shooting!");
        telemetry.update();
        sleep(1500); // Wait for wheels to spin up
        // Could add a servo to feed rings here
        sleep(1000); // Simulated shoot time
    }

    private void setDriveMotorMode(DcMotor.RunMode mode) {
        flmotor.setMode(mode);
        frmotor.setMode(mode);
        blmotor.setMode(mode);
        brmotor.setMode(mode);
    }

    private void setDrivePower(double power) {
        flmotor.setPower(power);
        frmotor.setPower(power);
        blmotor.setPower(power);
        brmotor.setPower(power);
    }

    private void stopAndResetDrive() {
        setDrivePower(0);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }
}
