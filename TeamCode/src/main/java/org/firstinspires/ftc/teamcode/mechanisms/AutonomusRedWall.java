package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "AutonomousBlueWall 1: Move and Shoot")
public class AutonomusRedWall extends LinearOpMode {
    // Drive motors
    private DcMotor flmotor;
    private DcMotor frmotor;
    private DcMotor blmotor;
    private DcMotor brmotor;

    // Shooter motors (twin wheels)
    private DcMotor leftShooterWheel;
    private DcMotor rightShooterWheel;

    // Constants for movement
    private static final double COUNTS_PER_MOTOR_REV = 537.7; // Example for a REV HD Hex Motor
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * 3.1415);
    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;
    private static final double SHOOTER_POWER = 0.85; // Power for shooter wheels

    @Override
    public void runOpMode() {
        // Initialize hardware
        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        leftShooterWheel = hardwareMap.get(DcMotor.class, "leftShooterWheel");
        rightShooterWheel = hardwareMap.get(DcMotor.class, "rightShooterWheel");

        // Set motor directions
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Shooter wheels often spin opposite directions
        leftShooterWheel.setDirection(DcMotor.Direction.FORWARD);
        rightShooterWheel.setDirection(DcMotor.Direction.REVERSE);

        // Reset encoders and set to run with encoders
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized and ready to run");
        telemetry.update();

        waitForStart();

        // Step 1: Move forward by 77 inches
        moveForward(77, DRIVE_SPEED);

        // Step 2: Rotate 45 degrees
        rotate(315, TURN_SPEED);

        // Step 3: Spin up shooter wheels and shoot
        shoot();
    }

    private void rotate(double degrees, double speed) {
        // Assuming a robot turning circle diameter of 15 inches, adjust as needed.
        final double TURN_DIAMETER_INCHES = 15.0;
        double inchesToTurn = (degrees / 360.0) * (TURN_DIAMETER_INCHES * 3.1415);
        int target = (int)(inchesToTurn * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // For a right turn
        flmotor.setTargetPosition(target);
        blmotor.setTargetPosition(target);
        frmotor.setTargetPosition(-target);
        brmotor.setTargetPosition(-target);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        flmotor.setPower(speed);
        frmotor.setPower(speed);
        blmotor.setPower(speed);
        brmotor.setPower(speed);

        while (opModeIsActive() && flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()) {
            telemetry.addData("Path", "Rotating %d degrees", (int)degrees);
            telemetry.update();
        }

        // Stop and reset
        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);

        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }

    private void moveForward(double inches, double speed) {
        int target = (int)(inches * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flmotor.setTargetPosition(target);
        frmotor.setTargetPosition(target);
        blmotor.setTargetPosition(target);
        brmotor.setTargetPosition(target);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        flmotor.setPower(speed);
        frmotor.setPower(speed);
        blmotor.setPower(speed);
        brmotor.setPower(speed);

        while (opModeIsActive() && flmotor.isBusy() && frmotor.isBusy() && blmotor.isBusy() && brmotor.isBusy()) {
            telemetry.addData("Path", "Moving forward %d inches", inches);
            telemetry.addData("Target", "Running to %d", target);
            telemetry.addData("Current Position", "fl:%d fr:%d bl:%d br:%d",
                    flmotor.getCurrentPosition(),
                    frmotor.getCurrentPosition(),
                    blmotor.getCurrentPosition(),
                    brmotor.getCurrentPosition());
            telemetry.update();
        }

        // Stop all motion
        flmotor.setPower(0);
        frmotor.setPower(0);
        blmotor.setPower(0);
        brmotor.setPower(0);

        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250); // Pause for stability
    }

    private void shoot() {
        telemetry.addData("Status", "Spinning up shooter");
        telemetry.update();

        // Spin up the shooter wheels to the desired power
        leftShooterWheel.setPower(SHOOTER_POWER);
        rightShooterWheel.setPower(SHOOTER_POWER);

        sleep(1500); // Wait for wheels to reach full speed

        // Here you would typically have another mechanism (like a servo) to push the ring into the wheels.
        // For this example, we'll  just simulate the action with a sleep.
        telemetry.addData("Status", "Shooting!");
        telemetry.update();
        sleep(1000); // Time to "shoot"

        // Stop the shooter wheels
        leftShooterWheel.setPower(0);
        rightShooterWheel.setPower(0);
        telemetry.addData("Status", "Shooting complete");
        telemetry.update();
    }

    private void setDriveMotorMode(DcMotor.RunMode mode) {
        flmotor.setMode(mode);
        frmotor.setMode(mode);
        blmotor.setMode(mode);
        brmotor.setMode(mode);
    }
}

