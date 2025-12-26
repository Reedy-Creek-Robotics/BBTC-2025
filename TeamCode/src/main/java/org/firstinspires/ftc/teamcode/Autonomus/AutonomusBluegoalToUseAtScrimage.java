package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Autonomous 1: Move and Shoot (Updated Logic blue)")
public class AutonomusBluegoalToUseAtScrimage extends LinearOpMode {

    // Drive motors
    private DcMotor flmotor, frmotor, blmotor, brmotor;

    // Shooter motors (same as TeleOp)
    private DcMotor shooter_1;/*, shooter_2;*/

    // Intake + transfer motors (same as TeleOp)
    private DcMotor intakeTransfer;

    // Servo (same name as TeleOp)
    private Servo intakeServo;

    // Constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.25;
    private static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    private static final double DRIVE_SPEED = 0.5;
    private static final double TURN_SPEED = 0.4;

    // Shooter & intake powers (same ranges as TeleOp)
    private static final double SHOOTER_FULL_POWER = 0.5;
    private static final double SHOOTER_HALF_POWER = 0.3;
    private static final double TRANSFER_POWER = 0.4;

    @Override
    public void runOpMode() {

        initializeHardware();

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS STEPS
        // -------------------------------

        // 1) Move forward 50 inches while intake is ON
        activateIntake(false);

        moveForward(63, DRIVE_SPEED);



        // 3) Spin up shooter and fire

        /*intakeServo.setPosition(0.8);
        shooter_1.setPower(0.5);
        shooter_2.setPower(0.5);
        sleep(3000);
        intakeTransfer.setPower(1);
        intakeServo.setPosition(0.38);
        sleep(10000);*/



        rotate(90, TURN_SPEED);



        moveForward(35,DRIVE_SPEED);



        // End
        stopAll();
    }

    // ============================================================
    // HARDWARE INITIALIZATION
    // ============================================================
    private void initializeHardware() {

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        shooter_1 = hardwareMap.get(DcMotor.class, "shooter_1");
       // shooter_2 = hardwareMap.get(DcMotor.class, "shooter_2");

        intakeTransfer = hardwareMap.get(DcMotor.class, "intakeTransfer");

        intakeServo = hardwareMap.get(Servo.class, "intakeServo");

        // Directions (same as TeleOp)
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        shooter_1.setDirection(DcMotor.Direction.FORWARD);
        //shooter_2.setDirection(DcMotor.Direction.REVERSE);
        intakeTransfer.setDirection(DcMotor.Direction.FORWARD);

        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Drive encoders
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
       // shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Default servo position: OPEN
        intakeServo.setPosition(1.0);
    }

    // ============================================================
    // INTAKE LOGIC (TeleOp Logic)
    // ============================================================
    private void activateIntake(boolean on) {
        if (on) {
            intakeTransfer.setPower(TRANSFER_POWER);
            intakeServo.setPosition(0.4);  // servo OPEN for intake
        }
    }

    // ============================================================
    // SHOOTER LOGIC (TeleOp Logic)
    // ============================================================
    private void activateShooter(boolean on, boolean halfPower) {

        if (on) {
            if (halfPower) {
                shooter_1.setPower(SHOOTER_HALF_POWER);
                //shooter_2.setPower(SHOOTER_HALF_POWER);
            } else {
                shooter_1.setPower(SHOOTER_FULL_POWER);
              //  shooter_2.setPower(SHOOTER_FULL_POWER);
            }

            intakeTransfer.setPower(TRANSFER_POWER);
            intakeServo.setPosition(0.0); // servo CLOSED for shooting
        }
        else {
            shooter_1.setPower(0);
           // shooter_2.setPower(0);
            intakeTransfer.setPower(0);
            intakeServo.setPosition(1.0); // open when idle
        }
    }

    // ============================================================
    // CONTROLLED SHOOT SEQUENCE
    // ============================================================
    private void autonomousShootSequence() {

        // 1) Spin up shooter
        activateShooter(true, false); // full power
        sleep(1500);

        // 2) Feed for 1 second
        intakeTransfer.setPower(TRANSFER_POWER);
        intakeServo.setPosition(0.8);

        sleep(3000);

        // 3) Shut down shooter
        activateShooter(false, false);
    }

    // ============================================================
    // MOVEMENT CODE (unchanged)
    // ============================================================
    private void moveForward(double inches, double speed) {
        int target = (int)(inches * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flmotor.setTargetPosition(target);
        frmotor.setTargetPosition(target);
        blmotor.setTargetPosition(target);
        brmotor.setTargetPosition(target);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed);

        while (opModeIsActive() &&
                flmotor.isBusy() && frmotor.isBusy() &&
                blmotor.isBusy() && brmotor.isBusy()) {

            telemetry.addData("Moving", "%.1f inches", inches);
            telemetry.update();
        }

        stopDrive();
    }

    private void rotate(double degrees, double speed) {
        final double TURN_DIAMETER_INCHES = 15.0;
        double inchesToTurn = (degrees / 360.0) * (TURN_DIAMETER_INCHES * Math.PI);
        int target = (int)(inchesToTurn * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flmotor.setTargetPosition(target);
        blmotor.setTargetPosition(target);
        frmotor.setTargetPosition(-target);
        brmotor.setTargetPosition(-target);

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);
        setDrivePower(speed);

        while (opModeIsActive() &&
                flmotor.isBusy() && frmotor.isBusy() &&
                blmotor.isBusy() && brmotor.isBusy()) {

            telemetry.addData("Rotating", "%.0f degrees", degrees);
            telemetry.update();
        }

        stopDrive();
    }

    private void stopDrive() {
        setDrivePower(0);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }

    private void stopAll() {
        intakeTransfer.setPower(0);
        shooter_1.setPower(0);
        //shooter_2.setPower(0);
        intakeServo.setPosition(0.8);
        stopDrive();
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
}
