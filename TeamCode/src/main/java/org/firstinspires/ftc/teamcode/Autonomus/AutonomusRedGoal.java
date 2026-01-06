package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
//RIGHT ONE

@Autonomous(name = "Autonomous : Red Goal")
public class AutonomusRedGoal extends LinearOpMode {

    // Drive motors
    private DcMotor flmotor, frmotor, blmotor, brmotor;

    // Shooter motors (same as TeleOp)
    private DcMotorEx shooter_1 ;/*, shooter_2;*/

    // Intake + transfer motors (same as TeleOp)
    private DcMotor intakeTransfer;

    // Servo (same name as TeleOp)
    private CRServo intakeServo;

    // Constants
    private static final double COUNTS_PER_MOTOR_REV = 537.7;
    private static final double DRIVE_GEAR_REDUCTION = 1.0;
    private static final double WHEEL_DIAMETER_INCHES = 4.25;

    private static final double HALF_OF_BOT_LENGTH = 8.5;
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
        moveForward((46 + HALF_OF_BOT_LENGTH), 0.5);



        // 3) Spin up shooter and fire

        /*intakeServo.setPosition(0.8);
        shooter_1.setPower(0.5);
        shooter_2.setPower(0.5);
        sleep(3000);
        intakeTransfer.setPower(1);
        intakeServo.setPosition(0.38);
        sleep(10000);*/


        rotate(-147, TURN_SPEED);
        sleep(100);
        intakeTransfer.setPower(1.0);

        moveForward((44),0.23);
        sleep(200);
        intakeTransfer.setPower(0.0);

        moveForward((-44), DRIVE_SPEED);
        rotate(147, TURN_SPEED);
        shooter_1.setVelocity(2100);
        sleep(5000);
        moveForward(-16,DRIVE_SPEED);
        intakeServo.setPower(1.0);
        sleep(2000);
        intakeTransfer.setPower(0.8);
        sleep(8000);


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

        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");
       // shooter_2 = hardwareMap.get(DcMotor.class, "shooter_2");

        intakeTransfer = hardwareMap.get(DcMotor.class, "intakeTransfer");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Directions (same as TeleOp)
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTransfer.setDirection(DcMotor.Direction.REVERSE);
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Drive encoders
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //shooter_2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Default servo position: OPEN
        intakeServo.setPower(0.0);
        shooter_1.setVelocityPIDFCoefficients(0.8, 0.0, 0.0, 15.0);

    }

    // ============================================================
    // INTAKE LOGIC (TeleOp Logic)
    // ============================================================
    private void activateIntake(boolean on) {
        if (on) {
            intakeTransfer.setPower(TRANSFER_POWER);
            intakeServo.setPower(0.0);  // servo OPEN for intake
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
               // shooter_2.setPower(SHOOTER_FULL_POWER);
            }

            intakeTransfer.setPower(TRANSFER_POWER);
            intakeServo.setPower(1.0); // servo CLOSED for shooting
        }
        else {
            shooter_1.setPower(0);
           // shooter_2.setPower(0);
            intakeTransfer.setPower(0);
            intakeServo.setPower(0.0); // open when idle
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
        final double TURN_DIAMETER_INCHES = 23.5;
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
        intakeServo.setPower(0.0);
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
