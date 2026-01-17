package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public abstract class BaseAutonomus extends LinearOpMode {
    protected DcMotor flmotor, frmotor, blmotor, brmotor;

    // Shooter motors (same as TeleOp)
    protected DcMotorEx shooter_1 ;

    // Intake + transfer motors (same as TeleOp)
    protected DcMotor intakeTransfer;

    // Servo (same name as TeleOp)
    protected CRServo intakeServo;

    // Constants
    protected static final double COUNTS_PER_MOTOR_REV = 537.7;
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;
    protected static final double WHEEL_DIAMETER_INCHES = 4.25;

    protected static final double HALF_OF_BOT_LENGTH = 8.5;
    protected static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * Math.PI);

    protected static final double SHOOTER_TPS = 810;
    protected static final double DRIVE_SPEED = 0.9;
    protected static final double TURN_SPEED = 0.4;


    protected void initializeHardware() {

        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");

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
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Default servo position: OPEN
        intakeServo.setPower(0.0);
        //PIDF values tuned for 810 TPS and shooting distance of ~45inches
        shooter_1.setVelocityPIDFCoefficients(16, 0.0, 0.0, 25.5);
    }

    // ============================================================
    // MOVEMENT CODE (unchanged)
    // ============================================================
    protected void moveForward(double inches, double speed) {
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

    protected void rotate(double degrees, double speed) {
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

    protected void stopDrive() {
        setDrivePower(0);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sleep(250);
    }

    protected void stopAll() {
        intakeTransfer.setPower(0);
        shooter_1.setPower(0);
        intakeServo.setPower(0.0);
        stopDrive();
    }
    protected void stopShootSequence(){
        intakeTransfer.setPower(0);
        shooter_1.setVelocity(0);
        intakeServo.setPower(0.0);

    }

    protected void setDriveMotorMode(DcMotor.RunMode mode) {
        flmotor.setMode(mode);
        frmotor.setMode(mode);
        blmotor.setMode(mode);
        brmotor.setMode(mode);
    }

    protected void setDrivePower(double power) {
        flmotor.setPower(power);
        frmotor.setPower(power);
        blmotor.setPower(power);
        brmotor.setPower(power);
    }
}
