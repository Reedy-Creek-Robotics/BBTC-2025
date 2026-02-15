package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

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

    protected static final double SHOOTER_TPS = 1000;
    protected static final double DRIVE_SPEED = 0.9;
    protected static final double TURN_SPEED = 0.4;

    protected static final double INTAKE_SPEED = 0.5;


    protected void initializeHardware() {


        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");

        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");

        intakeTransfer = hardwareMap.get(DcMotor.class, "intakeTransfer");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        // Directions (same as TeleOp)
        flmotor.setDirection(DcMotor.Direction.FORWARD);
        blmotor.setDirection(DcMotor.Direction.FORWARD);
        frmotor.setDirection(DcMotor.Direction.REVERSE);
        brmotor.setDirection(DcMotor.Direction.REVERSE);

        shooter_1.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeTransfer.setDirection(DcMotor.Direction.REVERSE);
        intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);

        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flmotor.setPower(0);
        blmotor.setPower(0);
        frmotor.setPower(0);
        brmotor.setPower(0);


        // Drive encoders
        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        shooter_1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeTransfer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Default servo position: OPEN
        intakeServo.setPower(0.0);
        //PIDF values tuned for 810 TPS and shooting distance of ~45inches
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

        // This loop runs while the robot is moving
        while (opModeIsActive() && flmotor.isBusy()) {
            // 1. Calculate how many ticks are left to go
            int remainingTicks = Math.abs(target - flmotor.getCurrentPosition());

            // 2. Convert ticks back to inches to make it easier to think about
            double inchesLeft = remainingTicks / COUNTS_PER_INCH;

            double currentPower = speed;

            // 3. RAMP DOWN: If we are within 5 inches, start slowing down
            if (inchesLeft < 20.0) {
                // Scale power linearly from 'speed' down to 0.15
                currentPower = Range.clip((inchesLeft / 15.0) * speed, 0.15, speed);
            }

            setDrivePower(currentPower);

            telemetry.addData("Inches Left", "%.2f", inchesLeft);
            telemetry.update();
        }

        stopDrive(); // This calls your sleep(250) which helps settle the bot
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
    protected void farShot(){
        shooter_1.setVelocityPIDFCoefficients(75, 0.0, 0.0, 6.5);
        shooter_1.setVelocity(1000);
    }
    protected void closeShot(){
        shooter_1.setVelocityPIDFCoefficients(28, 0.0, 0, 10.5);
        shooter_1.setVelocity(900);
    }

    // Inside your moveForward method logic:

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
