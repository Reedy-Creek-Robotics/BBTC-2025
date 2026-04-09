package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.InstantAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.constants.DriveConstants;
import org.firstinspires.ftc.teamcode.constants.ShooterConstants.ShotType;

public abstract class BaseAutonomus extends LinearOpMode {

    public enum StrafeDirection {
        LEFT,
        RIGHT
    }
    protected DcMotor flmotor, frmotor, blmotor, brmotor;

    // Shooter motors (same as TeleOp)
    protected DcMotorEx shooter_1 ;

    protected DistanceSensor distance1;
    protected DistanceSensor distance2;
    protected ColorSensor colorSensor;

    // Intake + transfer motors (same as TeleOp)
    protected DcMotor intakeTransfer;

    // Servo (same name as TeleOp)
    protected CRServo intakeServo;

    protected Camera camera;
    public Servo led = null;


    // Constants — now sourced from centralized DriveConstants
    protected static final double COUNTS_PER_INCH = DriveConstants.COUNTS_PER_INCH;
    protected static final double HALF_OF_BOT_LENGTH = DriveConstants.HALF_OF_BOT_LENGTH;
    protected static final double DRIVE_SPEED = DriveConstants.DRIVE_SPEED;
    protected static final double TURN_SPEED = DriveConstants.TURN_SPEED;
    protected static final double INTAKE_SPEED = DriveConstants.INTAKE_SPEED;

    protected double error = 0;
    protected double rotation = 0;



    protected void initializeHardware() {


        flmotor = hardwareMap.get(DcMotor.class, "flmotor");
        frmotor = hardwareMap.get(DcMotor.class, "frmotor");
        blmotor = hardwareMap.get(DcMotor.class, "blmotor");
        brmotor = hardwareMap.get(DcMotor.class, "brmotor");
        led=hardwareMap.get(Servo.class,"led");
        camera = new Camera(hardwareMap);


        shooter_1 = hardwareMap.get(DcMotorEx.class, "shooter_1");

        intakeTransfer = hardwareMap.get(DcMotor.class, "intakeTransfer");

        intakeServo = hardwareMap.get(CRServo.class, "intakeServo");

        distance1 = hardwareMap.get(DistanceSensor.class, "distance1");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        // Directions — matched to TeleOp (FL/BL reversed, FR/BR forward)
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
        camera.update();

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
    protected double turnCorrectionBlue(){
        camera.update();
        error = -(camera.getTx());
        rotation = -2.8 - error;
        if(error <= 0 ){
            rotate(rotation,0.2);
        } else if(error > 0){
            rotate(-rotation,0.2);
            rotation = -rotation;
        }

        camera.update();
        error = -(camera.getTx());

        if((0 >= error) && (error >= -5) ){
            led.setPosition(0.841);
        }

        stopDrive();
        return rotation;
    }
    protected double turnCorrectionRed(){
        camera.update();
        error = -(camera.getTx());
        rotation = 2.8 - error;//2.2
        if(error <= 0 ){
            rotate(rotation,0.2);
        } else if(error > 0){
            rotate(rotation,0.2);
        }

        camera.update();
        error = -(camera.getTx());

        if((0 <= error) && (error <= 5) ){
            led.setPosition(0.841);
        }

        stopDrive();
        return rotation;
    }

    protected void strafe(double inches, double speed, StrafeDirection direction) {

        int target = (int)(inches * COUNTS_PER_INCH);

        setDriveMotorMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        if (direction == StrafeDirection.RIGHT) {

            // Strafe Right
            flmotor.setTargetPosition(target);
            blmotor.setTargetPosition(-target);
            frmotor.setTargetPosition(-target);
            brmotor.setTargetPosition(target);

        } else { // LEFT

            // Strafe Left
            flmotor.setTargetPosition(-target);
            blmotor.setTargetPosition(target);
            frmotor.setTargetPosition(target);
            brmotor.setTargetPosition(-target);
        }

        setDriveMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                flmotor.isBusy() &&
                frmotor.isBusy() &&
                blmotor.isBusy() &&
                brmotor.isBusy()) {

            int remainingTicks = Math.abs(target - flmotor.getCurrentPosition());
            double inchesLeft = remainingTicks / COUNTS_PER_INCH;

            double currentPower = speed;

            // Smooth ramp down
            if (inchesLeft < 20.0) {
                currentPower = Range.clip((inchesLeft / 15.0) * speed, 0.2, speed);
            }

            setDrivePower(currentPower);

            telemetry.addData("Strafing", direction);
            telemetry.addData("Inches Left", "%.2f", inchesLeft);
            telemetry.update();
        }

        stopDrive();
    }


    protected void rotate(double degrees, double speed) {
        double inchesToTurn = (degrees / 360.0) * (DriveConstants.TURN_DIAMETER_INCHES * Math.PI);
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
    protected void farShot() {
        ShotType s = ShotType.LONG;
        shooter_1.setVelocityPIDFCoefficients(s.p, s.i, s.d, s.f);
        shooter_1.setVelocity(s.tps);
    }

    protected void closeShot() {
        ShotType s = ShotType.SHORT;
        shooter_1.setVelocityPIDFCoefficients(s.p, s.i, s.d, s.f);
        shooter_1.setVelocity(s.tps);
    }

    protected void veryCloseShot() {
        ShotType s = ShotType.MID;
        shooter_1.setVelocityPIDFCoefficients(s.p, s.i, s.d, s.f);
        shooter_1.setVelocity(s.tps);
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
        led.setPosition(0);
        stopDrive();
    }
    protected void stopShootSequence(){
        intakeTransfer.setPower(0);
        shooter_1.setVelocity(0);
        intakeServo.setPower(0.0);
        led.setPosition(0);
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
    protected void intakeOnUntilDetected() {
        intakeTransfer.setPower(0);
        intakeServo.setPower(0);
        sleep(100);

        if (distance1.getDistance(DistanceUnit.INCH) < 2) {
            intakeTransfer.setPower(-0.2);
            intakeServo.setPower(-0.8);//-0.4
            sleep(1000); // This is okay in Auto because we WANT to wait

            // Stop everything
            intakeTransfer.setPower(0);
            intakeServo.setPower(0);
        }
    }
}
