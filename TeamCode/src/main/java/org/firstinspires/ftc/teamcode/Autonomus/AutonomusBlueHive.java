package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto : Blue Hive")
public class AutonomusBlueHive extends LinearOpMode {

    // Hardware declarations
    public DcMotor flmotor, frmotor, blmotor, brmotor;
    public DcMotorEx Shooter;
    public DcMotorEx Intake;
    public CRServo ShooterServo;

    // Timer for time-based methods
    private ElapsedTime runtime = new ElapsedTime();

    // Encoder calculations (Adjust COUNTS_PER_INCH if your wheel size or gear ratio is different)
    static final double COUNTS_PER_INCH = (537.6 * 1.0) / (3.779 * Math.PI);

    public enum StrafeDirection {
        LEFT, RIGHT
    }

    @Override
    public void runOpMode() {

        // Map hardware components
        Shooter = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flmotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        blmotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frmotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        brmotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        Intake  = hardwareMap.get(DcMotorEx.class, "Intake");
        ShooterServo = hardwareMap.get(CRServo.class, "ShootingServo");

        // Set motor directions
        Shooter.setDirection(DcMotor.Direction.FORWARD);
        Intake.setDirection(DcMotor.Direction.FORWARD);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);
        ShooterServo.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set zero power behavior
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        Shooter.setVelocityPIDFCoefficients(37, 0, 0.01, 12);

        // Reset encoders on startup
        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS ROUTINE
        // -------------------------------
        sleep(10000);
        Shooter.setVelocity(1000);
        sleep(1000);
        Intake.setPower(1);
        ShooterServo.setPower(1);
        sleep(6000);
        Intake.setPower(0);
        ShooterServo.setPower(0);

        // Move forward 24 inches using encoders
        moveTime(0.5, 0.6);

        // Strafe right 12 inches using encoders
        /*strafeDistance(StrafeDirection.LEFT, 0.6, 36);

        sleep(100);
        moveDistance(0.5, 24*3.5);

        sleep(100);
        strafeDistance(StrafeDirection.LEFT, 0.6, 24);*/

        // End
        stopAll();
    }

    // ==========================================
    // MOVEMENT METHODS (TIME & DISTANCE)
    // ==========================================

    /**
     * Moves the robot for a specific amount of TIME.
     */
    public void moveTime(double seconds, double speed) {
        runtime.reset();
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while (opModeIsActive() && runtime.seconds() < seconds) {
            setAllPower(speed);
            telemetry.addData("Moving (Time)", "Time Left: %.2f s", (seconds - runtime.seconds()));
            telemetry.update();
        }

        stopDriveMotors();
    }

    /**
     * Moves the robot a specific DISTANCE in inches using motor encoders.
     */
    public void moveDistance(double speed, double targetInches) {
        int targetTicks = (int) (targetInches * COUNTS_PER_INCH);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flmotor.setTargetPosition(targetTicks);
        blmotor.setTargetPosition(targetTicks);
        frmotor.setTargetPosition(targetTicks);
        brmotor.setTargetPosition(targetTicks);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double directionalSpeed = Math.abs(speed) * Math.signum(targetInches);
        setAllPower(directionalSpeed);

        while (opModeIsActive() && (flmotor.isBusy() && blmotor.isBusy() && frmotor.isBusy() && brmotor.isBusy())) {
            telemetry.addData("Moving (Distance)", "Speed: %.2f | Target: %.1f in", speed, targetInches);
            telemetry.addData("Current FL Position", flmotor.getCurrentPosition());
            telemetry.update();
        }

        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafes the robot a specific DISTANCE in inches using motor encoders.
     *
     * @param direction    StrafeDirection.LEFT or StrafeDirection.RIGHT
     * @param speed        Motor power/speed (e.g., 0.6)
     * @param targetInches Distance in inches to strafe
     */
    public void strafeDistance(StrafeDirection direction, double speed, double targetInches) {
        int targetTicks = (int) (targetInches * COUNTS_PER_INCH);

        setDriveMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Mecanum strafe math: opposite pairs need opposite target signs
        int flTarget = (direction == StrafeDirection.RIGHT) ? targetTicks : -targetTicks;
        int blTarget = (direction == StrafeDirection.RIGHT) ? -targetTicks : targetTicks;
        int frTarget = (direction == StrafeDirection.RIGHT) ? -targetTicks : targetTicks;
        int brTarget = (direction == StrafeDirection.RIGHT) ? targetTicks : -targetTicks;

        flmotor.setTargetPosition(flTarget);
        blmotor.setTargetPosition(blTarget);
        frmotor.setTargetPosition(frTarget);
        brmotor.setTargetPosition(brTarget);

        setDriveMode(DcMotor.RunMode.RUN_TO_POSITION);

        double dirMultiplier = (direction == StrafeDirection.RIGHT) ? 1.0 : -1.0;
        flmotor.setPower(Math.abs(speed) * dirMultiplier);
        blmotor.setPower(Math.abs(speed) * -dirMultiplier);
        frmotor.setPower(Math.abs(speed) * -dirMultiplier);
        brmotor.setPower(Math.abs(speed) * dirMultiplier);

        while (opModeIsActive() && (flmotor.isBusy() && blmotor.isBusy() && frmotor.isBusy() && brmotor.isBusy())) {
            telemetry.addData("Strafing (Distance)", "%s | Target: %.1f in", direction.name(), targetInches);
            telemetry.addData("Current FL Position", flmotor.getCurrentPosition());
            telemetry.update();
        }

        stopDriveMotors();
        setDriveMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Time-based strafing method (kept as an alternative).
     */
    public void strafeTime(double seconds, double speed, StrafeDirection direction) {
        runtime.reset();
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double dirMultiplier = (direction == StrafeDirection.RIGHT) ? 1.0 : -1.0;

        while (opModeIsActive() && runtime.seconds() < seconds) {
            flmotor.setPower(speed * dirMultiplier);
            blmotor.setPower(-speed * dirMultiplier);
            frmotor.setPower(-speed * dirMultiplier);
            brmotor.setPower(speed * dirMultiplier);

            telemetry.addData("Strafing (Time)", "%s | Time Left: %.2f s", direction.name(), (seconds - runtime.seconds()));
            telemetry.update();
        }

        stopDriveMotors();
    }

    private void setDriveMode(DcMotor.RunMode mode) {
        flmotor.setMode(mode);
        blmotor.setMode(mode);
        frmotor.setMode(mode);
        brmotor.setMode(mode);
    }

    private void setAllPower(double power) {
        flmotor.setPower(power);
        blmotor.setPower(power);
        frmotor.setPower(power);
        brmotor.setPower(power);
    }

    private void stopDriveMotors() {
        flmotor.setPower(0);
        blmotor.setPower(0);
        frmotor.setPower(0);
        brmotor.setPower(0);
    }

    public void stopAll() {
        stopDriveMotors();
        Shooter.setPower(0);
        Intake.setPower(0);
        ShooterServo.setPower(0);
    }
}