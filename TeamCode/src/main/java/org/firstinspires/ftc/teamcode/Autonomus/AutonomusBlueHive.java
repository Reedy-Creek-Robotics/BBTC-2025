package org.firstinspires.ftc.teamcode.Autonomus;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Auto : Blue Hive")
public class AutonomusBlueHive extends LinearOpMode {

    // Hardware declarations from TeleopScrimmage
    public DcMotor flmotor, frmotor, blmotor, brmotor;
    public GoBildaPinpointDriver pinpoint;
    public DcMotorEx Shooter;
    public DcMotorEx Intake;
    public CRServo ShooterServo;

    // Timer for time-based autonomous movements
    private ElapsedTime runtime = new ElapsedTime();

    static final double DRIVE_SPEED = 0.6;

    public enum StrafeDirection {
        LEFT, RIGHT
    }

    @Override
    public void runOpMode() {

        // Map hardware components using exact names from TeleopScrimmage
        Shooter = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flmotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        blmotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frmotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        brmotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        Intake  = hardwareMap.get(DcMotorEx.class, "Intake");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        ShooterServo = hardwareMap.get(CRServo.class, "ShootingServo");

        // Set motor directions matching TeleopScrimmage
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motors to run without encoders since they aren't available
        setDriveMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready!");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) return;

        // -------------------------------
        // AUTONOMOUS STEPS (Timed in Seconds)
        // -------------------------------
        // Adjust the time (first parameter) to tune your distances
        Shooter.setPower(0.6);
        sleep(1000);
        Intake.setPower(1);
        ShooterServo.setPower(1);
        sleep(3500);
        Intake.setPower(0);
        ShooterServo.setPower(0);
        moveForward(0.5, DRIVE_SPEED); // Drive forward for 0.5 seconds

        sleep(100);
        //shoot

        // End
        stopAll();
    }

    // ==========================================
    // TIME-BASED MOVEMENT HELPER METHODS
    // ==========================================

    public void moveForward(double seconds, double speed) {
        runtime.reset();

        while (opModeIsActive() && runtime.seconds() < seconds) {
            flmotor.setPower(Math.abs(speed));
            blmotor.setPower(Math.abs(speed));
            frmotor.setPower(Math.abs(speed));
            brmotor.setPower(Math.abs(speed));

            telemetry.addData("Driving Forward", "Time Left: %.2f s", (seconds - runtime.seconds()));
            telemetry.update();
        }

        stopDriveMotors();
    }

    public void strafe(double seconds, double speed, StrafeDirection direction) {
        runtime.reset();
        double dirMultiplier = (direction == StrafeDirection.RIGHT) ? 1.0 : -1.0;

        while (opModeIsActive() && runtime.seconds() < seconds) {
            // Standard Mecanum Strafe Pattern (+FL, -BL, -FR, +BR for Right)
            flmotor.setPower(speed * dirMultiplier);
            blmotor.setPower(-speed * dirMultiplier);
            frmotor.setPower(-speed * dirMultiplier);
            brmotor.setPower(speed * dirMultiplier);

            telemetry.addData("Strafing", "%s | Time Left: %.2f s", direction.name(), (seconds - runtime.seconds()));
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