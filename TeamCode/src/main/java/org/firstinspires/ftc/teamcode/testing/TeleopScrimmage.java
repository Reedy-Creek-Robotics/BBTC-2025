package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Main Scrimmage TeleOp", group = "Scrimmage")
public class TeleopScrimmage extends OpMode {

    public DcMotor flmotor, frmotor, blmotor, brmotor;
    public GoBildaPinpointDriver pinpoint;
    public DcMotorEx Shooter; // Intake commented out
    // public DcMotorEx Intake;

    // Toggle and state variables
    private boolean shooterActive = false;
    private double shooterPower = 0.7; // Default starting target velocity (TPS)
    // private int intakeState = 0; // 0 = Off, 1 = Forward, -1 = Reverse

    // Edge-detection state variables
    private boolean lastGamepad1B = false;
    private boolean lastDpadUpState = false;
    private boolean lastDpadDownState = false;
    // private boolean lastGamepad1A = false;
    // private boolean lastGamepad1X = false;

    @Override
    public void init() {
        // Map hardware components
        // Intake = hardwareMap.get(DcMotorEx.class, "Intake");
        Shooter = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flmotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        blmotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frmotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        brmotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Set motor directions
        Shooter.setDirection(DcMotor.Direction.REVERSE);
        // Intake.setDirection(DcMotor.Direction.REVERSE);
        flmotor.setDirection(DcMotor.Direction.REVERSE);
        blmotor.setDirection(DcMotor.Direction.REVERSE);
        frmotor.setDirection(DcMotor.Direction.FORWARD);
        brmotor.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behavior
        flmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        blmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        brmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        Shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set motor run modes
        Shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized and Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Update the Pinpoint Odometry computer every loop cycle
        pinpoint.update();

        // 2. Mecanum Drivetrain Control (Driver 1)
        double forward = -gamepad1.left_stick_y;
        double strafe  = gamepad1.left_stick_x;
        double rotate  = gamepad1.right_stick_x;

        // Calculate raw mecanum motor powers
        double frontLeftPower  = forward + strafe + rotate;
        double backLeftPower   = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower  = forward + strafe - rotate;

        // Find the maximum absolute value among all four motors
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

        // Only scale down powers proportionally if any motor command exceeds 1.0
        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

        // Apply calculated powers to the drivetrain motors
        flmotor.setPower(frontLeftPower);
        blmotor.setPower(backLeftPower);
        frmotor.setPower(frontRightPower);
        brmotor.setPower(backRightPower);

        // 3. Shooter Toggle Control (Press 'B' to toggle ON/OFF)
        boolean currentB = gamepad1.b;
        if (currentB && !lastGamepad1B) {
            shooterActive = !shooterActive;
        }
        lastGamepad1B = currentB;

        // 4. Shooter Velocity Tuner (D-pad Up / Down to adjust target TPS by increments of 50)
        boolean currentDpadUp = gamepad1.dpad_up;
        boolean currentDpadDown = gamepad1.dpad_down;

        if (currentDpadUp && !lastDpadUpState) {
            shooterPower += 0.05; // Increase target velocity by 50 TPS per press
        }
        if (currentDpadDown && !lastDpadDownState) {
            shooterPower = Math.max(0.0, shooterPower - 0.05); // Decrease by 50 TPS, minimum 0
        }

        lastDpadUpState = currentDpadUp;
        lastDpadDownState = currentDpadDown;

        // Apply velocity to shooter based on state
        if (shooterActive) {
            Shooter.setPower(shooterPower);
        } else {
            Shooter.setPower(0.0);
        }

        /*
        // 5. Intake Toggle Control (Commented Out)
        boolean currentA = gamepad1.a;
        if (currentA && !lastGamepad1A) {
            intakeState = (intakeState == 1) ? 0 : 1;
        }
        lastGamepad1A = currentA;

        boolean currentX = gamepad1.x;
        if (currentX && !lastGamepad1X) {
            intakeState = (intakeState == -1) ? 0 : -1;
        }
        lastGamepad1X = currentX;

        if (intakeState == 1) {
            Intake.setPower(1.0);
        } else if (intakeState == -1) {
            Intake.setPower(-1.0);
        } else {
            Intake.setPower(0.0);
        }
        */

        // 6. Telemetry Diagnostics
        telemetry.addData("Shooter Status", shooterActive ? "ON" : "OFF");
        telemetry.addData("Target Velocity (TPS)", shooterPower);
        telemetry.addData("Actual Velocity (TPS)", Shooter.getVelocity());
        // telemetry.addData("Intake Status", intakeState == 1 ? "FORWARD" : (intakeState == -1 ? "REVERSE" : "OFF"));
        telemetry.update();
    }
}