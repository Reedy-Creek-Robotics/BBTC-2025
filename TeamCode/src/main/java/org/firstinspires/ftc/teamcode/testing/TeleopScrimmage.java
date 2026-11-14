package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Main Scrimmage TeleOp", group = "Scrimmage")
public class TeleopScrimmage extends OpMode {

    public DcMotor flmotor, frmotor, blmotor, brmotor;
    public GoBildaPinpointDriver pinpoint;
    public DcMotorEx Shooter;
    public DcMotorEx Intake;
    public CRServo ShooterServo;

    // Toggle and state variables
    private boolean shooterActive = false;
    private double shooterPower = 0.6; // Fixed shooter power set to 60%
    private int intakeState = 0; // 0 = Off, 1 = Forward, -1 = Reverse

    // Servo power level when intake is running
    private double servoPower = 1.0;

    // Edge-detection state variables
    private boolean lastGamepad1B = false;
    private boolean lastGamepad1A = false;
    private boolean lastGamepad1X = false;

    @Override
    public void init() {
        // Map hardware components
        Shooter = hardwareMap.get(DcMotorEx.class, "Flywheel");
        flmotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        blmotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frmotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        brmotor = hardwareMap.get(DcMotor.class, "backRightMotor");
        Intake  = hardwareMap.get(DcMotorEx.class, "Intake");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        ShooterServo = hardwareMap.get(CRServo.class, "ShootingServo");

        // Set motor directions
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

        // Set motor run modes
        Shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        blmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        brmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Initialized and Ready!");
        telemetry.update();
    }

    @Override
    public void loop() {
        // 1. Update the Pinpoint Odometry computer every loop cycle
        pinpoint.update();

        // 2. Mecanum Drivetrain Control (Driver 1)
        double forward = gamepad1.left_stick_y;
        double strafe  = -gamepad1.left_stick_x;
        double rotate  = -gamepad1.right_stick_x;

        // Calculate raw mecanum motor powers
        double frontLeftPower  = forward + strafe + rotate;
        double backLeftPower   = forward - strafe + rotate;
        double frontRightPower = forward - strafe - rotate;
        double backRightPower  = forward + strafe - rotate;

        // Find the maximum absolute value among all four motors
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            backLeftPower /= max;
            frontRightPower /= max;
            backRightPower /= max;
        }

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

        // Apply power to shooter based on state (Fixed at 0.6 when active)
        if (shooterActive) {
            Shooter.setPower(shooterPower);
        } else {
            Shooter.setPower(0.0);
        }

        // 4. Intake & Shooter Servo Toggle Control (A = Forward, X = Reverse)
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
            ShooterServo.setPower(servoPower);
        } else if (intakeState == -1) {
            Intake.setPower(-1.0);
            ShooterServo.setPower(-servoPower);
        } else {
            Intake.setPower(0.0);
            ShooterServo.setPower(0.0);
        }

        // 5. Telemetry Diagnostics
        telemetry.addData("Shooter Status", shooterActive ? "ON" : "OFF");
        telemetry.addData("Shooter Power", shooterPower);
        telemetry.addData("Actual Velocity (TPS)", Shooter.getVelocity());
        telemetry.addData("Intake / Servo Status", intakeState == 1 ? "FORWARD" : (intakeState == -1 ? "REVERSE" : "OFF"));
        telemetry.update();
    }
}