package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "TurretTest", group = "Testing")
public class TurretTest extends LinearOpMode {

    private Follower follower;
    private DcMotorEx turretMotor;

    private double StartingXcoordinate = 36;
    private double StartingYcoordinate = 12;
    private double StartingAngle = 90;

    // Define your field's goal coordinates (in inches, using Pedro's coordinate system)
    private final double GOAL_X = 72.0; // Center field X coordinate
    private final double GOAL_Y = 72.0; // Target goal Y coordinate

    // --- PIDF Tuning Constants (Tune these based on your mechanism) ---
    private final double KP = 0.025; // Proportional gain (scales with error magnitude)
    private final double KI = 0.0001; // Integral gain (accumulates error over time to fix steady-state error)
    private final double KD = 0.0015; // Derivative gain (dampens overshoot and oscillations)
    private final double KF = 0.0;    // Feedforward (static friction / gravity compensation if needed)

    // PIDF tracking variables
    private ElapsedTime loopTimer = new ElapsedTime();
    private double integratedError = 0.0;
    private double lastError = 0.0;

    @Override
    public void runOpMode() {
        // Initialize PedroPathing Follower using your Constants builder
        follower = Constants.createFollower(hardwareMap);

        // Set your starting pose
        follower.setPose(new Pose(StartingXcoordinate, StartingYcoordinate, Math.toRadians(StartingAngle)));

        // Initialize the turret motor
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized. Waiting for start...");
        telemetry.update();

        waitForStart();
        loopTimer.reset();

        while (opModeIsActive()) {
            // 1. CRITICAL: Update PedroPathing localization every loop cycle
            follower.update();

            // Get time delta (dt) since last loop iteration in seconds
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Prevent division by zero or massive spikes on the first loop
            if (dt > 0.1) dt = 0.02;

            // 2. Get current robot pose using getPose() and v2 getter methods
            Pose robotPose = follower.getPose();
            double robotX = robotPose.getX();
            double robotY = robotPose.getY();
            double robotHeading = robotPose.getHeading();

            // 3. Calculate the field-centric angle from the robot to the goal
            double deltaX = GOAL_X - robotX;
            double deltaY = GOAL_Y - robotY;

            double targetFieldAngle = Math.atan2(deltaY, deltaX);

            // 4. Calculate the relative angle error in degrees for easier PID tuning
            double turretAngleErrorRad = targetFieldAngle - robotHeading;
            turretAngleErrorRad = Math.atan2(Math.sin(turretAngleErrorRad), Math.cos(turretAngleErrorRad)); // Normalize (-PI to PI)
            double errorDegrees = Math.toDegrees(turretAngleErrorRad);

            // 5. Full PIDF Calculation
            // Proportional term
            double proportional = KP * errorDegrees;

            // Integral term (accumulates error over time) with anti-windup clamping
            integratedError += errorDegrees * dt;
            integratedError = Math.max(-500.0, Math.min(500.0, integratedError)); // Clamp integral windup
            double integral = KI * integratedError;

            // Derivative term (rate of change of error)
            double derivative = KD * ((errorDegrees - lastError) / dt);
            lastError = errorDegrees;

            // Feedforward term (helps overcome static friction)
            double feedforward = KF * Math.signum(errorDegrees);

            // Total PIDF Power Output
            double turretPower = proportional + integral + derivative + feedforward;

            // Clamp final power between -1.0 and 1.0
            turretPower = Math.max(-1.0, Math.min(1.0, turretPower));

            // Apply deadzone (e.g., within 0.75 degrees, turn off motor to stop buzzing/jitter)
            if (Math.abs(errorDegrees) > 0.75) {
                turretMotor.setPower(turretPower);
            } else {
                turretMotor.setPower(0);
                integratedError = 0; // Reset integral when close to target
            }

            // Telemetry diagnostics
            telemetry.addData("Robot X", robotX);
            telemetry.addData("Robot Y", robotY);
            telemetry.addData("Error (deg)", errorDegrees);
            telemetry.addData("P Output", proportional);
            telemetry.addData("I Output", integral);
            telemetry.addData("D Output", derivative);
            telemetry.addData("Final Power", turretPower);
            telemetry.update();
        }
    }
}