package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.pedropathing.geometry.Pose;

public class Turret {

    private DcMotorEx turretMotor;

    // Field goal coordinates (in inches)
    private double goalX = 72.0;
    private double goalY = 72.0;

    // --- PIDF Tuning Constants ---
    private final double KP = 0.025;
    private final double KI = 0.0001;
    private final double KD = 0.0015;
    private final double KF = 0.0;

    // PIDF tracking variables
    private ElapsedTime loopTimer = new ElapsedTime();
    private double integratedError = 0.0;
    private double lastError = 0.0;

    public void init(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        loopTimer.reset();
    }

    /**
     * Updates the turret aiming based on the robot's current PedroPathing pose.
     */
    public void update(Pose robotPose) {
        double robotX = robotPose.getX();
        double robotY = robotPose.getY();
        double robotHeading = robotPose.getHeading();

        // Get time delta (dt) since last loop iteration in seconds
        double dt = loopTimer.seconds();
        loopTimer.reset();

        // Prevent division by zero or massive spikes on the first loop
        if (dt > 0.1 || dt <= 0.0) dt = 0.02;

        // Calculate the field-centric angle from the robot to the goal
        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;
        double targetFieldAngle = Math.atan2(deltaY, deltaX);

        // Calculate the relative angle error in degrees
        double turretAngleErrorRad = targetFieldAngle - robotHeading;
        turretAngleErrorRad = Math.atan2(Math.sin(turretAngleErrorRad), Math.cos(turretAngleErrorRad)); // Normalize (-PI to PI)
        double errorDegrees = Math.toDegrees(turretAngleErrorRad);

        // Full PIDF Calculation
        double proportional = KP * errorDegrees;

        // Integral term with anti-windup clamping
        integratedError += errorDegrees * dt;
        integratedError = Math.max(-500.0, Math.min(500.0, integratedError));
        double integral = KI * integratedError;

        // Derivative term
        double derivative = KD * ((errorDegrees - lastError) / dt);
        lastError = errorDegrees;

        // Feedforward term
        double feedforward = KF * Math.signum(errorDegrees);

        // Total PIDF Power Output
        double turretPower = proportional + integral + derivative + feedforward;
        turretPower = Math.max(-1.0, Math.min(1.0, turretPower));

        // Apply deadzone (within 0.75 degrees, stop motor to prevent buzzing)
        if (Math.abs(errorDegrees) > 0.75) {
            turretMotor.setPower(turretPower);
        } else {
            turretMotor.setPower(0);
            integratedError = 0; // Reset integral when close to target
        }
    }

    /**
     * Dynamically change target goal coordinates if needed during a match.
     */
    public void setGoal(double x, double y) {
        this.goalX = x;
        this.goalY = y;
    }

    public double getErrorDegrees() {
        return lastError;
    }
}