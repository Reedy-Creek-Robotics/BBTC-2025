package org.firstinspires.ftc.teamcode.pedroPathing.PedroTesting.PedroTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@Autonomous(name = "Heading Tuner Gamepad", group = "Pedro Pathing Tuning")
public class Heading_Tuner extends OpMode {

    // Tunable heading coefficients
    public static double P = 1.0;
    public static double I = 0.0;
    public static double D = 0.003;
    public static double F = 0.0;

    // Adjustment increments per button press
    private final double pStep = 0.05;
    private final double dStep = 0.0005;
    private final double fStep = 0.005;

    // Edge-detection booleans
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastLeftBumper = false;
    private boolean lastRightBumper = false;

    private Follower follower;
    private PIDFCoefficients headingGains;
    private PIDFCoefficients originalHeadingGains;
    private double targetHeading = 0.0;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(0, 0, 0));
        follower.deactivateAllPIDFs();
        follower.activateHeading();

        headingGains = follower.getConstants().getCoefficientsHeadingPIDF();
        originalHeadingGains = new PIDFCoefficients(headingGains.P, headingGains.I, headingGains.D, headingGains.F);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addData("Status", "Initialized. Press play to start tuning.");
        telemetry.update();
    }

    @Override
    public void start() {
        targetHeading = Math.toRadians(90);
        follower.holdPoint(new Pose(0, 0, targetHeading));
    }

    @Override
    public void loop() {
        // --- P & D Adjustments (DPAD) ---
        if (gamepad1.dpad_up && !lastDpadUp) P += pStep;
        if (gamepad1.dpad_down && !lastDpadDown) P = Math.max(0.0, P - pStep);
        if (gamepad1.dpad_right && !lastDpadRight) D += dStep;
        if (gamepad1.dpad_left && !lastDpadLeft) D = Math.max(0.0, D - dStep);

        // --- F Adjustments (BUMPERS) ---
        if (gamepad1.right_bumper && !lastRightBumper) F += fStep;
        if (gamepad1.left_bumper && !lastLeftBumper) F = Math.max(0.0, F - fStep);

        // Store button states for edge-detection
        lastDpadUp = gamepad1.dpad_up;
        lastDpadDown = gamepad1.dpad_down;
        lastDpadLeft = gamepad1.dpad_left;
        lastDpadRight = gamepad1.dpad_right;
        lastLeftBumper = gamepad1.left_bumper;
        lastRightBumper = gamepad1.right_bumper;

        // Update the gains Pedro already reads so button presses take effect during the test.
        headingGains.P = P;
        headingGains.I = I;
        headingGains.D = D;
        headingGains.F = F;

        double requestedHeading = targetHeading;
        if (gamepad1.a) requestedHeading = 0.0;
        else if (gamepad1.b) requestedHeading = Math.toRadians(90);
        else if (gamepad1.x) requestedHeading = Math.toRadians(-90);
        else if (gamepad1.y) requestedHeading = Math.toRadians(180);

        // holdPoint clears the PID's memory and briefly cuts motor power.
        // Give Pedro a new target only when the requested heading changes.
        if (requestedHeading != targetHeading) {
            targetHeading = requestedHeading;
            follower.holdPoint(new Pose(0, 0, targetHeading));
        }
        follower.update();

        // Telemetry
        telemetry.addData("Target Heading (deg)", Math.toDegrees(targetHeading));
        telemetry.addData("Current Heading (deg)", Math.toDegrees(follower.getHeading()));
        telemetry.addData("Heading Error (deg)", Math.toDegrees(follower.getHeadingError()));
        telemetry.addLine("\n--- Gamepad Controls ---");
        telemetry.addLine("DPAD UP/DOWN    : Adjust P (+/- " + pStep + ")");
        telemetry.addLine("DPAD RIGHT/LEFT : Adjust D (+/- " + dStep + ")");
        telemetry.addLine("RIGHT/LEFT BUMPER: Adjust F (+/- " + fStep + ")");
        telemetry.addLine("Buttons A/B/X/Y : Target 0° / 90° / -90° / 180°");
        telemetry.addLine("\n--- Active Values ---");
        telemetry.addData("P Gain", "%.4f", P);
        telemetry.addData("D Gain", "%.5f", D);
        telemetry.addData("F Gain", "%.4f", F);
        telemetry.update();
    }

    @Override
    public void stop() {
        // Restore the robot's heading gains before the next OpMode uses them.
        follower.getConstants().setCoefficientsHeadingPIDF(originalHeadingGains);
        follower.breakFollowing();
    }
}
