package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.mechanisms.limelightbiobuzz;

/**
 * BioBuzz cluster-aim test for a continuously-tracking turret.
 *
 * <p>Points the Limelight at a hive cell and streams the full aim solution (turret bearing,
 * launch elevation, velocity, fusion quality, latency) to telemetry. The turret keeps its own
 * lock; this OpMode only reports what the solver wants, so you can verify convergence while
 * the turret is locked on.</p>
 *
 * <p>Gamepad1 tuning controls (live, no restart):</p>
 * <ul>
 *   <li>A / Y  - muzzle velocity -/+ 10 in/s</li>
 *   <li>X      - toggle drag model on/off</li>
 *   <li>B      - toggle low/high arc</li>
 *   <li>DPad L/R - drag k -/+ 0.0001</li>
 *   <li>DPad U/D  - latency compensation on/off</li>
 * </ul>
 */
@TeleOp(name = "BioBuzz Aim Test", group = "Testing")
public class BioBuzzTurretTesting extends LinearOpMode {

    private limelightbiobuzz bio;
    private IMU imu;

    @Override
    public void runOpMode() {
        // IMU is optional: solver falls back to no-rotation/no-latency-comp if missing.
        try {
            imu = hardwareMap.get(IMU.class, "imu");
            imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.UP,
                    RevHubOrientationOnRobot.UsbFacingDirection.FORWARD)));
        } catch (Exception e) {
            imu = null;
            telemetry.addData("IMU", "not available - latency comp degraded");
        }

        bio = new limelightbiobuzz(hardwareMap, "limelight", imu);

        telemetry.addLine("BioBuzz aim test ready - point at a hive cell");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // --- read solver ---
            bio.update();
            limelightbiobuzz.AimResult aim = bio.computeAim();

            // --- live tuning controls ---
            handleTuningButtons();

            // --- telemetry ---
            telemetry.clear();
            bio.addTelemetry(telemetry);
            telemetry.addLine("-- tuning --");
            telemetry.addData("v0", "%.0f in/s", bio.MUZZLE_VELOCITY_IN_S);
            telemetry.addData("drag k", "%.5f", bio.DRAG_K);
            telemetry.addData("drag model", bio.USE_DRAG_MODEL ? "on" : "vacuum");
            telemetry.addData("arc", bio.HIGH_ARC ? "high" : "low");
            telemetry.addData("latency comp", bio.COMPENSATE_LATENCY ? "on" : "off");
            if (bio.isLocked()) {
                telemetry.addData("LOCKED on cluster", "%d (%d tags)%s",
                        aim != null ? aim.clusterStart : -1,
                        aim != null ? aim.tagCount : 0,
                        bio.isCoasting() ? " (coasting)" : "");
            } else {
                telemetry.addData("LOCKED on cluster", "no (%s)", bio.getFuseNote());
            }
            telemetry.update();

            idle();
        }

        bio.stop();
    }

    /** Debounced gamepad tuning (see class javadoc for the button map). */
    private void handleTuningButtons() {
        // A / Y : muzzle velocity -/+ 10
        if (gamepad1.a) bio.MUZZLE_VELOCITY_IN_S -= 10;
        if (gamepad1.y) bio.MUZZLE_VELOCITY_IN_S += 10;
        bio.MUZZLE_VELOCITY_IN_S = Math.max(50, Math.min(1000, bio.MUZZLE_VELOCITY_IN_S));

        // D-pad L/R : drag k -/+ 0.0001
        if (gamepad1.dpad_left) bio.DRAG_K -= 0.0001;
        if (gamepad1.dpad_right) bio.DRAG_K += 0.0001;
        bio.DRAG_K = Math.max(0, Math.min(0.01, bio.DRAG_K));

        // toggles
        if (gamepad1.x && !xWas) bio.USE_DRAG_MODEL = !bio.USE_DRAG_MODEL;
        if (gamepad1.b && !bWas) bio.HIGH_ARC = !bio.HIGH_ARC;
        if (gamepad1.dpad_up && !upWas) bio.COMPENSATE_LATENCY = !bio.COMPENSATE_LATENCY;

        xWas = gamepad1.x;
        bWas = gamepad1.b;
        upWas = gamepad1.dpad_up;
        // small dead-time so held buttons don't spam toggles
        if (gamepad1.a || gamepad1.y || gamepad1.dpad_left || gamepad1.dpad_right) {
            sleep(60);
        }
    }

    private boolean xWas = false, bWas = false, upWas = false;
}
