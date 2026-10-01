
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "SWYFT LiDAR Comprehensive Test Suite", group = "Diagnostics")
public class SwyftLidarDiagnostic extends LinearOpMode {

    private SwyftRanger lidar;
    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime testTimer = new ElapsedTime();

    // Mode Switching
    private enum TestMode {
        REALTIME_MONITOR,
        THRESHOLD_TRIGGER,
        NOISE_STATISTICS,
        LOOP_LATENCY
    }

    private TestMode currentMode = TestMode.REALTIME_MONITOR;
    private boolean dpadUpPressed = false;
    private boolean dpadDownPressed = false;

    // Threshold Test Settings
    private double targetThresholdInches = 12.0;

    // Noise Test Variables
    private double minVal = Double.MAX_VALUE;
    private double maxVal = Double.MIN_VALUE;
    private double sumVal = 0.0;
    private int sampleCount = 0;

    @Override
    public void runOpMode() {
        // Initialize sensor connected to Analog Port 0 named "lidarSensor"
        lidar = new SwyftRanger(hardwareMap, "lidarSensor");

        telemetry.addData("Status", "Initialized. Press START to begin testing.");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            handleModeSwitching();

            telemetry.addData("=== CURRENT TEST MODE ===", currentMode);
            telemetry.addData("Controls", "DPAD Up/Down to switch test modes");
            telemetry.addLine("------------------------------------");

            switch (currentMode) {
                case REALTIME_MONITOR:
                    runRealtimeMonitorTest();
                    break;

                case THRESHOLD_TRIGGER:
                    runThresholdTriggerTest();
                    break;

                case NOISE_STATISTICS:
                    runNoiseStatisticsTest();
                    break;

                case LOOP_LATENCY:
                    runLoopLatencyTest();
                    break;
            }

            telemetry.update();
        }
    }

    /**
     * TEST 1: Real-Time Monitoring & Calibration Check
     * Evaluates raw voltage output vs. converted filtered distance.
     */
    private void runRealtimeMonitorTest() {
        double rawV = lidar.getRawVoltage();
        double distIn = lidar.getDistanceInches();
        double distCm = lidar.getDistanceCm();
        double filteredIn = lidar.getFilteredDistanceInches();

        telemetry.addData("Raw Voltage", "%.4f V", rawV);
        telemetry.addData("Distance (Raw)", "%.2f in | %.2f cm", distIn, distCm);
        telemetry.addData("Distance (Filtered)", "%.2f in", filteredIn);
        telemetry.addData("Raw vs Filtered Delta", "%.3f in", Math.abs(distIn - filteredIn));
    }

    /**
     * TEST 2: Threshold & Game Element Detection Test
     * Simulates object detection logic (e.g., detecting game pieces or walls).
     */
    private void runThresholdTriggerTest() {
        // Adjust threshold using Gamepad 1 A/B buttons
        if (gamepad1.a) targetThresholdInches += 0.1;
        if (gamepad1.b) targetThresholdInches = Math.max(1.0, targetThresholdInches - 0.1);

        double currentDist = lidar.getFilteredDistanceInches();
        boolean isDetected = lidar.isTargetDetected(targetThresholdInches);

        telemetry.addData("Target Threshold", "%.1f in (A: +0.1, B: -0.1)", targetThresholdInches);
        telemetry.addData("Current Range", "%.2f in", currentDist);

        if (isDetected) {
            telemetry.addData("TRIGGER STATUS", ">>> OBJECT DETECTED <<<");
        } else {
            telemetry.addData("TRIGGER STATUS", "NO TARGET IN RANGE");
        }
    }

    /**
     * TEST 3: Signal Noise & Stability Benchmark
     * Measures peak-to-peak jitter (Max - Min) and noise variance over time.
     */
    private void runNoiseStatisticsTest() {
        if (gamepad1.x) { // Reset stats
            minVal = Double.MAX_VALUE;
            maxVal = Double.MIN_VALUE;
            sumVal = 0.0;
            sampleCount = 0;
            testTimer.reset();
        }

        double val = lidar.getDistanceInches();
        minVal = Math.min(minVal, val);
        maxVal = Math.max(maxVal, val);
        sumVal += val;
        sampleCount++;

        double avg = sumVal / sampleCount;
        double jitter = maxVal - minVal;

        telemetry.addLine("Hold target steady and press 'X' to reset stats");
        telemetry.addData("Sample Count", sampleCount);
        telemetry.addData("Runtime", "%.1f sec", testTimer.seconds());
        telemetry.addData("Min Distance", "%.2f in", minVal);
        telemetry.addData("Max Distance", "%.2f in", maxVal);
        telemetry.addData("Average Distance", "%.2f in", avg);
        telemetry.addData("Peak-to-Peak Jitter", "%.3f in", jitter);
    }

    /**
     * TEST 4: Control Hub Reading Latency & Loop Speed
     * Measures sensor polling overhead to ensure non-blocking performance in Auto.
     */
    private void runLoopLatencyTest() {
        long startTime = System.nanoTime();
        double dist = lidar.getDistanceInches();
        long endTime = System.nanoTime();

        double elapsedMs = (endTime - startTime) / 1_000_000.0;

        telemetry.addData("Last Sample Latency", "%.3f ms", elapsedMs);
        telemetry.addData("Sampled Distance", "%.2f in", dist);
        telemetry.addLine(elapsedMs < 2.0 ? "PASS: Latency optimal for autonomous loop" : "WARN: High latency detected");
    }

    /**
     * Mode switching helper logic.
     */
    private void handleModeSwitching() {
        if (gamepad1.dpad_up && !dpadUpPressed) {
            int nextIdx = (currentMode.ordinal() + 1) % TestMode.values().length;
            currentMode = TestMode.values()[nextIdx];
        }
        dpadUpPressed = gamepad1.dpad_up;

        if (gamepad1.dpad_down && !dpadDownPressed) {
            int prevIdx = (currentMode.ordinal() - 1 + TestMode.values().length) % TestMode.values().length;
            currentMode = TestMode.values()[prevIdx];
        }
        dpadDownPressed = gamepad1.dpad_down;
    }
}


