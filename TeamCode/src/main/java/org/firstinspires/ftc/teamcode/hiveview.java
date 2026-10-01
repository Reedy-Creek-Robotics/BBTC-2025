package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import java.util.ArrayList;

@TeleOp(name = "HiveVision Limelight (Frozen Display)", group = "Vision")
public class hiveview extends LinearOpMode {

    private Limelight3A limelight;

    // Target Persistence variables
    private List<LLResultTypes.DetectorResult> lastDetections = new ArrayList<>();
    private double lastDetectionTime = 0.0;
    private static final double FREEZE_HOLD_SECONDS = 2.0; // Time in seconds to hold last detection

    @Override
    public void runOpMode() throws InterruptedException {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(3);

        telemetry.addData("Status", "Limelight Initialized with Target Persistence");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            // Store new detections whenever valid targets are visible
            if (result != null && result.isValid() && !result.getDetectorResults().isEmpty()) {
                lastDetections = result.getDetectorResults();
                lastDetectionTime = getRuntime();
            }

            double timeSinceLastSeen = getRuntime() - lastDetectionTime;

            // If we have saved detections and we are within the timeout window
            if (!lastDetections.isEmpty() && timeSinceLastSeen < FREEZE_HOLD_SECONDS) {
                boolean isCurrentlyVisible = (timeSinceLastSeen < 0.15);

                if (isCurrentlyVisible) {
                    telemetry.addData("Status", "🟢 LIVE TARGET");
                } else {
                    telemetry.addData("Status", "❄️ FROZEN (Holding for %.1fs)", FREEZE_HOLD_SECONDS - timeSinceLastSeen);
                }

                telemetry.addData("Targets Found", lastDetections.size());

                for (int i = 0; i < lastDetections.size(); i++) {
                    LLResultTypes.DetectorResult detection = lastDetections.get(i);
                    telemetry.addData("Target #" + i, "%s (ID: %d) [%.0f%%]",
                            detection.getClassName(),
                            detection.getClassId(),
                            detection.getConfidence() * 100);
                    telemetry.addData("  Angles (X, Y)", "%.1f deg, %.1f deg",
                            detection.getTargetXDegrees(),
                            detection.getTargetYDegrees());
                }
            } else {
                telemetry.addData("Status", "❌ No Target Found");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}


