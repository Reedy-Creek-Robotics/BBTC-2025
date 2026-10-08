package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "BIOBUZZ Limelight Color Tracker", group = "Limelight")
public class BiobuzzLimelightSimple extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {
        // Initialize Limelight from hardware map (config name must be "limelight")
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();

        // Switch to your Color Pipeline index (e.g., Pipeline 0 for Pollen, Pipeline 1 for Nectar)
        limelight.pipelineSwitch(0);

        telemetry.addData("Status", "Limelight Ready for BIOBUZZ!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            String detectedItem = "NONE";
            double targetX = 0.0;
            double targetY = 0.0;
            double targetArea = 0.0;

            if (result != null && result.isValid()) {
                // Read targeting data directly from the active color pipeline blob
                targetX = result.getTx();   // Horizontal offset (degrees)
                targetY = result.getTy();   // Vertical offset (degrees)
                targetArea = result.getTa(); // Target area percentage (0 to 100)

                int activePipeline = result.getPipelineIndex();

                if (activePipeline == 0) {
                    detectedItem = "POLLEN (Yellow)";
                } else if (activePipeline == 1) {
                    detectedItem = "NECTAR (Alliance)";
                }
            }

            telemetry.addData("Active Pipeline", limelight.getDeviceName());
            telemetry.addData("Detected Object", detectedItem);
            telemetry.addData("Target X (deg)", targetX);
            telemetry.addData("Target Area (%)", targetArea);
            telemetry.update();
        }
    }
}