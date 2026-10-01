package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "BioBuzz Relative Target Space Targeting", group = "Limelight")
public class rana2 extends LinearOpMode {

    private Limelight3A limelight;

    // Define all 4 AprilTag IDs for your Alliance's Hive cluster
    private static final int[] HIVE_TAGS = {0, 1, 2, 3}; // Change to your alliance IDs

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0); // Standard AprilTag pipeline
        limelight.start();

        telemetry.addData("Status", "Target-Space Tracker Initialized (No Map Required)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();

                boolean targetFound = false;
                LLResultTypes.FiducialResult activeTag = null;

                // Loop through all visible tags to find any matching our hive cluster
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int detectedID = fiducial.getFiducialId();

                    for (int tagId : HIVE_TAGS) {
                        if (detectedID == tagId) {
                            targetFound = true;
                            activeTag = fiducial;
                            break;
                        }
                    }
                    if (targetFound) break;
                }

                if (targetFound && activeTag != null) {
                    int matchedId = activeTag.getFiducialId();

                    // Target Space gives coordinates relative to the tag itself:
                    // X = Forward/Back distance from tag
                    // Y = Left/Right lateral distance from tag
                    // Z = Up/Down height difference between camera and tag
                    double xMeters = activeTag.getRobotPoseTargetSpace().getPosition().x;
                    double yMeters = activeTag.getRobotPoseTargetSpace().getPosition().y;
                    double zMeters = activeTag.getRobotPoseTargetSpace().getPosition().z;

                    // Convert to inches
                    double forwardInches = xMeters * 39.3701;
                    double lateralInches = yMeters * 39.3701;
                    double heightInches = zMeters * 39.3701; // Automatically changes when hive is raised/lowered!

                    // True distance from camera to the target face
                    double trueDistance = Math.sqrt(Math.pow(forwardInches, 2) + Math.pow(lateralInches, 2));

                    // Horizontal steering offset error (tx)
                    double tx = result.getTx();

                    // --- Telemetry for Drivers ---
                    telemetry.addData("Status", "LOCKED ON HIVE TAG ID: %d", matchedId);
                    telemetry.addData("Hive State", heightInches > 25.0 ? "RAISED" : "LOWERED");
                    telemetry.addData("True Distance", "%.1f inches", trueDistance);
                    telemetry.addData("Height Offset (Z)", "%.1f inches", heightInches);
                    telemetry.addData("Steering Aim Error (tx)", "%.2f deg", tx);

                } else {
                    telemetry.addData("Status", "Searching for Hive Tags...");
                }

            } else {
                telemetry.addData("Status", "No targets in view.");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}