package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

@TeleOp(name = "cam test")
public class cam extends LinearOpMode {

    private Limelight3A limelight;
    private static final int TARGET_TAG_ID = 20;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start(); // Make sure to start the camera!

        telemetry.addData("Status", "Limelight Initialized. Waiting for Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the latest vision results from the Limelight
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                for (FiducialResult fiducial : result.getFiducialResults()) {

                    // FIX 1: Use getFiducialId() instead of getId()
                    if (fiducial.getFiducialId() == TARGET_TAG_ID) {

                        Pose3D robotPose = fiducial.getRobotPoseTargetSpace();

                        if (robotPose != null) {
                            // The built-in Pose3D stores components in a 'position' object
                            // Values are in METERS
                            Position pos = robotPose.getPosition();

                            double lateralX = pos.x;      // Side-to-side
                            double verticalY = pos.y;     // Up-and-down
                            double forwardZ = pos.z;      // Distance from the tag

                            // Calculate straight-line distance
                            double totalDistance = Math.sqrt(lateralX * lateralX + verticalY * verticalY + forwardZ * forwardZ);
                            telemetry.addData("Distance (cm)", "%.1f", totalDistance * 100);

                            // Calculate lateral distance
                            telemetry.addData("Side (cm)", "%.1f", lateralX * 100);
                        }
                    } else {
                        telemetry.addData("Status", "No Target Found");
                    }
                    telemetry.update();
                    telemetry.update();
                }
            }
        }
    }
}