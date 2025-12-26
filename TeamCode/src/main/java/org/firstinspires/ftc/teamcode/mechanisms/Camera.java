package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

@TeleOp(name = "Limelight3A All Data Sensing", group = "Sensors")
public class Camera extends OpMode {

    private Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();           // begin polling
        limelight.setPollRateHz(50); // update ~50Hz

        telemetry.addLine("Limelight 3A initialized. Ready to sense data.");
        telemetry.update();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        if (result != null) {

            // 2D targeting values
            boolean valid = result.isValid();
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            int pipeIndex = result.getPipelineIndex();

            telemetry.addData("Valid Target?", valid);
            telemetry.addData("Pipeline Index", pipeIndex);

            if (valid) {
                telemetry.addData("tx (deg)", tx);
                telemetry.addData("ty (deg)", ty);
                telemetry.addData("ta (%)", ta);
            } else {
                telemetry.addLine("No valid target — tx/ty/ta unreliable");
            }

            // =======================
            // Botpose (3D robot pose)
            // =======================
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                YawPitchRollAngles orient = botpose.getOrientation();

                telemetry.addLine("=== Botpose 3D ===");
                telemetry.addData("X (m)", botpose.getPosition().x);
                telemetry.addData("Y (m)", botpose.getPosition().y);
                telemetry.addData("Z (m)", botpose.getPosition().z);
                telemetry.addData("Yaw (deg)", Math.toDegrees(orient.getYaw()));
                telemetry.addData("Pitch (deg)", Math.toDegrees(orient.getPitch()));
                telemetry.addData("Roll (deg)", Math.toDegrees(orient.getRoll()));
            } else {
                telemetry.addLine("Botpose not available.");
            }

            // =======================
            // AprilTag (Fiducial) Data
            // =======================
            List<FiducialResult> fidResults = result.getFiducialResults();
            if (fidResults != null && !fidResults.isEmpty()) {

                telemetry.addData("AprilTags Seen", fidResults.size());
                int count = 0;

                for (FiducialResult fr : fidResults) {
                    count++;
                    telemetry.addLine("=== Tag " + count + " ===");
                    telemetry.addData("Tag ID", fr.getFiducialId());

                    // Robot pose estimate from this tag
                    Pose3D roboPoseField = fr.getRobotPoseFieldSpace();
                    telemetry.addData("Robot X (m)", roboPoseField.getPosition().x);
                    telemetry.addData("Robot Y (m)", roboPoseField.getPosition().y);
                    telemetry.addData("Robot Z (m)", roboPoseField.getPosition().z);

                    // Tag pose relative to the camera
                    Pose3D tagPoseCam = fr.getTargetPoseCameraSpace();

                    double x = tagPoseCam.getPosition().x;
                    double y = tagPoseCam.getPosition().y;
                    double z = tagPoseCam.getPosition().z;

                    // Straight-line distance from camera to AprilTag
                    double distanceMeters = Math.sqrt(x * x + y * y + z * z);

                    telemetry.addData("Tag X from Cam (m)", x);
                    telemetry.addData("Tag Y from Cam (m)", y);
                    telemetry.addData("Tag Z from Cam (m)", z);
                    telemetry.addData("Distance to Tag (m)", distanceMeters);
                }

            } else {
                telemetry.addLine("No fiducial (AprilTag) results found.");
            }

        } else {
            telemetry.addLine("No result (result == null).");
        }

        telemetry.update();
    }
}
