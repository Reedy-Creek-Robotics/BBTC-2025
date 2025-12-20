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
public class AprilTagAutoAlignFTC extends OpMode {

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

            // 2D basic values — always available if pipeline sees a target
            double tx = result.getTx(); // horizontal angle
            double ty = result.getTy(); // vertical angle
            double ta = result.getTa(); // target area
            int pipeIndex = result.getPipelineIndex();
            boolean valid = result.isValid();

            telemetry.addData("Valid Target?", valid);
            telemetry.addData("Pipeline Index", pipeIndex);
            telemetry.addData("tx (deg)", tx);
            telemetry.addData("ty (deg)", ty);
            telemetry.addData("ta (%)", ta);

            // 3D bot pose — only useful if you're using a botpose pose‑estimation pipeline
            Pose3D botpose = result.getBotpose();
            YawPitchRollAngles orient = botpose.getOrientation();

            telemetry.addLine("=== Botpose 3D ===");
            telemetry.addData("X (m)", botpose.getPosition().x);
            telemetry.addData("Y (m)", botpose.getPosition().y);
            telemetry.addData("Z (m)", botpose.getPosition().z);
            telemetry.addData("Yaw (deg)", Math.toDegrees(orient.getYaw()));
            telemetry.addData("Pitch (deg)", Math.toDegrees(orient.getPitch()));
            telemetry.addData("Roll (deg)", Math.toDegrees(orient.getRoll()));

            // Per‑tag detailed results via FiducialResult list
            List<FiducialResult> fidResults = result.getFiducialResults();
            if (fidResults != null && !fidResults.isEmpty()) {
                int count = 0;
                for (FiducialResult fr : fidResults) {
                    count++;
                    telemetry.addLine("=== Tag " + count + " ===");
                    telemetry.addData("Tag ID", fr.getFiducialId());                 // get the AprilTag ID
                    Pose3D roboPoseField = fr.getRobotPoseFieldSpace();              // robot pose from this tag
                    telemetry.addData("Tag Robot X", roboPoseField.getPosition().x);
                    telemetry.addData("Tag Robot Y", roboPoseField.getPosition().y);
                    telemetry.addData("Tag Robot Z", roboPoseField.getPosition().z);
                }
            } else {
                telemetry.addLine("No fiducial (AprilTag) results list found.");
            }

        } else {
            telemetry.addLine("No result (result == null).");
        }

        telemetry.update();
    }
}
