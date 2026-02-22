package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class Camera {
    private Limelight3A limelight;

    // Tracking variables
    private int tid = -1;
    private double distance = -1;
    private double area = -1;
    private double tx = 0;
    private double ty = 0;
    private double yaw = 0;

    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }

    /**
     * Switches the active pipeline.
     * @param index The ID of the pipeline (0-9) as set in the Limelight web UI.
     */
    public void setPipelineBlue() {
        limelight.pipelineSwitch(0);
    }
    public void setPipelineRed() {
        limelight.pipelineSwitch(1);
    }
    public void setPipelineUseless() {
        limelight.pipelineSwitch(2);
    }

    public void update() {
        LLResult result = limelight.getLatestResult();

        // If no valid target is seen, reset everything
        if (result == null || !result.isValid()) {
            resetTracking();
            return;
        }

        // 2D Tracking Data
        this.tx = result.getTx();
        this.ty = result.getTy();
        this.area = result.getTa();

        List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();

        if (fidResults != null && !fidResults.isEmpty()) {
            LLResultTypes.FiducialResult tag = fidResults.get(0);
            tid = tag.getFiducialId();

            // 3D Pose Data
            Pose3D pose = tag.getTargetPoseCameraSpace();

            // Distance calculation
            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;
            distance = Math.sqrt(x*x + y*y + z*z) * 39.37; // Meters to Inches

            // Rotation (Yaw)
            this.yaw = pose.getOrientation().getYaw();
        } else {
            resetTracking();
        }
    }

    private void resetTracking() {
        tid = -1;
        distance = -1;
        area = -1;
        tx = 0;
        ty = 0;
        yaw = 0;
    }

    // --- Getters ---

    public int getTid() { return tid; }
    public double getDistance() { return distance; }
    public double getArea() { return area; }
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getYaw() { return yaw; }
    public boolean hasTarget() { return tid != -1; }
}