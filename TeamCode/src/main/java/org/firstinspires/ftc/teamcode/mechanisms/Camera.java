package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Camera {

    private Limelight3A limelight;
    private int tid = -1;
    private double distance = -1;

    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }
    public void startCamera(){
        limelight.start();

    }
    public void switchToBlue(){
        limelight.pipelineSwitch(0); //April tag #20 pipeline
    }
    public void switchToRed(){
        limelight.pipelineSwitch(1); //April tag #24 pipeline
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            tid = -1;
            distance = -1;
            return;
        }

        List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();

        if (fidResults != null && !fidResults.isEmpty()) {
            LLResultTypes.FiducialResult tag = fidResults.get(0);
            tid = tag.getFiducialId();

            Pose3D pose = tag.getTargetPoseCameraSpace();
            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            // Distance formula (hypotenuse)
            distance = Math.sqrt(x*x + y*y + z*z) * 39.37;
        } else {
            tid = -1;
            distance = -1;
        }
    }

    // ----- Getters -----
    public int getTid() { return tid; }
    public double getDistance() { return distance; }
}