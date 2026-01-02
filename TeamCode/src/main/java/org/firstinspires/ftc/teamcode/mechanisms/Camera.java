package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Camera {

    private Limelight3A limelight;

    private double tx, ty, ta;
    private int tid = -1;
    private double distance = -1;

    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50);
        limelight.start();
    }

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null) return;

        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();

        List<LLResultTypes.FiducialResult> fidResults =
                result.getFiducialResults();

        if (fidResults != null && !fidResults.isEmpty()) {
            LLResultTypes.FiducialResult tag = fidResults.get(0);
            tid = tag.getFiducialId();

            Pose3D pose = tag.getTargetPoseCameraSpace();
            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            distance = Math.sqrt(x*x + y*y + z*z) - 0.2;
        } else {
            tid = -1;
            distance = -1;
        }
    }
    public void enable(){
        limelight.start();
    }
    public void disable(){
        limelight.stop();
    }

    // ----- Getters -----
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public int getTid() { return tid; }
    public double getDistance() { return distance; }

    public void stop() {
        limelight.stop();
    }
}
