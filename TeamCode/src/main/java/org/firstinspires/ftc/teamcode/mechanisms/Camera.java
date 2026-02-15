package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class Camera {
    private Limelight3A limelight;
    private int tid = -1;
    private double distance = -1;
    private double area = -1;
    public Camera(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(50);

        limelight.start();

    }
    // ... (constructor and start methods remain the same)

    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            tid = -1;
            distance = -1;
            area = -1;
            return;
        }

        List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();

        if (fidResults != null && !fidResults.isEmpty()) {
            LLResultTypes.FiducialResult tag = fidResults.get(0);
            tid = tag.getFiducialId();
            area = tag.getTargetArea(); // This is a 0-100 value

            Pose3D pose = tag.getTargetPoseCameraSpace();
            // Using the Z-axis is often more stable for "straight ahead" distance
            // but the hypotenuse (below) is fine for total distance
            double x = pose.getPosition().x;
            double y = pose.getPosition().y;
            double z = pose.getPosition().z;

            distance = Math.sqrt(x*x + y*y + z*z) * 39.37; // Meters to Inches
        } else {
            tid = -1;
            distance = -1;
            area = -1;
        }
    }

    public int getTid() { return tid; }

    // Returns the 3D calculated distance
    public double getDistance() { return distance; }

    // Returns the area percentage (0-100)
    public double getArea() { return area; }
}