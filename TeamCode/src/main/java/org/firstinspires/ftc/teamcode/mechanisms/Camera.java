package org.firstinspires.ftc.teamcode.mechanisms;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;

public class Camera {

    private Limelight3A limelight;
//    private IMU imu;
    private double tx, ty, ta;
    private int tid = -1;
    private double distance = -1;

    // Field coordinates for Tag 20 (Blue Backdrop Center)
    private static final double TAG_X = -62.75;
    private static final double TAG_Y = 35.75;

    public Camera(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(50); //shud this be lower or not set here at all ? Not seen in Pratt video
        /*imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));*/
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

    /**
     * Calculates the Robot's global field position based on Tag 20.
     * Returns null if Tag 20 is not visible or data is invalid.
     */
    // Inside Camera.java
    public Pose2d getFieldPose() {
        /*YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());*/

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        /*Pose3D botPose = result.getBotpose_MT2();
        if (botPose != null) {
            // Convert to RoadRunner-friendly inches (Limelight is in Meters)
            double xInches = botPose.getPosition().x * 39.37;
            double yInches = botPose.getPosition().y * 39.37;
            double heading = Math.toRadians(botPose.getOrientation().getYaw());

            // Update RoadRunner
            return new Pose2d(xInches, yInches, heading);*/

        List<LLResultTypes.FiducialResult> fidResults = result.getFiducialResults();
        for (LLResultTypes.FiducialResult tag : fidResults) {
            // Tag 12 is the Blue Goal Center in many seasons,
            // verify your specific season ID!
            if (tag.getFiducialId() == 12 || tag.getFiducialId() == 20) {
                Pose3D pose = tag.getTargetPoseCameraSpace();
                double distToTag = pose.getPosition().z * 39.37;
                double xOffset = pose.getPosition().x * 39.37;
                // X Logic: Start at blue wall (-70.25) and move toward center (+X)
                double robotX = -70.25 + distToTag;
                // Y Logic:
                // If Limelight sees the tag to the RIGHT, xOffset is positive.
                // Since Blue Goal is at Y = 48, a robot to the right of it is at Y < 48.
                double robotY = 48.00 - xOffset;
                // HEADING:
                // If the robot is looking AT the Blue wall, it is facing 180 degrees.
                // If you want it to drive AWAY from the wall at the start of Auto,
                // your tangent in the Auto code should be 0 degrees.
                return new Pose2d(robotX, robotY, Math.toRadians(180));
            }
        }
        return null;
    }
    public void update() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            tid = -1;
            distance = -1;
            return;
        }

        tx = result.getTx();
        ty = result.getTy();
        ta = result.getTa();

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

    public void enable() { limelight.start(); }
    public void disable() { limelight.stop(); }
    public void stop() { limelight.stop(); }

    // ----- Getters -----
    public double getTx() { return tx; }
    public double getTy() { return ty; }
    public double getTa() { return ta; }
    public int getTid() { return tid; }
    public double getDistance() { return distance; }
}