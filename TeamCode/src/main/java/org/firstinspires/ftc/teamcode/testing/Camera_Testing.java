package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.HashMap;
import java.util.Map;

@TeleOp(name = "AprilTag Color Mapping", group = "Testing")
public class Camera_Testing extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Map tag IDs to “color codes” or actions
    private final Map<Integer, String> tagColorMap = new HashMap<Integer, String>() {{
        put(1, "Red");
        put(2, "Blue");
        put(3, "Green");
        put(4, "Yellow");
        put(5, "Purple");
    }};

    @Override
    public void runOpMode() {

        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("AprilTag system initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.clear();
            boolean tagFound = false;

            for (AprilTagDetection detection : aprilTag.getDetections()) {

                int id = detection.id;

                if (tagColorMap.containsKey(id)) {

                    double distanceInches = detection.ftcPose.range;
                    String color = tagColorMap.get(id);

                    telemetry.addLine("AprilTag detected");
                    telemetry.addData("Tag ID", id);
                    telemetry.addData("Distance (in)", "%.1f", distanceInches);
                    telemetry.addData("Color Code", color);

                    tagFound = true;
                    break; // Only show first recognized tag
                }
            }

            if (!tagFound) {
                telemetry.addLine("No recognized tag visible");
            }

            telemetry.update();
        }
    }
}
