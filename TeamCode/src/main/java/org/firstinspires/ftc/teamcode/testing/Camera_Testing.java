package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
@Disabled
@TeleOp(name = "AprilTag Pose Tracking", group = "Testing")
public class Camera_Testing extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Initialize the processor
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagID(true)
                .build();

        // Initialize the portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Wait for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            if (currentDetections.size() > 0) {
                for (AprilTagDetection detection : currentDetections) {
                    // Check if pose data is available (it might be null if tag is partially obscured)
                    if (detection.ftcPose != null) {
                        telemetry.addLine(String.format("--- Tag ID %d ---", detection.id));
                        telemetry.addData("Distance (Range)", "%.2f in", detection.ftcPose.range);
                        telemetry.addData("X Offset (Lateral)", "%.2f in", detection.ftcPose.x);
                        telemetry.addData("Y Offset (Forward)", "%.2f in", detection.ftcPose.y);
                        telemetry.addData("Z Offset (Height)", "%.2f in", detection.ftcPose.z);
                        telemetry.addData("Yaw (Angle)", "%.2f deg", detection.ftcPose.yaw);
                    } else {
                        telemetry.addLine(String.format("Tag ID %d detected, but pose is unavailable", detection.id));
                    }
                }
            } else {
                telemetry.addLine("No tags detected");
            }

            telemetry.update();
            sleep(20); // Small delay to prevent telemetry spam
        }
    }
}