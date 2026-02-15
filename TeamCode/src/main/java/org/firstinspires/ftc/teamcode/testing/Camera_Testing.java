package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;
//@Disabled
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
                    if (detection.ftcPose != null) {
                        // Calculate Area: (Right - Left) * (Bottom - Top)
                        double tagWidth = detection.corners[1].x - detection.corners[0].x;
                        double tagHeight = detection.corners[2].y - detection.corners[0].y;
                        double area = Math.abs(tagWidth * tagHeight);

                        telemetry.addLine(String.format("--- Tag ID %d ---", detection.id));
                        telemetry.addData("Area", "%.2f px^2", area); // Area in pixels
                        telemetry.addData("Distance", "%.2f in", detection.ftcPose.range);
                        //telemetry.addData("Yaw", "%.2f deg", detection.ftcPose.yaw);
                    }
                }
            } else {
                telemetry.addLine("No tags detected");
            }
            telemetry.update();
        }
    }
}