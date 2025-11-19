package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvPipeline;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

import java.util.List;

@Autonomous(name = "AprilTag + Robust Ball Color Detection", group = "Vision")
public class ballcolorident extends LinearOpMode {

    OpenCvWebcam webcam;
    AprilTagAndColorPipeline pipeline;

    @Override
    public void runOpMode() {
        int camMonitorViewId = hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"), camMonitorViewId);

        pipeline = new AprilTagAndColorPipeline();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 480);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.log().add("Error opening camera: " + errorCode);
            }
        });

        telemetry.addLine("Camera starting...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("AprilTags Detected", pipeline.getAprilTagCount());
            telemetry.addData("Detected Ball Color", pipeline.getDetectedColor());
            telemetry.update();

            sleep(50);
        }

        webcam.stopStreaming();
    }


    // ----- Pipeline that detects AprilTags and ball color -----
    class AprilTagAndColorPipeline extends OpenCvPipeline {

        private AprilTagProcessor aprilTagProcessor;
        private List<AprilTagDetection> currentDetections;

        private String detectedColor = "UNKNOWN";

        // Adjust ROI location and size as needed for your camera & ball position
        private final Rect colorRegion = new Rect(280, 200, 80, 80);

        public AprilTagAndColorPipeline() {
            aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        }

        public int getAprilTagCount() {
            if (currentDetections == null) return 0;
            return currentDetections.size();
        }

        public String getDetectedColor() {
            return detectedColor;
        }

        @Override
        public Mat processFrame(Mat input) {
            // Detect AprilTags
            currentDetections = aprilTagProcessor.getDetections( );

            // Convert frame to HSV for color detection
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Validate ROI bounds
            Rect roi = colorRegion;
            if (roi.x + roi.width > input.cols() || roi.y + roi.height > input.rows()) {
                detectedColor = "OUT OF BOUNDS";
                hsv.release();
                return input;
            }

            Mat roiMat = hsv.submat(roi);

            // Create masks for red (two hue ranges) and blue colors
            Mat maskRed1 = new Mat();
            Mat maskRed2 = new Mat();
            Core.inRange(roiMat, new Scalar(0, 100, 50), new Scalar(10, 255, 255), maskRed1);
            Core.inRange(roiMat, new Scalar(170, 100, 50), new Scalar(180, 255, 255), maskRed2);
            Core.bitwise_or(maskRed1, maskRed2, maskRed1);

            Mat maskBlue = new Mat();
            Core.inRange(roiMat, new Scalar(100, 150, 50), new Scalar(130, 255, 255), maskBlue);

            // Clean up masks with morphological operations
            Imgproc.erode(maskRed1, maskRed1, new Mat());
            Imgproc.dilate(maskRed1, maskRed1, new Mat());

            Imgproc.erode(maskBlue, maskBlue, new Mat());
            Imgproc.dilate(maskBlue, maskBlue, new Mat());

            // Calculate ratio of detected color pixels
            double totalPixels = roiMat.rows() * roiMat.cols();
            double redRatio = Core.countNonZero(maskRed1) / totalPixels;
            double blueRatio = Core.countNonZero(maskBlue) / totalPixels;

            // Determine color based on ratio thresholds (adjust 0.3 as needed)
            if (redRatio > 0.3) {
                detectedColor = "RED";
            } else if (blueRatio > 0.3) {
                detectedColor = "BLUE";
            } else {
                detectedColor = "UNKNOWN";
            }

            // Draw green rectangle around ROI
            Imgproc.rectangle(input, roi, new Scalar(0, 255, 0), 2);

            // Draw detected AprilTags on screen
            if (currentDetections != null) {
                for (AprilTagDetection det : currentDetections) {
                    Point center = new Point(det.center.x, det.center.y);
                    Imgproc.circle(input, center, 10, new Scalar(255, 0, 0), 3);

                    String label = "ID: " + det.id;
                    Imgproc.putText(input, label, new Point(det.center.x + 15, det.center.y),
                            Imgproc.FONT_HERSHEY_SIMPLEX, 0.5, new Scalar(255, 0, 0), 2);
                }
            }

            // Release mats to avoid memory leaks
            maskRed1.release();
            maskRed2.release();
            maskBlue.release();
            roiMat.release();
            hsv.release();

            return input;
        }
    }
}
