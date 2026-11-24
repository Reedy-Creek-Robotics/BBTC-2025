package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "AprilTag Turret Tracking", group = "Testing")
public class Camera_Testing extends LinearOpMode {

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DcMotor turretMotor; // Declare your turret motor

    // Tuning constants for the turret control loop
    final double KP = 0.02;       // Proportional gain (adjust based on your motor speed/gearing)
    final double MAX_POWER = 0.5; // Max power limit to prevent the turret from spinning too fast

    @Override
    public void runOpMode() {
        // Initialize the turret motor (make sure "turret" matches your Robot Config name)
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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
            boolean tagFound = false;

            for (AprilTagDetection detection : currentDetections) {
                if (detection.ftcPose == null) {
                    continue;
                }

                // The pose refers to the tag center or, for a cluster, its defined origin.
                double error = detection.ftcPose.bearing;
                double motorPower = Range.clip(error * KP, -MAX_POWER, MAX_POWER);
                turretMotor.setPower(motorPower);
                tagFound = true;

                if (detection instanceof AprilTagSingleDetection) {
                    AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                    telemetry.addData("Tracking", "Tag ID %d", tag.id);
                } else if (detection instanceof AprilTagClusterDetection) {
                    AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                    telemetry.addData("Tracking cluster", cluster.metadata.name);
                }
                telemetry.addData("Bearing (Error)", "%.2f deg", error);
                telemetry.addData("Motor Power", "%.2f", motorPower);
                telemetry.addData("Distance", "%.2f in", detection.ftcPose.range);
                break; // Track the first detection with a pose.
            }

            // If no tags are visible, stop the turret motor
            if (!tagFound) {
                turretMotor.setPower(0);
                telemetry.addLine("No tags detected - Turret Idle");
            }

            telemetry.update();
        }
    }
}
