package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
@Disabled
@TeleOp(name = "Manual Track Width Calibrator", group = "Testing")
public class TrackWidthCalibration extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize drive with your current pose
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("1. Align robot with a field tile line.");
        telemetry.addLine("2. Press START.");
        telemetry.addLine("3. Spin the robot manually (or with stick) exactly 10 times.");
        telemetry.addLine("4. The 'Calculated TrackWidth' will appear.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            // We calculate the track width based on the heading change
            // Formula: TrackWidth = (Average Wheel Ticks * inPerTick) / Total Radians Turned

            telemetry.addData("Current Heading (Deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Current Heading (Rad)", currentPose.heading.toDouble());
            telemetry.addLine("---------------------------");
            telemetry.addLine("After 10 full rotations (62.83 radians):");
            telemetry.addLine("Your trackWidthTicks should be adjusted until");
            telemetry.addLine("the Heading shows exactly 3600 degrees.");
            telemetry.update();
        }
    }
}