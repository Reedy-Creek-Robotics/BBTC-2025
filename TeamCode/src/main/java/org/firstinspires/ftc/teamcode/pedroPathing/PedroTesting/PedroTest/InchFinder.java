package org.firstinspires.ftc.teamcode.pedroPathing.PedroTesting.PedroTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp(name = "Inch Finder (Pinpoint Test)", group = "Pedro Test")
public class InchFinder extends LinearOpMode {

    private Follower follower;

    @Override
    public void runOpMode() {
        // Initialize Pedro Pathing with your Pinpoint Constants
        follower = Constants.createFollower(hardwareMap);

        // Optional: set origin pose to (0,0) with 0 degrees heading
        follower.setStartingPose(new Pose(0, 0, 0));

        telemetry.addData("Status", "Initialized. Press Play to test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update the Pedro follower loop to fetch new Pinpoint odometry readings
            follower.update();

            // Display current X and Y positions in inches directly to telemetry
            telemetry.addData("X Position (Inches)", follower.getPose().getX());
            telemetry.addData("Y Position (Inches)", follower.getPose().getY());
            telemetry.addData("Heading (Degrees)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}