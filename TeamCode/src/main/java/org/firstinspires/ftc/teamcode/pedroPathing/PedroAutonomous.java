package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomous extends OpMode {
    private static final Pose START_POSE = new Pose(56, 8, Math.toRadians(90));
    private static final double MAX_POWER = 0.4;

    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private PathChain mainChain;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        // Use the same pose for the robot's starting location and the first point on the path.
        follower.setStartingPose(START_POSE);
        mainChain = follower.pathBuilder()
                .addPath(new BezierCurve(
                        START_POSE,
                        new Pose(83.974, 54.881),
                        new Pose(35.723, 83.167)
                ))
                .setLinearHeadingInterpolation(START_POSE.getHeading(), Math.toRadians(180))
                .build();

        panelsTelemetry.debug("Status", "Place robot at (56, 8), heading 90 degrees");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        follower.setMaxPower(MAX_POWER);
        // Pressing Play starts the path once. false tells Pedro to release the endpoint.
        follower.followPath(mainChain, false);
    }

    @Override
    public void loop() {
        // update() reads the sensors and drives the robot along the active path.
        follower.update();

        panelsTelemetry.debug("Path", follower.isBusy() ? "Following" : "Follower idle");
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void stop() {
        follower.breakFollowing();
    }
}


