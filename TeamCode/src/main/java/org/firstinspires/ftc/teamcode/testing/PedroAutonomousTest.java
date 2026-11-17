package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

// Pedro Pathing 3 Imports
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.paths.Path;
import com.pedropathing.follower.Follower;

import org.firstinspires.ftc.teamcode.pedroPathing.PedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable
public class PedroAutonomousTest extends OpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private PathsGroup paths;

    private static final PoseFactory p = PoseFactory.degrees();

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        // Initializes follower using your Constants.java setup
        follower = Constants.create(hardwareMap);
        if (follower != null) {
            follower.setPose(p.of(72, 8, 90));
        }

        paths = new PathsGroup();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        if (follower != null) {
            follower.update();
        }
        pathState = autonomousPathUpdate();

        panelsTelemetry.debug("Path State", pathState);
        if (follower != null) {
            panelsTelemetry.debug("X", follower.pose().x());
            panelsTelemetry.debug("Y", follower.pose().y());
            panelsTelemetry.debug("Heading", follower.pose().heading());
        }
        panelsTelemetry.update(telemetry);
    }

    public static class PathsGroup {
        public Path mainChain;

        public PathsGroup() {
            // Pedro 3 curve and line builder
            Path firstCurve = curve(
                    p.of(56.000, 8.000, 90),
                    p.of(83.974, 54.881, 135),
                    p.of(35.723, 83.167, 180)
            ).linear(p.of(56.000, 8.000, 90), p.of(35.723, 83.167, 180));

            mainChain = path(firstCurve);
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine here, e.g.:
        // switch(pathState) {
        //     case 0:
        //         follower.followPath(paths.mainChain);
        //         pathState = 1;
        //         break;
        // }
        return pathState;
    }
}