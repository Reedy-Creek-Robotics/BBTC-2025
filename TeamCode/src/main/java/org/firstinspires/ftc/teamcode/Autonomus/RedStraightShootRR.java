package org.firstinspires.ftc.teamcode.Autonomus;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.Camera;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "Red Straight Shoot RR", group = "Autonomous")
public class RedStraightShootRR extends LinearOpMode {

    @Override
    public void runOpMode() {
        Camera camera = new Camera(hardwareMap);

        // Final coordinates sourced from the camera
      /*  Pose2d initialPose = null;

        // --- INIT PHASE: SOURCE OF TRUTH ---
        while (!isStarted() && !isStopRequested()) {
            camera.update();
            Pose2d visionPose = camera.getFieldPose();

            if (visionPose != null) {
                initialPose = visionPose;
                telemetry.addData("Status", "CAMERA LOCKED");
            } else {
                telemetry.addData("Status", "SEARCHING FOR TAG...");
            }

            // Fallback display if not locked yet
            Pose2d displayPose = (initialPose != null) ? initialPose : new Pose2d(62, 9, 0);
            telemetry.addData("X", "%.2f", displayPose.position.x);
            telemetry.addData("Y", "%.2f", displayPose.position.y);
            telemetry.addData("Heading", "%.1f deg", Math.toDegrees(displayPose.heading.toDouble()));
            telemetry.update();
        }

        if (isStopRequested() || initialPose == null) return;

        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);*/

        Pose2d setStartPose = new Pose2d(62,12,Math.toRadians(180));

        MecanumDrive drive = new MecanumDrive(hardwareMap, setStartPose);
        waitForStart();

        // Define Constraints
        MinVelConstraint intakeVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0), new AngularVelConstraint(Math.PI / 4)));
        MinVelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(80.0), new AngularVelConstraint(Math.PI / 2)));
        ProfileAccelConstraint fastAccel = new ProfileAccelConstraint(-30.0, 30.0);
        TurnConstraints preciseTurn = new TurnConstraints(3.0, -2.0, 2.0);

        // 3. Build Trajectory
        Action trajectory = drive.actionBuilder(setStartPose)
                // setTangent(180) tells the robot to move toward the center of the field
                .setTangent(Math.toRadians(180))
                // Move to (0,13) while rotating to face the field (180 degrees)
                //.splineToLinearHeading(new Pose2d(0, 13, Math.toRadians(180)), Math.toRadians(180))
                //.splineToLinearHeading(new Pose2d(-22, 29, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-12,12), fastVel,fastAccel)

                .stopAndAdd(drive.shooterOn())
                .turn(Math.toRadians(-135), preciseTurn)
                .waitSeconds(0.3)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(3.5)//4
                .stopAndAdd(drive.intakeOff())
                .stopAndAdd(drive.stopAll())

              //  .strafeTo(new Vector2d(-13.6, 14.6), fastVel, fastAccel)
                .turn(Math.toRadians(-225), preciseTurn)

                .stopAndAdd(drive.intakeOn())
                .strafeTo(new Vector2d(-12, 53), intakeVel, new ProfileAccelConstraint(-10, 10))
                .stopAndAdd(drive.intakeOff())

//                .strafeTo(new Vector2d(-12, 34), fastVel, fastAccel)
                .strafeTo(new Vector2d(-12, 12), fastVel, fastAccel)

                .stopAndAdd(drive.shooterOn())
                .turn(Math.toRadians(135), preciseTurn)
                .waitSeconds(0.3)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(3.5)//4
                .stopAndAdd(drive.stopAll())

                .strafeTo(new Vector2d(5, 22))
                .build();

        Actions.runBlocking(trajectory);
        camera.stop();
    }
}