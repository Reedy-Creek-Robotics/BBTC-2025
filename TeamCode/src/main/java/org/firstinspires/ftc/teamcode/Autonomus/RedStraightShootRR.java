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

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;

import java.util.Arrays;

@Autonomous(name = "Red Straight Shoot Roadrunner", group = "Autonomous")
public class RedStraightShootRR extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(62, 9, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        // Define a very slow speed for collecting blocks
        MinVelConstraint intakeVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(10.0), // 10 inches per second (very slow)
                new AngularVelConstraint(Math.PI / 4)
        ));
        ProfileAccelConstraint intakeAccel = new ProfileAccelConstraint(-10.0, 10.0);

        // Define a "Slow and Precise" constraint for turns and approaches
        MinVelConstraint slowVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(20.0),
                new AngularVelConstraint(Math.PI / 2)
        ));
        ProfileAccelConstraint slowAccel = new ProfileAccelConstraint(-15.0, 15.0);

        // Slow down the turn speed specifically (Max Ang Vel, Min Ang Accel, Max Ang Accel)
        TurnConstraints preciseTurn = new TurnConstraints(2.0, -2.0, 2.0);
        TurnConstraints fastTurn = new TurnConstraints(5.0,-5.0,5.0);

        Action trajectory = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(0, 13), Math.toRadians(180))
                .splineTo(new Vector2d(-34, 34), Math.toRadians(180))

                // 1. Slow down this massive turn to stop the over-rotation
                .turn(Math.toRadians(-230), preciseTurn)
                .waitSeconds(0.3) // Give it a moment to settle the "tilt" before shooting
                .stopAndAdd(drive.shootAction())

                // 2. Approach intake slowly to prevent overshooting the block
               // .splineTo(new Vector2d(-24, 10), Math.toRadians(0), slowVel, slowAccel)
                .strafeTo(new com.acmerobotics.roadrunner.Vector2d(-13.6, 14.6))
                .turn(Math.toRadians(-225), fastTurn)


                .stopAndAdd(drive.intakeOn())


              //  .splineTo(new Vector2d(-12, 34), Math.toRadians(90), slowVel, slowAccel)

                .strafeTo(new Vector2d(-12, 53),intakeVel,intakeAccel)
                .stopAndAdd(drive.intakeOff())

                .strafeTo(new Vector2d(-12, 34))
                .strafeTo(new Vector2d(-34, 34))

                // 3. Slow down the return turn for the final shot
                .turn(Math.toRadians(230), preciseTurn)
                .waitSeconds(0.3)
                .stopAndAdd(drive.shootAction())

                .strafeTo(new Vector2d(5, 22))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(trajectory);
    }
}