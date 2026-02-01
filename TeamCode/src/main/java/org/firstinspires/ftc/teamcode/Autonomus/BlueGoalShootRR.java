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

@Autonomous(name = "Blue Goal Shoot RR", group = "Autonomous")
public class BlueGoalShootRR extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d setStartPose = new Pose2d(-50.4,-50.3,Math.toRadians(45));

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
                .strafeTo(new Vector2d(-12,-12), fastVel,fastAccel)

                .stopAndAdd(drive.shooterOn())
                .waitSeconds(0.3)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(4)
                .stopAndAdd(drive.stopAll())
                .strafeTo(new Vector2d(-12,-12))

                .turn(Math.toRadians(225), preciseTurn)

                .stopAndAdd(drive.intakeOn())
                .strafeTo(new Vector2d(-12, -49), intakeVel, new ProfileAccelConstraint(-10, 10))
                .stopAndAdd(drive.intakeOff())

                .strafeTo(new Vector2d(-12, -12), fastVel, fastAccel)

                .stopAndAdd(drive.shooterOn())
                .turn(Math.toRadians(135), preciseTurn)
                .waitSeconds(0.3)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(4)
                .stopAndAdd(drive.stopAll())

                .strafeTo(new Vector2d(5, -22))
                .build();

        Actions.runBlocking(trajectory);
    }
}