package org.firstinspires.ftc.teamcode.Autonomus;

import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.fastAccelMaxAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.fastAccelMinAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.fastVelMinTransVel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.intakeMaxAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.intakeMinAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.intakeVelMinTransVel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.preciseTurnMaxAngAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.preciseTurnMaxAngVel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.preciseTurnMinAngAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad1_x_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad1_y_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad2_x_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad2_y_sign;

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

@Autonomous(name = "Red Goal Shoot RR", group = "Autonomous")
public class RedGoalShootRR extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d setStartPose = new Pose2d((quad2_x_sign)*50.4,(quad2_y_sign)*50.3,Math.toRadians(-45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, setStartPose);

        // Define Constraints
        MinVelConstraint intakeVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(intakeVelMinTransVel), new AngularVelConstraint(Math.PI / 4)));
        MinVelConstraint fastVel = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(fastVelMinTransVel), new AngularVelConstraint(Math.PI / 2)));
        ProfileAccelConstraint fastAccel = new ProfileAccelConstraint(fastAccelMinAccel, fastAccelMaxAccel);
        TurnConstraints preciseTurn = new TurnConstraints(preciseTurnMaxAngVel, preciseTurnMinAngAccel, preciseTurnMaxAngAccel);

        waitForStart();

        // Build Trajectory
        Action trajectory = drive.actionBuilder(setStartPose)
                // setTangent(180) tells the robot to move toward the center of the field
                .setTangent(Math.toRadians(180))
                .strafeTo(new Vector2d((quad2_x_sign)*12,(quad2_y_sign)*12), fastVel, fastAccel)

                .stopAndAdd(drive.shooterOn())
                .waitSeconds(1)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(4)
                .stopAndAdd(drive.stopAll())
                .strafeTo(new Vector2d((quad2_x_sign)*12,(quad2_y_sign)*12))

                .turn(Math.toRadians(-225), preciseTurn)

                .stopAndAdd(drive.intakeOn())
                .strafeTo(new Vector2d((quad2_x_sign)*12, (quad2_y_sign)*49), intakeVel, new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel))
                .stopAndAdd(drive.intakeOff())

                .strafeTo(new Vector2d((quad2_x_sign)*12,(quad2_y_sign)*12), fastVel, fastAccel)

                .stopAndAdd(drive.shooterOn())
                .turn(Math.toRadians(-135), preciseTurn)
                .waitSeconds(1)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(4)
                .stopAndAdd(drive.stopAll())

                .strafeTo(new Vector2d((quad1_x_sign)*5, (quad1_y_sign)*22))
                .build();

        Actions.runBlocking(trajectory);
    }
}