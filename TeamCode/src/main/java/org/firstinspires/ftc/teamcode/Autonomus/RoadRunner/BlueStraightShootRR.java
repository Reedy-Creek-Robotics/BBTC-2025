package org.firstinspires.ftc.teamcode.Autonomus.RoadRunner;

import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.ballShootX;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.ballShootY;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.fastAccelMaxAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.fastAccelMinAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.fastVelMinTransVel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.intakeVelMinTransVel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.preciseTurnMaxAngAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.preciseTurnMaxAngVel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.preciseTurnMinAngAccel;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad3_x_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad3_y_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad4_x_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.quad4_y_sign;
import static org.firstinspires.ftc.teamcode.mechanisms.RR_RobotConstants.shootTimer;

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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.mechanisms.MecanumDriveClose;

import java.util.Arrays;
@Disabled
@Autonomous(name = "Blue Straight Shoot RR", group = "Autonomous")
public class BlueStraightShootRR extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d setStartPose = new Pose2d((quad4_x_sign)*62,(quad4_y_sign)*12,Math.toRadians(180));
        MecanumDriveClose drive = new MecanumDriveClose(hardwareMap, setStartPose);

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
                .strafeTo(new Vector2d((quad4_x_sign)*(55),(quad4_y_sign)*(10)), fastVel, fastAccel)
                //.stopAndAdd(drive.shooterStraightOn())
                .turn(Math.toRadians(-156))//precise turn
                .waitSeconds(0.3)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(shootTimer)
                .stopAndAdd(drive.stopAll())

                //Get first line
                .turn(Math.toRadians(156))//precise turn
                .strafeTo(new Vector2d((quad4_x_sign)*(36),(quad4_y_sign)*(10)))
                .turn(Math.toRadians(90))//precise turn
                .stopAndAdd(drive.intakeSSOn())
                .strafeTo(new Vector2d((quad4_x_sign)*(35), (quad4_y_sign)*(61)), fastVel, fastAccel)//, intakeVel, new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                .stopAndAdd(drive.intakeOff())
                .strafeTo(new Vector2d((quad4_x_sign)*(55),(quad4_y_sign)*(10)), fastVel, fastAccel)
                //  .stopAndAdd(drive.shooterStraightOn())
                .turn(Math.toRadians(115))//precise turn
                .waitSeconds(0.3)
                .stopAndAdd(drive.transferOn())
                .waitSeconds(shootTimer)
                .stopAndAdd(drive.stopAll())

                //Get second line
                .turn(Math.toRadians(-115))
                .strafeTo(new Vector2d((quad4_x_sign)*(12),(quad4_y_sign)*(10)))
                .stopAndAdd(drive.intakeSSOn())
                .strafeTo(new Vector2d((quad4_x_sign)*(12),(quad4_y_sign)*(61)))
                .stopAndAdd(drive.intakeOff())
                .strafeTo(new Vector2d((quad4_x_sign)*(55),(quad4_y_sign)*(10)))
                .turn(Math.toRadians(115))
                .waitSeconds(0.3)
                //.stopAndAdd(drive.shooterStraightOn())
                .stopAndAdd(drive.transferOn())
                .waitSeconds(shootTimer)
                .stopAndAdd(drive.stopAll())

                //Get third line
                .turn(Math.toRadians(-115))
                .strafeTo(new Vector2d((quad3_x_sign)*(12),(quad3_y_sign)*(10)))
                .stopAndAdd(drive.intakeSSOn())
                .strafeTo(new Vector2d((quad3_x_sign)*(12),(quad3_y_sign)*(56)))
                .stopAndAdd(drive.intakeOff())
                .strafeTo(new Vector2d((quad3_x_sign)*(ballShootX),(quad3_y_sign)*(ballShootY)))
                .turn(Math.toRadians(115))
                .waitSeconds(0.3)
                // .stopAndAdd(drive.shooterGoalOn())
                .stopAndAdd(drive.transferOn())
                .waitSeconds(shootTimer)
                .stopAndAdd(drive.stopAll())

                //Move out of scoring zone
                .strafeTo(new Vector2d((quad4_x_sign)*(43), (quad4_y_sign)*(23)))









                // .turn(Math.toRadians(-133.67), preciseTurn)
                /* .waitSeconds(0.3)
                 .waitSeconds(4)
                 .strafeTo(new Vector2d(-12,12))


                 //.strafeTo(new Vector2d(-13.6, -14.6), fastVel, fastAccel)
                 .turn(Math.toRadians(-225))

                 .strafeTo(new Vector2d(-12, 49))

 //                .strafeTo(new Vector2d(-12, -34), fastVel, fastAccel)
                 .strafeTo(new Vector2d(-12, 12))

                 .turn(Math.toRadians(-135))
                 .waitSeconds(0.3)
                 .waitSeconds(4)

                 .strafeTo(new Vector2d(5, 22))*/
                //.waitSeconds(3)

                .build();

        Actions.runBlocking(trajectory);
    }
}