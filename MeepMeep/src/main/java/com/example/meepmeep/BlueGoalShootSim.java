package com.example.meepmeep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;


public class BlueGoalShootSim {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(54,40, Math.toRadians(180),Math.toRadians(180),17)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-49.4,49.3,Math.toRadians(-45)))
                        .waitSeconds(5)
                .setTangent(Math.toRadians(180))
                // Move to (0,13) while rotating to face the field (180 degrees)
                // .splineToLinearHeading(new Pose2d(0, -13, Math.toRadians(180)), Math.toRadians(180))
                // .splineToLinearHeading(new Pose2d(-22, -29, Math.toRadians(180)), Math.toRadians(180))
                .strafeTo(new Vector2d(-12,12))

                // .turn(Math.toRadians(-133.67), preciseTurn)
                .waitSeconds(0.3)
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

                .strafeTo(new Vector2d(5, 22))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95F)
                .addEntity(myBot)
                .start();
    }
}