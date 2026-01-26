package com.example.meepmeep;

import static java.lang.Thread.sleep;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;




public class RedStraightShootSim  {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(54,40, Math.toRadians(180),Math.toRadians(180),17)
                .build();
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62,9,Math.toRadians(180)))
                        .waitSeconds(5)
                .splineTo(new Vector2d(0, 13), Math.toRadians(180))
                .splineTo(new Vector2d(-34, 34), Math.toRadians(180))

                // 1. Slow down this massive turn to stop the over-rotation
                .turn(Math.toRadians(-230))
                .waitSeconds(0.3) // Give it a moment to settle the "tilt" before shooting

                // 2. Approach intake slowly to prevent overshooting the block
                // .splineTo(new Vector2d(-24, 10), Math.toRadians(0), slowVel, slowAccel)
                .strafeTo(new com.acmerobotics.roadrunner.Vector2d(-13.6, 14.6))
                .turn(Math.toRadians(-225))

                //  .splineTo(new Vector2d(-12, 34), Math.toRadians(90), slowVel, slowAccel)

                .strafeTo(new Vector2d(-12, 53))

                .strafeTo(new Vector2d(-12, 34))
                .strafeTo(new Vector2d(-34, 34))

                // 3. Slow down the return turn for the final shot
                .turn(Math.toRadians(230))
                .waitSeconds(0.3)

                .strafeTo(new Vector2d(5, 22))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95F)
                .addEntity(myBot)
                .start();
    }
}