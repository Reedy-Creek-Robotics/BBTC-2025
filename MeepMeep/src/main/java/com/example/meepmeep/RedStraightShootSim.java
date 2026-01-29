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

                // Turn and Shoot 1
                .turn(Math.toRadians(-45))
                .waitSeconds(0.3)

                // Navigate to Intake
                .strafeTo(new Vector2d(-13.6, 14.6))
                .turn(Math.toRadians(-225))

                // Start Intake

                // Slow crawl to pick up the block/ring
                .strafeTo(new Vector2d(-12, 53))

                // Stop Intake

                // Return to shooting position
                .strafeTo(new Vector2d(-12, 34))
                .strafeTo(new Vector2d(-34, 34))

                // Turn and Shoot 2
                .turn(Math.toRadians(230))
                .waitSeconds(0.3)

                // Park
                .strafeTo(new Vector2d(5, 22))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95F)
                .addEntity(myBot)
                .start();
    }
}