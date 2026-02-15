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
        double quad4_x_sign = 1;
        double quad1_x_sign = 1;
        double quad1_y_sign = 1;
        double quad2_x_sign = -1;
        double quad2_y_sign = 1;
        double quad3_x_sign = -1;
        double quad3_y_sign = -1;
        double quad4_y_sign = -1;
        double intakeMinAccel = -5.0;
        double intakeMaxAccel = 5.0;
        double intakeVelMinTransVel = 10.0;
        double fastVelMinTransVel = 80.0;
        double fastAccelMinAccel = -30.0;
        double fastAccelMaxAccel = 30.0;
        double preciseTurnMaxAngVel = 3.0;
        double preciseTurnMinAngAccel = -2.0;
        double preciseTurnMaxAngAccel = 2.0;
        double ballCollectStartX = 12;
        double ballCollectStartY = 12;
        double ballCollectEndX = 12;
        double ballCollectEndY = 54;
        double ballShootX = 12;
        double ballShootY = 12;
        double moveOutX = 5;
        double moveOutY = 22;
        double shootTimer = 3.5;
      //  myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62,-12,Math.toRadians(20)))
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62,-14,Math.toRadians(180)))

                .setTangent(Math.toRadians(180))
                .waitSeconds(2)

                //.strafeTo(new Vector2d((quad4_x_sign)*(55),(quad4_y_sign)*(10)))
                //.turn(Math.toRadians(-156))
                .waitSeconds(0.3)
                .waitSeconds(shootTimer)
                //Get first line
                //.turn(Math.toRadians(156))
                .strafeTo(new Vector2d((quad4_x_sign)*(35),(quad4_y_sign)*(10)))
                .turn(Math.toRadians(-90))
                .strafeTo(new Vector2d((quad4_x_sign)*(35), (quad4_y_sign)*(61)))//, intakeVel, new ProfileAccelConstraint(intakeMinAccel, intakeMaxAccel)
                .strafeTo(new Vector2d((quad4_x_sign)*(55),(quad4_y_sign)*(10)))
                .turn(Math.toRadians(115))
                .waitSeconds(0.3)
                .waitSeconds(shootTimer)

                //Get second line
                .turn(Math.toRadians(-115))
                .strafeTo(new Vector2d((quad4_x_sign)*(12),(quad4_y_sign)*(10)))
                .strafeTo(new Vector2d((quad4_x_sign)*(12),(quad4_y_sign)*(61)))
                .strafeTo(new Vector2d((quad4_x_sign)*(55),(quad4_y_sign)*(10)))
                .turn(Math.toRadians(115))
                .waitSeconds(0.3)
                .waitSeconds(shootTimer)

                //Get third line
                .turn(Math.toRadians(-115))
                .strafeTo(new Vector2d((quad3_x_sign)*(12),(quad3_y_sign)*(10)))
                .strafeTo(new Vector2d((quad3_x_sign)*(12),(quad3_y_sign)*(56)))
                .strafeTo(new Vector2d((quad3_x_sign)*(ballShootX),(quad3_y_sign)*(ballShootY)))
                .turn(Math.toRadians(135))
                .waitSeconds(0.3)
                .waitSeconds(shootTimer)

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

                .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95F)
                .addEntity(myBot)
                .start();
    }
}