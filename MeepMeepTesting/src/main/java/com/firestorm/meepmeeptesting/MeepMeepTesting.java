package com.firestorm.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // adjust this to be where the robot really starts
                .setDimensions(15,15)
                .setStartPose(new Pose2d(-47,-60.5, Math.toRadians(45)))
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // adjust this to be where the robot really starts
                .setDimensions(15,15)
                .setStartPose(new Pose2d(-47,-60.5, Math.toRadians(45)))
                .build();

        Vector2d blueBasket = new Vector2d(-56.1923881554, -55.5502525317);

        // Start to Red Basket
//        Action basketTrajectory = myBot.getDrive().actionBuilder(myBot.getPose())
//                .strafeTo(Field.RED_BASKET)
//                .build();
//        myBot.runAction(basketTrajectory);


        /* Red Basket to Observation Zone */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .splineTo(new Vector2d(-24, -35), Math.toRadians(0))
//                .splineTo(new Vector2d(10, -35), Math.toRadians(0))
//                .splineTo(Field.RED_OBSERVATION, Math.toRadians(0))
//                .build());

        /* Red Basket to First Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .setTangent(0)
//                .splineTo(new Vector2d(-36, -35), Math.toRadians(100))
//                .build());

        /* First Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-36, -35), Math.toRadians(145)))
//                        .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .build());

        /* Red Basket to Second Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                    .setTangent(0)
//                    .splineTo(new Vector2d(-41, -34), Math.toRadians(100))
//                .build());

        /* Second Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-41, -34), Math.toRadians(145)))
//                .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//        .build());

        /* Red Basket to Third Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .setTangent(0)
//                .splineTo(new Vector2d(-52.5, -34), Math.toRadians(110))
//                .build());

        /* Third Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-52.5, -34), Math.toRadians(145)))
//                .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//        .build());


        /** BLUE SIDE PATHS **/

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
//                .addEntity(myBot2)
                .start();
    }
}