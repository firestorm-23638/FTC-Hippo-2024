package com.firestorm.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeBlueLight;
import com.noahbres.meepmeep.roadrunner.Constraints;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
import com.acmerobotics.roadrunner.Pose2d;

public class MeepMeepTesting {
    public static Pose2d forBlue(Pose2d pose) {
        return new Pose2d(-pose.position.x, -pose.position.y, pose.heading.plus(Math.toRadians(180)).log());
    }
    public static Vector2d forBlue(Vector2d pos) {
        return new Vector2d(-pos.x, -pos.y);
    }

    public static double forBlue(double theta) {
        return theta + Math.toRadians(180);
    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 120, Math.toRadians(500), Math.toRadians(720), 15)
                // adjust this to be where the robot really starts
                .setDimensions(17.8,17)
                .setStartPose(new Pose2d(0, -61, Math.toRadians(180)))
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 120, Math.toRadians(500), Math.toRadians(720), 15)
                // adjust this to be where the robot really starts
                .setDimensions(17.8,17)
                .setStartPose(forBlue(new Pose2d(-47,-58.5, Math.toRadians(45))))
                .setColorScheme(new ColorSchemeBlueLight())
                .build();

        RoadRunnerBotEntity myBot3 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 120, Math.toRadians(500), Math.toRadians(720), 15)
                // adjust this to be where the robot really starts
                .setDimensions(17.8,17)
                .setStartPose(forBlue(new Pose2d(-38,-61.5, Math.toRadians(45))))
                .setColorScheme(new ColorSchemeBlueLight())
                .build();

        /* Sanity Test */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(0,0), Math.toRadians(0)))
//                .strafeTo(new Vector2d(-6, 10))
//                .build());

        /* Pretty sure all routes can be adjusted for blue by negating x,y and adding 180 to all angles */

        /* Start to Red Basket */
//        Action basketTrajectory = myBot.getDrive().actionBuilder(myBot.getPose())
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .build();
//        myBot.runAction(basketTrajectory);


        /* Red Basket to Observation Zone */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .splineTo(new Vector2d(-24, -37), Math.toRadians(0))
//                .splineTo(new Vector2d(30, -37), Math.toRadians(0))
//                .splineTo(new Vector2d(55, -61), Math.toRadians(0))
//                .build());

        /* Red Basket to First Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .strafeToLinearHeading(new Vector2d(-48, -48), Math.toRadians(90))
//                .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-36, -35), Math.toRadians(145)))
//                .strafeTo(new Vector2d(-36 + Math.cos(145) * -5, -35 + Math.sin(145) * 5))
//                .build());

        /* Red specimen to first sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.SPECIMEN_POS, Math.toRadians(180)))
////                .strafeTo(Field.SPECIMEN_POS)
//                        .strafeTo(new Vector2d(Field.SPECIMEN_POS.x, Field.SPECIMEN_POS.y-7))
//                .strafeToLinearHeading(new Vector2d(-30, -42), Math.toRadians(140))
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .splineToLinearHeading(new Pose2d(-56, -53, Math.toRadians(90)), Math.toRadians(90))
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .strafeToLinearHeading(new Vector2d(-44, -37), Math.toRadians(150))
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .build());

        /* First Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-36, -35), Math.toRadians(145)))
//                        .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .build());

        /* Red Basket to Second Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .strafeToLinearHeading(new Vector2d(-57, -46), Math.toRadians(90))
//                .build());

        /* Second Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-41, -34), Math.toRadians(145)))
//                .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//        .build());

        /* Red Basket to Third Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                    .strafeToLinearHeading(new Vector2d(-55, -45), Math.toRadians(125))
//                    .build());

        /* Red Basket to Fourth Sample */
//            myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//            .splineTo(new Vector2d(-24, -37), Math.toRadians(0))
//            .splineTo(new Vector2d(10, -37), Math.toRadians(0))
//            .splineTo(new Vector2d(30, -35), Math.toRadians(30))
//            .build());

        /*  Red Basket to Submersible Zone */

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//            .strafeToLinearHeading(new Vector2d(30, -30), Math.toRadians(0))
//            .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .splineTo(new Vector2d(-30, -10), Math.toRadians(0))
//                .build());

        /* Third Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-52.5, -34), Math.toRadians(145)))
//                .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//        .build());


        /** BLUE SIDE PATHS **/
//        Action basketTrajectory = myBot2.getDrive().actionBuilder(myBot2.getPose())
//                .strafeTo(Field.BLUE_BASKET)
//                .build();
//
//        myBot2.runAction(basketTrajectory);

        // Blue bucket to Blue Observation Zone
//        myBot2.runAction(myBot.getDrive().actionBuilder(forBlue(new Pose2d(Field.RED_BASKET, Math.toRadians(45))))
//                .splineTo(forBlue(new Vector2d(-24, -40)), forBlue(Math.toRadians(0)))
//                .splineTo(forBlue(new Vector2d(23, -40)), forBlue(Math.toRadians(0)))
//                .splineTo(forBlue(new Vector2d(46.9, -61)), forBlue(Math.toRadians(0)))
//                .build());
        // start to specimen score
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0, -61, Math.toRadians(180)))
//                .strafeTo(new Vector2d(0, -33))
//                .build());
//         specimen score to spike mark
        myBot2.runAction(myBot2.getDrive().actionBuilder(new Pose2d(0, -33, Math.toRadians(180)))
                .setReversed(true)
                // from specimen to first spike mark
                .splineToConstantHeading(new Vector2d(10, -37), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(30.5, -37, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(30.5, -20, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(44, -10, Math.toRadians(270)), Math.toRadians(270))
                // from first spike to obs then back
                .splineToConstantHeading(new Vector2d(44, -55), Math.toRadians(270))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(44, -20), Math.toRadians(90))
                .strafeTo(new Vector2d(56, -20))
                // second spike to obs then back
                .strafeTo(new Vector2d(56, -55))
                .waitSeconds(0.1)
                .setReversed(true)
                .strafeTo(new Vector2d(56, -20))
                .strafeTo(new Vector2d(64, -20))
                // third spike to obs
                .strafeTo(new Vector2d(64, -55))
                .build()
        );
//
//        myBot3.runAction(myBot3.getDrive().actionBuilder(new Pose2d(60, -55, Math.toRadians(90)))
//                .waitSeconds(9.2)
//                .strafeToSplineHeading(new Vector2d(37, -57), Math.toRadians(0))
//                .strafeTo(new Vector2d(37, -60))
//                .strafeToSplineHeading(new Vector2d(8, -33), Math.toRadians(180))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(37, -57), Math.toRadians(0))
//                .strafeTo(new Vector2d(37, -60))
//                .strafeToSplineHeading(new Vector2d(6, -33), Math.toRadians(180))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(37, -57), Math.toRadians(0))
//                .strafeTo(new Vector2d(37, -60))
//                .strafeToSplineHeading(new Vector2d(4, -33), Math.toRadians(180))
//                .waitSeconds(0.5)
//                .strafeToSplineHeading(new Vector2d(37, -57), Math.toRadians(0))
//                .strafeTo(new Vector2d(37, -60))
//                .strafeToSplineHeading(new Vector2d(2, -33), Math.toRadians(180))
//                .build()
//        );
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .addEntity(myBot3)
                .start();

    }
}