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
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                // adjust this to be where the robot really starts
                .setDimensions(17.8,17)
                .setStartPose(new Pose2d(-38,-61.5, Math.toRadians(90)))
                .build();
        RoadRunnerBotEntity myBot2 = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
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
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
                .splineTo(new Vector2d(-24, -37), Math.toRadians(0))
                .splineTo(new Vector2d(30, -37), Math.toRadians(0))
                .splineTo(new Vector2d(55, -61), Math.toRadians(0))
                .build());

        /* Red Basket to First Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                .setTangent(0)
//                .splineTo(new Vector2d(-26.5, -40.1), Math.toRadians(100))
//                .build());

//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-36, -35), Math.toRadians(145)))
//                .strafeTo(new Vector2d(-36 + Math.cos(145) * -5, -35 + Math.sin(145) * 5))
//                .build());


        /* First Sample to Red Basket */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(new Vector2d(-36, -35), Math.toRadians(145)))
//                        .setReversed(true)
//                .strafeToLinearHeading(Field.RED_BASKET, Math.toRadians(45))
//                .build());

        /* Red Basket to Second Sample */
//        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(Field.RED_BASKET, Math.toRadians(45)))
//                    .setTangent(0)
//                    .splineTo(new Vector2d(-39, -40), Math.toRadians(100))
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
//        Action basketTrajectory = myBot2.getDrive().actionBuilder(myBot2.getPose())
//                .strafeTo(Field.BLUE_BASKET)
//                .build();
//
//        myBot2.runAction(basketTrajectory);

        // Blue bucket to Blue Observation Zone
        myBot2.runAction(myBot.getDrive().actionBuilder(forBlue(new Pose2d(Field.RED_BASKET, Math.toRadians(45))))
                .splineTo(forBlue(new Vector2d(-24, -40)), forBlue(Math.toRadians(0)))
                .splineTo(forBlue(new Vector2d(23, -40)), forBlue(Math.toRadians(0)))
                .splineTo(forBlue(new Vector2d(46.9, -61)), forBlue(Math.toRadians(0)))
                .build());
        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .addEntity(myBot2)
                .start();
    }
}