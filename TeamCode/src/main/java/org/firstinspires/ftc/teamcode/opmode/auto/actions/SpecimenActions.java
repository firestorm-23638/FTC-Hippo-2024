package org.firstinspires.ftc.teamcode.opmode.auto.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class SpecimenActions {
    public static final Pose2d startingPos = new Pose2d(10, -61, Math.toRadians(270));
    public static final Pose2d firstSpecimenScore = new Pose2d(5, -29, Math.toRadians(270));

    //    public static Action toFirstSample(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(rightSpecimenPos, Math.toRadians(180)))
//                .setReversed(true)
//                // from specimen to first spike mark
//                .strafeTo(new Vector2d(rightSpecimenPos.x, rightSpecimenPos.y - 5))
//                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(0)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(39, -20, Math.toRadians(0)), Math.toRadians(90))
//                .build();
//    }

    public static Action startToScore(Drivetrain drivetrain) {
        return drivetrain.getTrajectoryBuilder(startingPos)
                .strafeToLinearHeading(firstSpecimenScore.position, firstSpecimenScore.heading)
                .build();
    }

    public static Action pushFirstSample(Drivetrain drivetrain) {
        return drivetrain.getTrajectoryBuilder(firstSpecimenScore)
                .splineToLinearHeading(new Pose2d(5, -35, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(35, -35, Math.toRadians(270)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(35, -10, Math.toRadians(270)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(45, -10, Math.toRadians(270)), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(45, -55, Math.toRadians(270)), Math.toRadians(270))
                .build();
    }



//    public static Action pushFirstSample(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(39, -20, Math.toRadians(0)))
//                //.waitSeconds(1)
//                // from first spike to obs then back
//                .strafeTo(new Vector2d(39, -66))
//                .build();
//    }
//
//    public static Action pushTwoSamples(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(4, specimenPlaceY, Math.toRadians(270)))
//                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(270)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(270)), Math.toRadians(90))
//                .splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(270)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(46, -20), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(46, -30), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(46, -40), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(46, -55), Math.toRadians(270))
//                .splineToLinearHeading(new Pose2d(53, -10, Math.toRadians(270)), Math.toRadians(0))
//                .splineToConstantHeading(new Vector2d(55, -20), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(55, -30), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(55, -40), Math.toRadians(270))
//                .splineToConstantHeading(new Vector2d(55, -60), Math.toRadians(270))
//                .strafeTo(new Vector2d(45, -40))
//                .build();
//    }
//
//    public static Action toSecondSample(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(39, -60, Math.toRadians(0)))
//                //.waitSeconds(1)
//                // from first spike to obs then back
//                .setReversed(true)
//                .strafeTo(new Vector2d(39, -20))
//                .build();
//    }
//
////    public static Action pushSecondSample(Drivetrain drive) {
////        return drive.getTrajectoryBuilder(new Pose2d(39, -20, Math.toRadians(0)))
////                .strafeTo(new Vector2d(49, -20))
////                .strafeTo(pickupSpecimenPos.position)
////                .build();
////    }
//
//    public static Action placeSecondSpecimen(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(55, -45, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(5, specimenPlaceY), Math.toRadians(90))
//                .build();
//    }
//
//    public static Action pickupThirdSpecimen(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(5, specimenPlaceY, Math.toRadians(270)))
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(330))
//                .build();
//    }
//
//    public static Action placeThirdSpecimen(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(30, -50, Math.toRadians(330)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(5, specimenPlaceY, Math.toRadians(270)), Math.toRadians(90))
//                .build();
//    }
//
//    public static Action pickupFourthSpecimen(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(5, specimenPlaceY, Math.toRadians(270)))
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(330))
//                .build();
//    }
//
//    public static Action placeFourthSpecimen(Drivetrain drive) {
//        return drive.getTrajectoryBuilder(new Pose2d(30, -50, Math.toRadians(330)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(5, specimenPlaceY, Math.toRadians(270)), Math.toRadians(90))
//                .build();
//    }
}
