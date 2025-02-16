package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

// This class is all the actions used in autonomous. They are all in one file so you only have to change one action to affect all autos.
public class BlueActions {
    public static final Vector2d basketPos = new Vector2d(-58.523881554, -55.2525317);
    public static final double specimenPlaceY = -32.25;
/*
                // pickup fourth specimen

                .waitSeconds(0.200)

                // place fourth specimen

                .waitSeconds(0.200)
                .build()
 */
    public static final Vector2d leftSpecimenPos = new Vector2d(-12, specimenPlaceY);
    public static final Vector2d rightSpecimenPos = new Vector2d(4, specimenPlaceY);
    public static final Pose2d pickupSpecimenPos = new Pose2d(45, -66, Math.toRadians(0));


    public static Action startToBasket(Drivetrain drive, Pose2d home) {
        return drive.getTrajectoryBuilder(home)
                .strafeToLinearHeading(basketPos, Math.toRadians(45))
                .build();
    }

    public static Action startToSpecimen(Drivetrain drive, Pose2d home, Vector2d specPos) {
        return drive.getTrajectoryBuilder(home)
                .strafeTo(specPos)
                .build();
    }

    public static Action basketToFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                //.turnTo(Math.toRadians(75))
                .strafeToLinearHeading(new Vector2d(-50, -52), Math.toRadians(90))
                .build();
    }

    public static Action specimenToFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(leftSpecimenPos, Math.toRadians(180)))
                .strafeTo(new Vector2d(leftSpecimenPos.x, leftSpecimenPos.y-7))
                .strafeToLinearHeading(new Vector2d(-30, -42), Math.toRadians(133))
                .build();
    }

    public static Action basketToSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .turnTo(Math.toRadians(93))
                //.splineToLinearHeading(new Pose2d(-55.5, -53, Math.toRadians(89)), Math.toRadians(90))
                .build();
    }

    public static Action basketToThirdSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .strafeToLinearHeading(new Vector2d(-58, -46), Math.toRadians(116))
                //.strafeToLinearHeading(new Vector2d(-44, -37), Math.toRadians(152))
                .build();
    }

    public static Action basketToFourthSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .splineTo(new Vector2d(-24, -37), Math.toRadians(0))
                .splineTo(new Vector2d(10, -37), Math.toRadians(0))
                .splineTo(new Vector2d(30, -35), Math.toRadians(30))
                .build();
    }

    public static Action basketToObservation(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .splineTo(new Vector2d(-24, -35), Math.toRadians(0))
                .splineTo(new Vector2d(30, -35), Math.toRadians(0))
                .splineTo(new Vector2d(55, -61), Math.toRadians(0))
                .build();
    }

    public static Action basketToSubmersible(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .splineTo(new Vector2d(-21, -7), Math.toRadians(0))
                .build();
    }

    public static Action submersibleToBasket(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(new Vector2d(-21, -7), Math.toRadians(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(240))
                .build();
    }

    public static Action basketToSubmersible2(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .splineTo(new Vector2d(-21, -12), Math.toRadians(0))
                .build();
    }

    public static Action submersible2ToBasket(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(new Vector2d(-21, -12), Math.toRadians(0)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(240))
                .build();
    }

    public static Action toFirstSample(Drivetrain drive) {
         return drive.getTrajectoryBuilder(new Pose2d(rightSpecimenPos, Math.toRadians(180)))
                .setReversed(true)
                // from specimen to first spike mark
                .strafeTo(new Vector2d(rightSpecimenPos.x, rightSpecimenPos.y - 5))
                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(0)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(39, -20, Math.toRadians(0)), Math.toRadians(90))
                .build();
    }

    public static Action pushFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(39, -20, Math.toRadians(0)))
                //.waitSeconds(1)
                // from first spike to obs then back
                .strafeTo(new Vector2d(39, -66))
                .build();
    }

    public static Action pushTwoSamples(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(4, specimenPlaceY, Math.toRadians(270)))
                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(270)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(35, -20, Math.toRadians(270)), Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(41, -10, Math.toRadians(270)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(46, -20), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(46, -30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(46, -40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(46, -55), Math.toRadians(270))
                .splineToLinearHeading(new Pose2d(53, -10, Math.toRadians(270)), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(55, -20), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(55, -30), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(55, -40), Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(55, -60), Math.toRadians(270))
                .strafeTo(new Vector2d(45, -40))
                .build();
    }

    public static Action toSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(39, -60, Math.toRadians(0)))
                //.waitSeconds(1)
                // from first spike to obs then back
                .setReversed(true)
                .strafeTo(new Vector2d(39, -20))
                .build();
    }

    public static Action pushSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(39, -20, Math.toRadians(0)))
                .strafeTo(new Vector2d(49, -20))
                .strafeTo(pickupSpecimenPos.position)
                .build();
    }

    public static Action placeSecondSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(55, -45, Math.toRadians(270)))
                .splineToConstantHeading(new Vector2d(5, specimenPlaceY), Math.toRadians(90))
                .build();
    }

    public static Action pickupThirdSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(5, specimenPlaceY, Math.toRadians(270)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(330))
                .build();
    }

    public static Action placeThirdSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(30, -50, Math.toRadians(330)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(5, specimenPlaceY, Math.toRadians(270)), Math.toRadians(90))
                .build();
    }

    public static Action pickupFourthSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(5, specimenPlaceY, Math.toRadians(270)))
                .setReversed(true)
                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(330))
                .build();
    }

    public static Action placeFourthSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(30, -50, Math.toRadians(330)))
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(5, specimenPlaceY, Math.toRadians(270)), Math.toRadians(90))
                .build();
    }
}
