package org.firstinspires.ftc.teamcode.opmode.auto.red;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

// This class is all the actions used in autonomous. They are all in one file so you only have to change one action to affect all autos.
public class RedActions {
    public static final Vector2d basketPos = new Vector2d(58.023881554, 54.02525317);
    public static final double specimenPlaceY = 30.25;

    public static final Vector2d leftSpecimenPos = new Vector2d(12, specimenPlaceY);
    public static final Vector2d rightSpecimenPos = new Vector2d(16, specimenPlaceY);
    public static final Pose2d pickupSpecimenPos = new Pose2d(-49, 62, Math.toRadians(180));


    public static Action startToBasket(Drivetrain drive, Pose2d home) {
        return drive.getTrajectoryBuilder(home)
                .strafeToLinearHeading(basketPos, Math.toRadians(45+180))
                .build();
    }

    public static Action startToSpecimen(Drivetrain drive, Pose2d home, Vector2d specPos) {
        return drive.getTrajectoryBuilder(home)
                .strafeTo(specPos)
                .build();
    }

    public static Action basketToFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45+180)))
                .strafeToLinearHeading(new Vector2d(48, 52), Math.toRadians(270))
                .build();
    }

    public static Action specimenToFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(leftSpecimenPos, Math.toRadians(0)))
                .strafeTo(new Vector2d(leftSpecimenPos.x, leftSpecimenPos.y+7))
                .strafeToLinearHeading(new Vector2d(30, 42), Math.toRadians(133+180))
                .build();
    }

    public static Action basketToSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45+180)))
                .splineToLinearHeading(new Pose2d(55.5, 53, Math.toRadians(269)), Math.toRadians(270))
                .build();
    }

    public static Action basketToThirdSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45+180)))
                .strafeToLinearHeading(new Vector2d(44, 37), Math.toRadians(152+180))
                .build();
    }

    public static Action basketToFourthSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45+180)))
                .splineTo(new Vector2d(24, 37), Math.toRadians(180))
                .splineTo(new Vector2d(-10, 37), Math.toRadians(180))
                .splineTo(new Vector2d(-30, 35), Math.toRadians(30+180))
                .build();
    }

    public static Action basketToObservation(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45+180)))
                .splineTo(new Vector2d(24, 35), Math.toRadians(180))
                .splineTo(new Vector2d(-30, 35), Math.toRadians(180))
                .splineTo(new Vector2d(-55, 61), Math.toRadians(180))
                .build();
    }

    public static Action basketToSubmersible(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45+180)))
                .splineTo(new Vector2d(21, 10), Math.toRadians(180))
                .build();
    }

    public static Action toFirstSample(Drivetrain drive) {
         return drive.getTrajectoryBuilder(new Pose2d(rightSpecimenPos, Math.toRadians(0)))
                .setReversed(true)
                // from specimen to first spike mark
                .strafeTo(new Vector2d(rightSpecimenPos.x, rightSpecimenPos.y + 5))
                .splineToLinearHeading(new Pose2d(-30, 45, Math.toRadians(180)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-39, 20, Math.toRadians(180)), Math.toRadians(90+180))
                .build();
    }

    public static Action pushFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(-39, 20, Math.toRadians(180)))
                //.waitSeconds(1)
                // from first spike to obs then back
                .strafeTo(new Vector2d(-39, 60))
                .build();
    }

    public static Action toSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(-39, 60, Math.toRadians(180)))
                //.waitSeconds(1)
                // from first spike to obs then back
                .setReversed(true)
                .strafeTo(new Vector2d(-39, 20))
                .build();
    }

    public static Action pushSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(-39, 20, Math.toRadians(180)))
                .strafeTo(new Vector2d(-49, 20))
                .strafeTo(pickupSpecimenPos.position)
                .build();
    }

    public static Action placeSecondSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(pickupSpecimenPos)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23, 46, Math.toRadians(270)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-8, specimenPlaceY, Math.toRadians(0)), Math.toRadians(270))
                .build();
    }

    public static Action pickupThirdSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(-8, specimenPlaceY, Math.toRadians(0)))
                .setReversed(true)
                .strafeTo(new Vector2d(-8, specimenPlaceY+5))
                .splineToLinearHeading(pickupSpecimenPos, Math.toRadians(90))
                .build();
    }

    public static Action placeThirdSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(pickupSpecimenPos)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23, 46, Math.toRadians(270)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-5, specimenPlaceY, Math.toRadians(0)), Math.toRadians(270))
                .build();
    }

    public static Action pickupFourthSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(-5, specimenPlaceY, Math.toRadians(0)))
                .setReversed(true)
                .strafeTo(new Vector2d(-5, specimenPlaceY+5))
                .splineToLinearHeading(pickupSpecimenPos, Math.toRadians(90))
                .build();
    }

    public static Action placeFourthSpecimen(Drivetrain drive) {
        return drive.getTrajectoryBuilder(pickupSpecimenPos)
                .setReversed(true)
                .splineToLinearHeading(new Pose2d(-23, 46, Math.toRadians(270)), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-2, specimenPlaceY, Math.toRadians(0)), Math.toRadians(270))
                .build();
    }
}
