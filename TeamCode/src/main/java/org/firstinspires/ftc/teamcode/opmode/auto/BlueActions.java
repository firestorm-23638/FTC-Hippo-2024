package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

// This class is all the actions used in autonomous. They are all in one file so you only have to change one action to affect all autos.
public class BlueActions {
    public static final Pose basketPos = new Pose(15, 129, Math.toRadians(315));
    public static final Pose rightmostSamplePose = new Pose(23.8, 120.35, Math.toRadians(0));
    public static final Pose middleSamplePose = new Pose(basketPos.getX(), basketPos.getY(), Math.toRadians(0));
    public static final Pose leftmostSamplePose = new Pose(27.6, 131.35, Math.toRadians(25));

    public static final double specimenPlaceY = -32.25;

    public static final Pose leftSpecimenPos   = new Pose(-12, specimenPlaceY, Math.toRadians(180));
    public static final Pose rightSpecimenPos  = new Pose(4, specimenPlaceY, Math.toRadians(180));
    public static final Pose pickupSpecimenPos = new Pose(45, -66, Math.toRadians(0));


    public static PathChain startToBasket(Drivetrain drive, Pose home) {
        return drive.getBuilder()
                .addPath(new BezierLine(new Point(home), new Point(basketPos)))
                .setLinearHeadingInterpolation(home.getHeading(), basketPos.getHeading())
                .build();
    }

    public static PathChain startToSpecimen(Drivetrain drive, Pose home, Pose specPos) {
        return drive.getBuilder()
                .addPath(new BezierLine(new Point(home), new Point(specPos)))
                .setLinearHeadingInterpolation(home.getHeading(), specPos.getHeading())
                .build();
    }

    public static PathChain basketToFirstSample(Drivetrain drive) {
        return drive.getBuilder()
                .addPath(new BezierLine(new Point(basketPos), new Point(rightmostSamplePose)))
                .setLinearHeadingInterpolation(basketPos.getHeading(), rightmostSamplePose.getHeading())
                .build();
    }

    public static PathChain basketToSecondSample(Drivetrain drive) {
        return drive.getBuilder()
                .setLinearHeadingInterpolation(basketPos.getHeading(), middleSamplePose.getHeading())
                .build();
    }

    public static PathChain basketToThirdSample(Drivetrain drive) {
        return drive.getBuilder()
                .addPath(new BezierLine(new Point(basketPos), new Point(leftmostSamplePose)))
                .setLinearHeadingInterpolation(basketPos.getHeading(), leftmostSamplePose.getHeading())
                //.strafeToLinearHeading(new Vector2d(-44, -37), Math.toRadians(152))
                .build();
    }

    public static PathChain basketToSubmersible(Drivetrain drive) {
        return drive.getBuilder()
                .addPath(new BezierCurve(
                        new Point(15.000, 129.000, Point.CARTESIAN),
                        new Point(61.430, 132.845, Point.CARTESIAN),
                        new Point(59.765, 96.221, Point.CARTESIAN)
                ))
                .build();
    }

    public static PathChain submersibleToBasket(Drivetrain drive) {
        return drive.getBuilder()
                //.splineToLinearHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(240))
                .build();
    }

    public static PathChain basketToSubmersible2(Drivetrain drive) {
        return drive.getBuilder()
                //.splineTo(new Vector2d(-21, -12), Math.toRadians(0))
                .build();
    }

    public static PathChain submersible2ToBasket(Drivetrain drive) {
        return drive.getBuilder()
                //.splineToLinearHeading(new Pose2d(basketPos, Math.toRadians(45)), Math.toRadians(240))
                .build();
    }

//    public static Action toFirstSample(Drivetrain drive) {
//         return drive.getBuilder(new Pose2d(rightSpecimenPos, Math.toRadians(180)))
//                .setReversed(true)
//                // from specimen to first spike mark
//                .strafeTo(new Vector2d(rightSpecimenPos.x, rightSpecimenPos.y - 5))
//                .splineToLinearHeading(new Pose2d(30, -45, Math.toRadians(0)), Math.toRadians(0))
//                .splineToLinearHeading(new Pose2d(39, -20, Math.toRadians(0)), Math.toRadians(90))
//                .build();
//    }

//    public static Action pushFirstSample(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(39, -20, Math.toRadians(0)))
//                //.waitSeconds(1)
//                // from first spike to obs then back
//                .strafeTo(new Vector2d(39, -66))
//                .build();
//    }

//    public static Action pushTwoSamples(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(4, specimenPlaceY, Math.toRadians(270)))
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

//    public static Action toSecondSample(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(39, -60, Math.toRadians(0)))
//                //.waitSeconds(1)
//                // from first spike to obs then back
//                .setReversed(true)
//                .strafeTo(new Vector2d(39, -20))
//                .build();
//    }

//    public static Action pushSecondSample(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(39, -20, Math.toRadians(0)))
//                .strafeTo(new Vector2d(49, -20))
//                .strafeTo(pickupSpecimenPos.position)
//                .build();
//    }

//    public static Action placeSecondSpecimen(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(55, -45, Math.toRadians(270)))
//                .splineToConstantHeading(new Vector2d(5, specimenPlaceY), Math.toRadians(90))
//                .build();
//    }

//    public static Action pickupThirdSpecimen(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(5, specimenPlaceY, Math.toRadians(270)))
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(330))
//                .build();
//    }

//    public static Action placeThirdSpecimen(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(30, -50, Math.toRadians(330)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(5, specimenPlaceY, Math.toRadians(270)), Math.toRadians(90))
//                .build();
//    }

//    public static Action pickupFourthSpecimen(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(5, specimenPlaceY, Math.toRadians(270)))
//                .setReversed(true)
//                .strafeToLinearHeading(new Vector2d(30, -50), Math.toRadians(330))
//                .build();
//    }

//    public static Action placeFourthSpecimen(Drivetrain drive) {
//        return drive.getBuilder(new Pose2d(30, -50, Math.toRadians(330)))
//                .setReversed(true)
//                .splineToLinearHeading(new Pose2d(5, specimenPlaceY, Math.toRadians(270)), Math.toRadians(90))
//                .build();
//    }
}
