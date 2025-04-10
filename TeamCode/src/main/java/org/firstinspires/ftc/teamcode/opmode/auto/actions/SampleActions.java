package org.firstinspires.ftc.teamcode.opmode.auto.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

// This class is all the actions used in autonomous. They are all in one file so you only have to change one action to affect all autos.
public class SampleActions { // 8, 21, -16
    public static final Pose2d basketPos = new Pose2d(
            -57.523881554,
            -56.2525317,
            Math.toRadians(45));

    public static final Pose2d rightmostSamplePos = new Pose2d(
            -50,
            -46,
            Math.toRadians(71));

    public static final Pose2d toRightmostBasketPos = new Pose2d(
            -60.5,
            -55.25,
            Math.toRadians(68));

    public static final Pose2d middleSamplePos = new Pose2d(
            basketPos.position,
            Math.toRadians(80)
    );

    public static final Pose2d toMiddleBasketPos = new Pose2d(
            -59,
            -52.5,
            Math.toRadians(74));

    public static final Pose2d leftmostSamplePos = new Pose2d(
            -56,
            -46,
            Math.toRadians(117)
    );

    public static final Pose2d toLeftmostBasketPos = new Pose2d(
            -57.5,
            -56.25,
            Math.toRadians(110));

    public static final Pose2d submersiblePos1 = new Pose2d(
            -21,
            -7,
            Math.toRadians(0)
    );

    public static final Pose2d submersiblePos2 = new Pose2d(
            -21,
            -12,
            Math.toRadians(0)
    );

    public static final Pose2d submersiblePos3 = new Pose2d(
            -21,
            -10,
            Math.toRadians(0)
    );


    public static Action startToBasket(Drivetrain drive, Pose2d home) {
        return drive.getTrajectoryBuilder(home)
                .strafeToLinearHeading(basketPos.position, basketPos.heading)
                .build();
    }

    public static Action startToAngledBasket(Drivetrain drive, Pose2d home) {
        return drive.getTrajectoryBuilder(home)
                .strafeToLinearHeading(toRightmostBasketPos.position, toRightmostBasketPos.heading)
                .build();
    }

    public static Action basketToFirstSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(basketPos)
                //.turnTo(Math.toRadians(75))
                .turnTo(rightmostSamplePos.heading)
                .build();
    }

    public static Action basketToSecondSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(basketPos)
                .turnTo(middleSamplePos.heading)
                .build();
    }

    public static Action basketToThirdSample(Drivetrain drive) {
        return drive.getTrajectoryBuilder(basketPos)
                .strafeToLinearHeading(leftmostSamplePos.position, leftmostSamplePos.heading)
                .build();
    }

    public static Action basketToSubmersible(Drivetrain drive) {
        return drive.getTrajectoryBuilder(basketPos)
                .splineTo(submersiblePos1.position, Math.toRadians(0))
                .build();
    }

    public static Action submersibleToBasket(Drivetrain drive) {
        return drive.getTrajectoryBuilder(submersiblePos1)
                .setReversed(true)
                .splineToLinearHeading(basketPos, Math.toRadians(240))
                .build();
    }

    public static Action basketToSubmersible2(Drivetrain drive) {
        return drive.getTrajectoryBuilder(basketPos)
                .splineTo(submersiblePos2.position, Math.toRadians(0))
                .build();
    }

    public static Action submersible2ToBasket(Drivetrain drive) {
        return drive.getTrajectoryBuilder(submersiblePos2)
                .setReversed(true)
                .splineToLinearHeading(basketPos, Math.toRadians(240))
                .build();
    }

    public static Action too(Drivetrain drive) {
        return drive.getTrajectoryBuilder(new Pose2d(-57.5, -56.25, Math.toRadians(45)))
                .splineTo(submersiblePos3.position, Math.toRadians(0))
                .build();
    }

    public static Action from(Drivetrain drive) {
        return drive.getTrajectoryBuilder(submersiblePos3)
                .splineToLinearHeading(basketPos, Math.toRadians(240))
                .build();
    }
}