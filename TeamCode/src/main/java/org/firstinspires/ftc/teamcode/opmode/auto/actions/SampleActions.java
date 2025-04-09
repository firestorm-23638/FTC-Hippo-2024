package org.firstinspires.ftc.teamcode.opmode.auto.actions;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

// This class is all the actions used in autonomous. They are all in one file so you only have to change one action to affect all autos.
public class SampleActions {
    public static final Pose2d basketPos = new Pose2d(
            -58.523881554,
            -55.2525317,
            Math.toRadians(45));

    public static final Pose2d rightmostSamplePos = new Pose2d(
            -50,
            -46,
            Math.toRadians(71));

    public static final Pose2d middleSamplePos = new Pose2d(
            basketPos.position,
            Math.toRadians(90)
    );

    public static final Pose2d leftmostSamplePos = new Pose2d(
            -58,
            -46,
            Math.toRadians(115)
    );

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


    public static Action startToBasket(Drivetrain drive, Pose2d home) {
        return drive.getTrajectoryBuilder(home)
                .strafeToLinearHeading(basketPos.position, basketPos.heading)
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
}