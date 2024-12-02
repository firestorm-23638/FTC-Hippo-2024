package org.firstinspires.ftc.teamcode;

/*
PORTS
Control Hub - Motor

* */

import com.acmerobotics.roadrunner.Pose2d;

public class Constants {
    public static final String driveFrontLeftConfig = "frontLeft";   // Control Hub Port 0
    public static final String driveFrontRightConfig = "frontRight"; // Control Hub Port 1
    public static final String driveBackLeftConfig = "backLeft";     // Control Hub Port 2
    public static final String driveBackRightConfig = "backRight";   // Control Hub Port 3

    public static final String depositorVerticalConfig = "vertical";

    public static final String depositorBasketConfig = "basketServo";
    public static final double depositorVerticalToTopPose = 740;//2700;
    public static final double depositorVerticalToMidPose = 180;
    public static final double depositorVerticalToBottomPose = 0;
    public static final double depositorVerticalKP = .01;
    public static final double depositorVerticalErrorRange = 10;

    public static final double depositorVerticalToTopBar = 2000;
    public static final double depositorVerticalUpSlightly = 500;

    public static final double depositorBasketToDepositAngle = 179;
    public static final double depositorBasketToHomeAngle = 5;

    public static final String intakeLeftHorizontalConfig = "leftHorizontal";
    public static final String intakeRightHorizontalConfig = "rightHorizontal";
    public static final String intakePivotConfig = "intakePivot";
    public static final String intakeLeftVacuumConfig = "leftVacuum";
    public static final String intakeRightVacuumConfig = "rightVacuum";

    public static final double intakeHorizontalToIntakePose = 100;
    public static final double intakeHorizontalToHomePose = 0;

    public static final double intakePivotToBasket = 55;
    public static final double intakePivotToDown = 230;

    public static final double intakeRestingSpeed = 0.05;
    public static final double intakeVacuumSpeed = -.3;
            ;
    public static final double intakeEjectSpeed = .3;

    public static final double intakeHorizontalKP = .05;

    public static final String specimenClawConfig = "specimen";

    public static final double specimenOpenAngle = 90;
    public static final double specimenCloseAngle = 0;

    public static final short limelightApriltagPipeline = 1;

    public static boolean isRed = false;

    public static Pose2d pose;
}
