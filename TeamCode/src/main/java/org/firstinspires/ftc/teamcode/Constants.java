package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;

public class Constants {
    public static final String driveFrontLeftConfig = "frontLeft";   // Control Hub Port 0
    public static final String driveFrontRightConfig = "frontRight"; // Control Hub Port 1
    public static final String driveBackLeftConfig = "backLeft";     // Control Hub Port 2
    public static final String driveBackRightConfig = "backRight";   // Control Hub Port 3

    public static final String depositorVerticalConfig = "vertical";

    public static final String depositorBasketConfig = "basketServo";
    public static final double depositorVerticalToTopPose = 2600;
    public static final double depositorVerticalToMidPose = 1600;
    public static final double depositorVerticalToBottomPose = 0;
    public static final double depositorVerticalKP = .0022;
    public static final double depositorVerticalErrorRange = 10;

    public static final double depositorBasketToDepositAngle = 110;
    public static final double depositorBasketToHomeAngle = 0;

    public static final String intakeHorizontalConfig = "horizontal";
    public static final String intakePivotConfig = "intakePivot";
    public static final String intakeVacuumConfig = "vacuumServo";

    public static final double intakeHorizontalToIntakePose = 100;
    public static final double intakeHorizontalToHomePose = 0;

    public static final double intakePivotToBasket = 50;
    public static final double intakePivotToDown = 205;

    public static final double intakeRestingSpeed = .1;
    public static final double intakeVacuumSpeed = .5;
    public static final double intakeEjectSpeed = -.7;

    public static final double intakeHorizontalKP = .05;

    public static Pose2d pose;
}
