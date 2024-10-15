package org.firstinspires.ftc.teamcode;

public class Constants {
    public static final String driveFrontLeftConfig = "frontLeft";
    public static final String driveFrontRightConfig = "frontRight";
    public static final String driveBackLeftConfig = "backLeft";
    public static final String driveBackRightConfig = "backRight";

    public static final String depositorVerticalConfig = "vertical";
    public static final String depositorBasketConfig = "basketServo";

    public static final double depositorVerticalToTopPose = 2880;
    public static final double depositorVerticalToMidPose = 1600;
    public static final double depositorVerticalToBottomPose = 0;
    public static final double depositorVerticalKP = .0018;
    public static final double depositorVerticalErrorRange = 10;

    public static final double depositorBasketToDepositAngle = 110;
    public static final double depositorBasketToHomeAngle = 0;

    public static final String intakeHorizontalConfig = "horizontal";
    public static final String intakePivotConfig = "intakePivot";
    public static final String intakeVacuumConfig = "vacuumServo";

    public static final double intakeHorizontalToIntakePose = 100;
    public static final double intakeHorizontalToHomePose = 0;

    public static final double intakePivotToBasket = 35;
    public static final double intakePivotToDown = 210;
    public static final double intakePivotToUp = 90;

    public static final double intakeVacuumSpeed = .4;

    public static final double intakeHorizontalKP = .05;
}
