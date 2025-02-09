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
    public static final String depositor2ndVerticalConfig = "secondVertical";

    public static final String depositorBasketConfig = "basketServo";
    public static final double depositorVerticalToTopPose = 840
            ;//2700;
    public static final double depositorVerticalToMidPose = 120;
    public static final double depositorVerticalToBottomPose = 0;
    public static final double depositorVerticalKP = .007;
    public static final double depositorVerticalErrorRange = 10;

    public static final double depositorVerticalToTopBar = 2000;
    public static final double depositorVerticalUpSlightly = 500;

    public static final double depositorBasketToDepositAngle = 240;
    public static final double depositorBasketToSpecimenAngle = 260;
    public static final double depositorBasketToTransitionAngle = 20;
    public static final double depositorBasketToHomeAngle = 20;

    public static final long depositorArmSpecimenTimeMs = 700;
    public static final long depositorClawOpenTimeMs = 300;

    public static final String intakeLeftHorizontalConfig = "leftHorizontal";
    public static final String intakeRightHorizontalConfig = "rightHorizontal";
    public static final String intakePivotConfig = "intakePivot";
    public static final String intakeLeftVacuumConfig = "rightVacuum";
    public static final String intakeRightVacuumConfig = "leftVacuum";

    public static final double intakeHorizontalToIntakePose = 25;
    public static final double intakeHorizontalToHomePose = 0;

    public static final double intakePivotToBasket = 210;
    public static final double intakePivotToDown = 60;
    public static final double intakePivotToRest = 210;
    public static final double intakePivotToEject = 190;

    public static final double intakeInchingSpeed = 0.2;
    public static final double intakeVacuumSpeed = -1;

    public static final double intakeEjectSpeed = .5;

    public static final double intakeHorizontalKP = .05;

    public static final String specimenClawConfig = "specimen";
    public static final String kickerConfig = "kicker";

    public static final String intakeBeamBreakConfig = "intakeBeam";
    public static final String horizontalEncoderConfig = "extensionEncoder";
    public static final String intakeWristEncoder = "intakeEncoder";


    public static final double specimenOpenAngle = 90;
    public static final double specimenCloseAngle = 11;

    public static final double intakeJamCurrentThreshold = 800;

    public static final short limelightApriltagPipeline = 1;

    public static boolean isRed = false;

    public static Pose2d pose;

}
