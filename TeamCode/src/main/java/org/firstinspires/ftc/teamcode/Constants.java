package org.firstinspires.ftc.teamcode;

/*
* Constants File - all configurations/angles/positions/speeds should be located here
*/

import com.acmerobotics.roadrunner.Pose2d;

public class Constants {

    // Drivetrain
    public static final String DRIVE_FRONT_LEFT_CONFIG  = "frontLeft";   // Control Hub Port 0
    public static final String DRIVE_FRONT_RIGHT_CONFIG = "frontRight"; // Control Hub Port 1
    public static final String DRIVE_BACK_LEFT_CONFIG   = "backLeft";     // Control Hub Port 2
    public static final String DRIVE_BACK_RIGHT_CONFIG  = "backRight";   // Control Hub Port 3

    // Elevator
    public static final String ELEVATOR_MOTOR_CONFIG     = "vertical";
    public static final String ELEVATOR_2ND_MOTOR_CONFIG = "secondVertical";
    public static final double ELEVATOR_HIGH_BASKET_POS  = 840;//2700;
    public static final double ELEVATOR_LOW_BASKET_POS   = 120;
    public static final double ELEVATOR_HOME_POS         = 0;
    public static final double ELEVATOR_PID_P_TERM       = .007;

    // Depositor Pivot
    public static final String DEPOSITOR_PIVOT_SERVO_CONFIG        = "basketServo";
    public static final double DEPOSITOR_PIVOT_TO_BASKET_ANGLE     = 240;
    public static final double DEPOSITOR_PIVOT_TO_SPECIMEN_ANGLE   = 260;
    public static final double DEPOSITOR_PIVOT_TO_TRANSITION_ANGLE = 20;
    public static final double DEPOSITOR_PIVOT_TO_HOME_ANGLE       = 20;
    public static final double DEPOSITOR_PIVOT_TO_PRIME_ANGLE      = 140;

    // Extension
    public static final String LEFT_EXTENSION_CONFIG    = "leftHorizontal";
    public static final String RIGHT_EXTENSION_CONFIG   = "rightHorizontal";
    public static final double EXTENSION_OUT_ANGLE      = 25;
    public static final double EXTENSION_IN_ANGLE       = 0;
    public static final String EXTENSION_ENCODER_CONFIG = "extensionEncoder";

    // Intake
    public static final String INTAKE_PIVOT_CONFIG              = "intakePivot";
    public static final String INTAKE_LEFT_VACUUM_CONFIG        = "rightVacuum";
    public static final String INTAKE_RIGHT_VACUUM_CONFIG       = "leftVacuum";
    public static final String INTAKE_BEAM_BREAK_CONFIG         = "intakeBeam";
    public static final String INTAKE_PIVOT_ENCODER             = "intakeEncoder";
    public static final double INTAKE_PIVOT_TO_TRANSITION_ANGLE = 210;
    public static final double INTAKE_PIVOT_TO_DOWN_ANGLE       = 70;
    public static final double INTAKE_PIVOT_TO_REST_ANGLE       = 210;
    public static final double INTAKE_PIVOT_TO_EJECT_ANGLE      = 190;
    public static final double INTAKE_INCHING_SPEED             = 0.2;
    public static final double INTAKE_SUCK_SPEED                = -1;
    public static final double INTAKE_EJECT_SPEED               = .5;
    public static final double INTAKE_CURRENT_JAM_THRESHOLD     = 800;

    // Main claw
    public static final String MAIN_CLAW_CONFIG      = "specimen";
    public static final double MAIN_CLAW_OPEN_ANGLE  = 90;
    public static final double MAIN_CLAW_CLOSE_ANGLE = 26;
    public static final long MAIN_CLAW_MS_OPEN       = 250;

    // Kicker
    public static final String KICKER_CONFIG = "kicker";

    // Limelight
    public static final short LIMELIGHT_APRILTAG_PIPELINE = 1;

    // ETC.
    public static boolean IS_RED = false;
    public static Pose2d CURRENT_POSE;

}
