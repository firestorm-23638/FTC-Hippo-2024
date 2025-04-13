package org.firstinspires.ftc.teamcode;

/*
* Constants File - all configurations/angles/positions/speeds should be located here
*/


import org.firstinspires.ftc.teamcode.subsystems.Depositor;

public class Constants {

    // Drivetrain
    public static final String DRIVE_FRONT_LEFT_CONFIG  = "frontLeft";   // Control Hub Port 0
    public static final String DRIVE_FRONT_RIGHT_CONFIG = "frontRight"; // Control Hub Port 1
    public static final String DRIVE_BACK_LEFT_CONFIG   = "backLeft";     // Control Hub Port 2
    public static final String DRIVE_BACK_RIGHT_CONFIG  = "backRight";   // Control Hub Port 3

    // Elevator
    public static final String ELEVATOR_MOTOR_CONFIG     = "vertical";
    public static final String ELEVATOR_2ND_MOTOR_CONFIG = "secondVertical";
    public static final double ELEVATOR_HIGH_BASKET_POS  = 535;
    public static final double ELEVATOR_OVERFLOW_POS  = 535;//2700;

    public static final double ELEVATOR_LOW_BASKET_POS   = 300;
    public static final double ELEVATOR_HOME_POS         = 0;
    public static final double ELEVATOR_PID_P_TERM       = .025;

    // Depositor Pivot
    public static final String DEPOSITOR_PIVOT_ELBOW_CONFIG        = "elbowServo";
    public static final String DEPOSITOR_PIVOT_SHOULDER_CONFIG     = "shoulderServo";

    public static final double shoulderOffset = 9;

    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_VERTICAL_TRANSITION_ANGLE_PAIR = new Depositor.AnglePair(72+shoulderOffset, 129);
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_SPECIMEN_PICKUP_ANGLE_PAIR     = new Depositor.AnglePair(130,83); //109,130
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_SPECIMEN_PRIME_ANGLE_PAIR      = new Depositor.AnglePair(123+shoulderOffset, 67); //109,130
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_PRIME_PAIR                   = DEPOSITOR_PIVOT_TO_VERTICAL_TRANSITION_ANGLE_PAIR;
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_HOME_ANGLE_PAIR              = new Depositor.AnglePair(149+shoulderOffset, 90);
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_BASKET_ANGLE_PAIR            = new Depositor.AnglePair(41+shoulderOffset, 74);  //32 92
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_OVERFLOW_BASKET_ANGLE_PAIR = new Depositor.AnglePair(32+shoulderOffset, 92);
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_SPECIMEN_SCORE_ANGLE_PAIR    = new Depositor.AnglePair(83+shoulderOffset, 41);
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_SPECIMEN_INITIATE_ANGLE_PAIR = new Depositor.AnglePair(83+shoulderOffset, 41);
    public static final Depositor.AnglePair DEPOSITOR_PIVOT_TO_PRIME_BASKET_ANGLE_PAIR      = new Depositor.AnglePair(50+shoulderOffset, 98);

    // Extension
    public static final String LEFT_EXTENSION_CONFIG    = "leftHorizontal";
    public static final String RIGHT_EXTENSION_CONFIG   = "rightHorizontal";
    public static final double EXTENSION_OUT_ANGLE      = 15;
    public static final double EXTENSION_IN_ANGLE       = 0;
    public static final double EXTENSION_TRANSFER_ANGLE = EXTENSION_IN_ANGLE;

    // Intake
    public static final String INTAKE_PIVOT_CONFIG              = "intakePivot";
    public static final String INTAKE_LEFT_VACUUM_CONFIG        = "rightVacuum";
    public static final String INTAKE_RIGHT_VACUUM_CONFIG       = "leftVacuum";
    public static final String INTAKE_BEAM_BREAK_CONFIG         = "intakeBeam";
    public static final String INTAKE_REV_COLOR_SENSOR_CONFIG   = "revColor";
    public static final double INTAKE_PIVOT_TO_TRANSITION_ANGLE = 173;
    public static final double INTAKE_PIVOT_TO_DOWN_ANGLE       = 87;
    public static final double INTAKE_PIVOT_TO_REST_ANGLE       = INTAKE_PIVOT_TO_TRANSITION_ANGLE;
    public static final double INTAKE_PIVOT_TO_SPECIMEN_ANGLE   = INTAKE_PIVOT_TO_TRANSITION_ANGLE;
    public static final double INTAKE_PIVOT_TO_EJECT_ANGLE      = INTAKE_PIVOT_TO_DOWN_ANGLE;
    public static final double INTAKE_INCHING_SPEED             = 0.2;
    public static final double INTAKE_SUCK_SPEED                = -1;
    public static final double INTAKE_EJECT_SPEED               = .5;
    public static final double INTAKE_CURRENT_JAM_THRESHOLD     = 800;

    // Main claw
    public static final String MAIN_CLAW_CONFIG      = "mainClaw";
    public static final double MAIN_CLAW_OPEN_ANGLE  = 90;
    public static final double MAIN_CLAW_CLOSE_ANGLE = 26;
    public static final long MAIN_CLAW_MS_OPEN       = 250;

    // Kicker
    public static final String KICKER_CONFIG = "kicker";

    // Light indicator
    public static final String LIGHT_INDICATOR0_CONFIG = "lightIndicator0";
    public static final String LIGHT_INDICATOR1_CONFIG = "lightIndicator1";


    // Limelight
    public static final short LIMELIGHT_APRILTAG_PIPELINE = 1;

    // Climb
    public static final String CLIMB_LEFT_MOTOR_CONFIG = "leftClimb";
    public static final String CLIMB_RIGHT_MOTOR_CONFIG = "rightClimb";

    // ETC.
    public static boolean IS_RED = false;

}
