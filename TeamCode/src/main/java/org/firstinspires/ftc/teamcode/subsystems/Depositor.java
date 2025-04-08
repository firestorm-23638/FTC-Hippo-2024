package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/*
DEPOSITOR SERVO CONFIGS:



*/

public class Depositor extends SubsystemBase {
    public enum state {     // The state that the basket is currently in. Home = in, bucket = deposit. Used in BasketPositionCommand
        HOME,
        VERTICAL_TRANSITION,
        SPECIMEN,
        PRIME_SPECIMEN,
        PLACE_SPECIMEN,
        SCORE_SPECIMEN,
        BUCKET,
        PRIME,
        PRIME1,
        PRIME_BASKET,
        CLAWOPEN,
        CLAWCLOSE,
        CLAWTIGHTEN,
        ZERO
    };

    public static class AnglePair {
        public double elbow;
        public double shoulder;
        public AnglePair(double shoulder, double elbow) {
            this.elbow = elbow;
            this.shoulder = shoulder;
        }
    }// Range31-149

    private final ServoEx elbowServo;     // Servo object
    private final ServoEx shoulderServo;     // Servo object
    private final ServoEx claw;     // Servo object
    public double currentElbow = 90;
    public double currentShoulder = 90;

    private final Telemetry telemetry;     // Telemetry object, for printouts

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        elbowServo = new SimpleServo(hardwareMap, Constants.DEPOSITOR_PIVOT_ELBOW_CONFIG, 0, 180, AngleUnit.DEGREES);
        shoulderServo = new SimpleServo(hardwareMap, Constants.DEPOSITOR_PIVOT_SHOULDER_CONFIG, 0, 180, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, Constants.MAIN_CLAW_CONFIG, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        shoulderServo.turnToAngle(currentShoulder);
        elbowServo.turnToAngle(currentElbow); //hello luke
    }

    public void toPosition(state state) {
        switch(state) {
            case HOME:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_HOME_ANGLE_PAIR);
                break;
            case PRIME1:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_PRIME_PAIR);
                break;
            case PRIME_SPECIMEN:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_PRIME_ANGLE_PAIR);
                break;
            case VERTICAL_TRANSITION:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_VERTICAL_TRANSITION_ANGLE_PAIR);
                break;
            case BUCKET:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_BASKET_ANGLE_PAIR);
                break;
            case CLAWOPEN:
                claw.turnToAngle(Constants.MAIN_CLAW_OPEN_ANGLE);
                break;
            case CLAWCLOSE:
                claw.turnToAngle(Constants.MAIN_CLAW_CLOSE_ANGLE);
                break;
            case CLAWTIGHTEN:
                claw.turnToAngle(15);
                break;
            case SPECIMEN:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_PICKUP_ANGLE_PAIR);
                break;
            case PLACE_SPECIMEN:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_SCORE_ANGLE_PAIR);
                break;
            case SCORE_SPECIMEN:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_INITIATE_ANGLE_PAIR);
                break;
            case PRIME_BASKET:
                turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_PRIME_BASKET_ANGLE_PAIR);
                break;
            case ZERO:
                shoulderServo.turnToAngle(90);
                elbowServo.turnToAngle(90); //hello luke
                break;
        }
    }

    private void turnToAnglePair(AnglePair pair) {
        currentElbow = pair.elbow;
        currentShoulder = pair.shoulder;
    }
}