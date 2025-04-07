package subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import Constants.Constants;

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
        CLAWOPEN,
        CLAWCLOSE,
        CLAWTIGHTEN
    };

    public static class AnglePair {
        public double elbow;
        public double shoulder;
        public AnglePair(double shoulder, double elbow) {
            this.elbow = elbow;
            this.shoulder = shoulder;
        }
    }

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

    public void toHome() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_HOME_ANGLE_PAIR);
    }

    public void toPrime1() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_PRIME_PAIR);
    }

    public void toPrimeSpecimens() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_PRIME_ANGLE_PAIR);
    }

    public void zero() {
        shoulderServo.turnToAngle(90);
        elbowServo.turnToAngle(90); //hello luke
    }

    public void toVerticalTransition() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_VERTICAL_TRANSITION_ANGLE_PAIR);
    }

    public void toBasket() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_BASKET_ANGLE_PAIR);
    }

    public void clawOpen() {
        claw.turnToAngle(Constants.MAIN_CLAW_OPEN_ANGLE);
    }
    public void clawClose() {
        claw.turnToAngle(Constants.MAIN_CLAW_CLOSE_ANGLE);
    }
    public void clawTighten() {
        claw.turnToAngle(15);
    }

    public void toSpecimenPickup() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_PICKUP_ANGLE_PAIR);
    }

    public void toPlaceSpecimen() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_SCORE_ANGLE_PAIR);
    }

    public void toScoreSpecimen() {
        turnToAnglePair(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_INITIATE_ANGLE_PAIR);
    }

    private void turnToAnglePair(AnglePair pair) {
        currentElbow = pair.elbow;
        currentShoulder = pair.shoulder;
    }
}