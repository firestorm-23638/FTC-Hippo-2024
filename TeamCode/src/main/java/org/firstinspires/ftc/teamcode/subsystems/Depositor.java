package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

/*
Basket Class: Controls the Basket section of the depositor
*/

public class Depositor extends SubsystemBase {
    public enum state {     // The state that the basket is currently in. Home = in, bucket = deposit. Used in BasketPositionCommand
        HOME,
        TRANSITIONING,
        SPECIMEN,
        BUCKET,
        PRIME,
        CLAWOPEN,
        CLAWCLOSE,
        CLAWTIGHTEN
    };

    private final ServoEx pivot;     // Servo object
    private final ServoEx claw;     // Servo object

    private final Telemetry telemetry;     // Telemetry object, for printouts

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        pivot = new SimpleServo(hardwareMap, Constants.DEPOSITOR_PIVOT_SERVO_CONFIG, 0, 282.35, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, Constants.MAIN_CLAW_CONFIG, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        telemetry.addData("CLAW SET POS", claw.getAngle());
    }

    public void toHome() {
        pivot.turnToAngle(Constants.DEPOSITOR_PIVOT_TO_HOME_ANGLE);
    }

    public void toPrime() {
        pivot.turnToAngle(Constants.DEPOSITOR_PIVOT_TO_PRIME_ANGLE);
    }

    public void toTransition() {
        pivot.turnToAngle(Constants.DEPOSITOR_PIVOT_TO_TRANSITION_ANGLE);
    }

    public void toBasket() {
        pivot.turnToAngle(Constants.DEPOSITOR_PIVOT_TO_BASKET_ANGLE);
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

    public void toSpecimen() {
        pivot.turnToAngle(Constants.DEPOSITOR_PIVOT_TO_SPECIMEN_ANGLE);
    }
}
