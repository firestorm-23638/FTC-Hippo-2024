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
        CLAWOPEN,
        CLAWCLOSE,
        CLAWTIGHTEN
    };

    private final ServoEx pivot;     // Servo object
    private final ServoEx claw;     // Servo object

    private final Telemetry telemetry;     // Telemetry object, for printouts

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        pivot = new SimpleServo(hardwareMap, Constants.depositorBasketConfig, 0, 282.35, AngleUnit.DEGREES);
        claw = new SimpleServo(hardwareMap, Constants.specimenClawConfig, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        telemetry.addData("CLAW SET POS", claw.getAngle());
    }

    public void toHome() {
        pivot.turnToAngle(Constants.depositorBasketToHomeAngle);
    }

    public void toTransition() {
        pivot.turnToAngle(Constants.depositorBasketToTransitionAngle);
    }

    public void toBasket() {
        pivot.turnToAngle(Constants.depositorBasketToDepositAngle);
    }

    public void clawOpen() {
        claw.turnToAngle(Constants.specimenOpenAngle);
    }
    public void clawClose() {
        claw.turnToAngle(Constants.specimenCloseAngle);
    }
    public void clawTighten() {
        claw.turnToAngle(15);
    }

    public void toSpecimen() {
        pivot.turnToAngle(Constants.depositorBasketToSpecimenAngle);
    }
}
