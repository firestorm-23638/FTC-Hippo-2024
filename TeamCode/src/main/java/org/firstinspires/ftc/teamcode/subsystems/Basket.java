package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Basket extends SubsystemBase {
    public enum state {
        HOME,
        BUCKET
    };

    private final ServoEx basket;
    private final Telemetry telemetry;

    public state currentStage = state.HOME;
    private double pidTarget = 0;

    public Basket(HardwareMap hardwareMap, Telemetry telemetry) {
        basket = new SimpleServo(hardwareMap, Constants.depositorBasketConfig, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        telemetry.addData("Basket Position", basket.getAngle());

    }
    public boolean toDeposit() {
        basket.turnToAngle(Constants.depositorBasketToDepositAngle);
        return false;
    }

    public boolean toHome() {
        basket.turnToAngle(Constants.depositorBasketToHomeAngle);
        return false;
    }
}
