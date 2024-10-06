package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Depositor extends SubsystemBase {
    private final Motor vertical;
    private final ServoEx basket;

    public Depositor(HardwareMap hardwareMap) {
        vertical = new Motor(hardwareMap, Constants.depositorVerticalConfig);
        basket = new SimpleServo(hardwareMap, Constants.depositorBasketConfig, 0, 180, AngleUnit.DEGREES);
    };

    private void verticalToPos(double targetPos) {
        vertical.setRunMode(Motor.RunMode.RawPower);
        double kP = Constants.depositorVerticalKP;
        double kConstant = .1;
        double kTolerance = 10;

        double currPos = vertical.getCurrentPosition();
        double error = targetPos - currPos;
        if (error > kTolerance || error < -kTolerance) {
            vertical.set(error * kP);
        }
        else {
            vertical.set(kConstant);
        }
    }

    public void moveVertical(double speed) {
        vertical.setRunMode(Motor.RunMode.RawPower);
        vertical.set(speed);
    }

    public void verticalToTop(boolean controlInput) {
        if (controlInput) {
            verticalToPos(Constants.depositorVerticalToTopPose);
        }
        else {
            vertical.stopMotor();
        }
    }

    public void verticalToBottom(boolean controlInput) {
        if (controlInput) {
            verticalToPos(Constants.depositorVerticalToBottomPose);
        }
        else {
            vertical.stopMotor();
        }
    }

    public void basketToDeposit() {
        basket.turnToAngle(Constants.depositorBasketToDepositAngle);
    }

    public void basketToHome() {
        basket.turnToAngle(Constants.depositorBasketToHomeAngle);
    }
}
