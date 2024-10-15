package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Depositor extends SubsystemBase {
    private final Motor vertical;
    private final ServoEx basket;
    private final Telemetry telemetry;

    private int currentStage = 0;
    private double pidTarget = 0;

    private final double[] stageTargets = {Constants.depositorVerticalToBottomPose, Constants.depositorVerticalToMidPose, Constants.depositorVerticalToTopPose};

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        vertical = new Motor(hardwareMap, Constants.depositorVerticalConfig);
        basket = new SimpleServo(hardwareMap, Constants.depositorBasketConfig, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    public void increaseStage() {
        if (currentStage == 2) {
            return;
        }
        currentStage ++;
    }

    public void decreaseStage() {
        if (currentStage == 0) {
            return;
        }
        currentStage --;
    }

    @Override
    public void periodic() {
        telemetry.addData("Vertical Position", vertical.getCurrentPosition());
        telemetry.addData("Basket Position", basket.getAngle());
        telemetry.addData("Current Stage", currentStage);

        pidTarget = stageTargets[currentStage];
        verticalToPos(pidTarget);

    }

    private void verticalToPos(double targetPos) {
        vertical.setRunMode(Motor.RunMode.RawPower);
        double kP = Constants.depositorVerticalKP;
        double kConstant = .1;
        //double kTolerance = 10;

        double currPos = vertical.getCurrentPosition();
        double error = targetPos - currPos;
        //if (error > kTolerance || error < -kTolerance) {
        vertical.set(error * kP);
        //}
        //else {
        //vertical.set(kConstant);
        //}
    }

    public void moveVertical(double speed) {
        vertical.setRunMode(Motor.RunMode.RawPower);
        vertical.set(speed);
    }

    public void verticalToTop() {
        pidTarget = Constants.depositorVerticalToTopPose;
    }

    public void verticalToBottom() {
        pidTarget = Constants.depositorVerticalToBottomPose;
    }

    public void basketToDeposit() {
        basket.turnToAngle(Constants.depositorBasketToDepositAngle);
    }

    public void basketToHome() {
        basket.turnToAngle(Constants.depositorBasketToHomeAngle);
    }
}
