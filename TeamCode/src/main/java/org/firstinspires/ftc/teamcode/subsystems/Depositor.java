package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;


/*
 DEPRECATED. Use Basket and Elevator separate subsystems instead
 */
public class Depositor extends SubsystemBase {
    public enum state {
            HOME((int)Constants.depositorVerticalToBottomPose),
            MIDDLE_BASKET((int)Constants.depositorVerticalToMidPose),
            HIGH_BASKET((int)Constants.depositorVerticalToTopPose);

            public final int pos;
            private state(int pos) {
                this.pos = pos;
            }
    }

    public enum basketState {
        HOME,
        BUCKET
    };

    private final Motor vertical;
    private final ServoEx basket;
    private final Telemetry telemetry;

    public state currentStage = state.HOME;
    private double pidTarget = 0;
    private double basketTime = -1;

    private final double[] stageTargets = {Constants.depositorVerticalToBottomPose, Constants.depositorVerticalToMidPose, Constants.depositorVerticalToTopPose};

    public Depositor(HardwareMap hardwareMap, Telemetry telemetry) {
        vertical = new Motor(hardwareMap, Constants.depositorVerticalConfig);
        basket = new SimpleServo(hardwareMap, Constants.depositorBasketConfig, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    public void increaseStage() {
        if (currentStage == state.HIGH_BASKET) {
            return;
        }
        if (currentStage == state.HOME) {
            currentStage = state.MIDDLE_BASKET;
        }
        else {
            currentStage = state.HIGH_BASKET;
        }
    }

    public void decreaseStage() {
        if (currentStage == state.HOME) {
            return;
        }
        if (currentStage == state.HIGH_BASKET) {
            currentStage = state.MIDDLE_BASKET;
        }
        else {
            currentStage = state.HOME;
        }
    }

    @Override
    public void periodic() {
        telemetry.addData("Vertical Position", vertical.getCurrentPosition());
        telemetry.addData("Basket Position", basket.getAngle());
        telemetry.addData("Current Stage", currentStage);


        pidTarget = currentStage.pos;
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

    public boolean isAtPos() {
        telemetry.addData("At target", (vertical.getCurrentPosition() >= currentStage.pos - 150) && (vertical.getCurrentPosition() <= currentStage.pos + 150));
        return (vertical.getCurrentPosition() >= currentStage.pos - 150) && (vertical.getCurrentPosition() <= currentStage.pos + 150);
    }

    public void rawVertical(double speed) {
        vertical.setRunMode(Motor.RunMode.RawPower);
        vertical.set(speed);
    }

    public boolean basketToDeposit() {
        basket.turnToAngle(Constants.depositorBasketToDepositAngle);
        return false;
    }

    public boolean basketToHome() {
        basket.turnToAngle(Constants.depositorBasketToHomeAngle);
        return false;
    }
}
