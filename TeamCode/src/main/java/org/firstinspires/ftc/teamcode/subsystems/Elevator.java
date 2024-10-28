package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Elevator extends SubsystemBase {
    public enum state {
        HOME((int)Constants.depositorVerticalToBottomPose),
        MIDDLE_BASKET((int)Constants.depositorVerticalToMidPose),
        HIGH_BASKET((int)Constants.depositorVerticalToTopPose);

        public final int pos;
        private state(int pos) {
            this.pos = pos;
        }
    }

    private final Motor vertical;
    private final Telemetry telemetry;

    public state currentStage = state.HOME;
    private double pidTarget = 0;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        vertical = new Motor(hardwareMap, Constants.depositorVerticalConfig);

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
        telemetry.addData("Current Stage", currentStage);

        pidTarget = currentStage.pos;
        verticalToPos(pidTarget);

    }

    private void verticalToPos(double targetPos) {
        vertical.setRunMode(Motor.RunMode.RawPower);
        double kP = Constants.depositorVerticalKP;

        double currPos = vertical.getCurrentPosition();
        double error = targetPos - currPos;

        vertical.set(error * kP);

    }

    public boolean isAtPos() {
        telemetry.addData("At target", (vertical.getCurrentPosition() >= currentStage.pos - 150) && (vertical.getCurrentPosition() <= currentStage.pos + 150));
        return (vertical.getCurrentPosition() >= currentStage.pos - 150) && (vertical.getCurrentPosition() <= currentStage.pos + 150);
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
}
