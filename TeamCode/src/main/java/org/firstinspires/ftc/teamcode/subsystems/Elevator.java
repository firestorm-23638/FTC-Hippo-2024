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
Elevator: The elevator subsystem for the robot.
*/

public class Elevator extends SubsystemBase {
    public enum basketState {    // The current "state" or position of the elevator
        HOME((int)Constants.depositorVerticalToBottomPose),
        MIDDLE_BASKET((int)Constants.depositorVerticalToMidPose),
        HIGH_BASKET((int)Constants.depositorVerticalToTopPose);

        public final int pos;
        basketState(int pos) {
            this.pos = pos;
        }
    }

    public boolean isBarState = false;
    public boolean isSlightState = false;

    private final Motor vertical;     // The motor for the elevator

    private final Telemetry telemetry;    // Telemetry class for printouts

    public basketState currentStage = basketState.HOME;

    private double pidTarget = 0;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        vertical = new Motor(hardwareMap, Constants.depositorVerticalConfig);

        this.telemetry = telemetry;
    };

    public void increaseStage() {       // Increases the stage of the elevator
        if (currentStage == basketState.HIGH_BASKET) {
            return;
        }
        if (currentStage == basketState.HOME) {
            currentStage = basketState.MIDDLE_BASKET;
        }
        else {
            currentStage = basketState.HIGH_BASKET;
        }
    }

    public void decreaseStage() {      // Decreases the stage of the elevator
        if (currentStage == basketState.HOME) {
            return;
        }
        if (currentStage == basketState.HIGH_BASKET) {
            currentStage = basketState.MIDDLE_BASKET;
        }
        else {
            currentStage = basketState.HOME;
        }
    }

    @Override
    public void periodic() {         // Constantly runs the PID algorithm based on the currentStage
        telemetry.addData("Vertical Position", vertical.getCurrentPosition());
        telemetry.addData("Current Stage", currentStage);

        if (isSlightState) {
            pidTarget = Constants.depositorVerticalUpSlightly;
        }
        else if (isBarState) {
            pidTarget = Constants.depositorVerticalToTopBar;
        }
        else {
            pidTarget = currentStage.pos;
        }

        verticalToPos(pidTarget);    // Runs PID algorithm
    }

    private void verticalToPos(double targetPos) {     // PID algorithm function. There is just a kP term for now
        vertical.setRunMode(Motor.RunMode.RawPower);
        double kP = Constants.depositorVerticalKP;

        double currPos = vertical.getCurrentPosition();
        double error = targetPos - currPos;

        vertical.set(error * kP);

    }

    public boolean isAtPos() {     // Checks to see if the elevator is at a current position, with a 150 tick deadband.
        return (vertical.getCurrentPosition() >= currentStage.pos - 150) && (vertical.getCurrentPosition() <= currentStage.pos + 150);
    }

    public void moveVertical(double speed) {           // Runs elevator at raw speed. For testing purposes
        vertical.setRunMode(Motor.RunMode.RawPower);
        vertical.set(speed);
    }
}
