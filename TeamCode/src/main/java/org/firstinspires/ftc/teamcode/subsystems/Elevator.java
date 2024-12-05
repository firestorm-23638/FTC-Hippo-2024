package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/*
Elevator: The elevator subsystem for the robot.
*/

public class Elevator extends SubsystemBase {
    public enum BasketState {    // The current "state" or position of the elevator
        HOME((int)Constants.depositorVerticalToBottomPose, false),
        SLIGHT_STATE((int)Constants.depositorVerticalUpSlightly, true),
        MIDDLE_BASKET((int)Constants.depositorVerticalToMidPose, false),
        HIGH_BASKET((int)Constants.depositorVerticalToTopPose, false);

        public final int pos;
        public final boolean skip;
        BasketState(int pos, boolean skip) {
            this.pos = pos;
            this.skip = skip;
        }
    }

    public boolean isBarState = false;

    public final Motor vertical;     // The motor for the elevator

    private final Telemetry telemetry;    // Telemetry class for printouts

    private BasketState currentStage = BasketState.HOME;
    private BasketState desiredState  = BasketState.HOME;
    private double pidTarget = 0;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        vertical = new Motor(hardwareMap, Constants.depositorVerticalConfig);

        this.telemetry = telemetry;
    };

    private BasketState incrementState(BasketState state) {
        switch (state) {
            case HOME:
                return BasketState.SLIGHT_STATE;
            case SLIGHT_STATE:
                return BasketState.MIDDLE_BASKET;
            case MIDDLE_BASKET:
                return BasketState.HIGH_BASKET;
            default:
                return state;
        }
    }

    private BasketState decrementState(BasketState state) {
        switch (this.desiredState) {
            case SLIGHT_STATE:
                return BasketState.HOME;
            case MIDDLE_BASKET:
                return BasketState.SLIGHT_STATE;
            case HIGH_BASKET:
                return BasketState.MIDDLE_BASKET;
            default:
                return state;
        }
    }

    public BasketState increaseStage() {       // Increases the stage of the elevator
        BasketState state = incrementState(this.currentStage);

        // drivers shouldn't be able to go to the SLIGHT_STATE manually
        while(state.skip) {
            state = incrementState(state);
        }

        return state;
    }

    public BasketState decreaseStage() { // Decreases the stage of the elevator
        BasketState state = decrementState(this.currentStage);
        while(state.skip) {
            state = decrementState(state);
        }

        return state;
    }

    @Override
    public void periodic() {         // Constantly runs the PID algorithm based on the currentStage
        telemetry.addData("Vertical Position", vertical.getCurrentPosition());
        telemetry.addData("Current Stage", currentStage);
        telemetry.addData("Vertical %", vertical.get());

        if (isBarState) {
            pidTarget = Constants.depositorVerticalToTopBar;
        }
        else {
            pidTarget = desiredState.pos;
        }
        this.currentStage = computeCurrentState();

        verticalToPos(pidTarget); // Runs PID algorithm
    }

    private void verticalToPos(double targetPos) {     // PID algorithm function. There is just a kP term for now
        vertical.setRunMode(Motor.RunMode.RawPower);
        double kP = Constants.depositorVerticalKP;

        double currPos = vertical.getCurrentPosition();
        double error = targetPos - currPos;

        vertical.set(error * kP);

    }

    private BasketState computeCurrentState() {
        BasketState nextState = incrementState(this.currentStage);
        BasketState previousState = decrementState(this.currentStage);

        if (vertical.getCurrentPosition() >= nextState.pos) {
            return nextState;
        }

        if (vertical.getCurrentPosition() <= currentStage.pos) {
            return previousState;
        }

        return currentStage;
    }

    public boolean isAtPos() {     // Checks to see if the elevator is at a current position, with a 150 tick deadband.
        return (vertical.getCurrentPosition() >= desiredState.pos - 150) && (vertical.getCurrentPosition() <= desiredState.pos + 150);
    }

    public void moveVertical(double speed) {           // Runs elevator at raw speed. For testing purposes
        vertical.setRunMode(Motor.RunMode.RawPower);
        vertical.set(speed);
    }

    public BasketState getCurrentStage() {
        return this.currentStage;
    }

    public BasketState getDesiredState() {
        return desiredState;
    }

    public void setDesiredState(BasketState desiredState) {
        this.desiredState = desiredState;
    }
}