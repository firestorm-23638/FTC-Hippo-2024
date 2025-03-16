package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

/*
Elevator: The elevator subsystem for the robot.
*/

public class Elevator extends SubsystemBase {
    public enum basketState {    // The current "state" or position of the elevator
        HOME((int)Constants.ELEVATOR_HOME_POS),
        MIDDLE_BASKET((int)Constants.ELEVATOR_LOW_BASKET_POS),
        SPECIMEN((int)400),
        HIGH_BASKET((int)Constants.ELEVATOR_HIGH_BASKET_POS);

        public final int pos;
        basketState(int pos) {
            this.pos = pos;
        }
    }

    public boolean isBarState = false;
    public boolean isSlightState = false;
    public boolean isZeroed = false;
    private double trim = 0;

    public final Motor vertical;     // The motor for the elevator
    public final Motor secondVertical;
    private final Telemetry telemetry;    // Telemetry class for printouts
    private final DigitalChannel limitSwitch;
    private final SpecimenClaw claw;
    private Gamepad gamepad;

    public basketState currentStage = basketState.HOME;

    private double pidTarget = 0;
    private boolean isGamepad = false;

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        vertical = new Motor(hardwareMap, Constants.ELEVATOR_MOTOR_CONFIG);
        secondVertical = new Motor(hardwareMap, Constants.ELEVATOR_2ND_MOTOR_CONFIG);
        limitSwitch = hardwareMap.get(DigitalChannel.class, "elevatorLimit");
        claw = new SpecimenClaw(hardwareMap, telemetry);
        this.gamepad = gamepad;
        isGamepad = true;

        secondVertical.setInverted(true);

        this.telemetry = telemetry;
    };

    public Elevator(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
        isGamepad = false;
    };

    public void increaseStage() {       // Increases the stage of the elevator
        if (currentStage == basketState.HIGH_BASKET) {
            return;
        }
        if (currentStage == basketState.SPECIMEN) {
            currentStage = basketState.HIGH_BASKET;
        }
        else if (currentStage == basketState.MIDDLE_BASKET) {
            currentStage = basketState.SPECIMEN;
        }
        else if (currentStage == basketState.HOME) {
            currentStage = basketState.MIDDLE_BASKET;
        }
        else {
            currentStage = basketState.HIGH_BASKET;
        }
    }

    public void decreaseStage() {      // Decreases the stage of the elevator
        if (currentStage == basketState.MIDDLE_BASKET) {
            return;
        }
        if (currentStage == basketState.HIGH_BASKET) {
            currentStage = basketState.SPECIMEN;
        }
        else if (currentStage == basketState.SPECIMEN) {
            currentStage = basketState.MIDDLE_BASKET;
        }
        else {
            currentStage = basketState.MIDDLE_BASKET;
        }
    }

    public void setTrim(double amt) {
        trim = amt;
    }

    @Override
    public void periodic() {         // Constantly runs the PID algorithm based on the currentStage
        telemetry.addData("Vertical Position", vertical.getCurrentPosition());
        telemetry.addData("Current Stage", currentStage);
        telemetry.addData("Vertical %", vertical.get());
        telemetry.addData("Limit Switch", limitSwitch.getState());

//        if (isSlightState) {
//            pidTarget = Constants.depositorVerticalUpSlightly;
//        }
//        else if (isBarState) {
//            pidTarget = Constants.depositorVerticalToTopBar;
//        }
        //else {
        pidTarget = currentStage.pos;
        //}
        if (isGamepad) {
            trim = (gamepad.right_trigger * 275);
        }
        if (trim > 175) {
            claw.open();
        }
        telemetry.addData("Trim", trim);

        if ((currentStage == basketState.HOME) || currentStage == basketState.MIDDLE_BASKET) {
            if (!isZeroed) {
                if (!limitSwitch.getState()) {
                    moveVertical(0);
                    resetPosition();
                    isZeroed = true;
                }
                else {
                    moveVertical(-0.5);
                }
            }
            else {
                verticalToPos(pidTarget - trim);
            }
        }
        else {
            verticalToPos(pidTarget - trim);    // Runs PID algorithm
        }

    }

    private void resetPosition() {
        vertical.resetEncoder();
    }

    private void verticalToPos(double targetPos) {     // PID algorithm function. There is just a kP term for now
        vertical.setRunMode(Motor.RunMode.RawPower);
        double kP = Constants.ELEVATOR_PID_P_TERM;

        double currPos = getPosition();
        double error = targetPos - currPos;

        moveVertical(error * kP);

    }

    public double getPosition() {
        return vertical.getCurrentPosition();
    }

    public double getMaxRPM() {
        return vertical.getMaxRPM();
    }

    public boolean getInverted() {
        return vertical.getInverted();
    }

    public boolean isAtPos() {     // Checks to see if the elevator is at a current position, with an 80 tick deadband.
        return (getPosition() >= currentStage.pos - 80) && (getPosition() <= currentStage.pos + 80);
    }

    public void moveVertical(double speed) {           // Runs elevator at raw speed. For testing purposes
        vertical.setRunMode(Motor.RunMode.RawPower);
        secondVertical.setRunMode(Motor.RunMode.RawPower);
        if (speed > 1) {
            speed = 1;
        }
        if (speed < -1) {
            speed = -1;
        }
        vertical.set(speed);
        secondVertical.set(speed);
    }
}