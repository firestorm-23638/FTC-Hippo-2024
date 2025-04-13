package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Climb extends SubsystemBase {
    private final Motor leftMotor;
    private final Motor rightMotor;
    private double leftMotorHoldPos;
    private double rightMotorHoldPos;
    private boolean isLeftMotorHoldPos = false;
    private boolean isRightMotorHoldPos = false;
    private Telemetry telemetry;

    public Climb(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = new Motor(hardwareMap, Constants.CLIMB_LEFT_MOTOR_CONFIG);
        rightMotor = new Motor(hardwareMap, Constants.CLIMB_RIGHT_MOTOR_CONFIG);
        this.telemetry = telemetry;

        rightMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        telemetry.addData("Left pos", leftMotor.getCurrentPosition());
        telemetry.addData("Right pos", rightMotor.getCurrentPosition());

        if (isLeftMotorHoldPos) {
            holdLeftMotor();
        }
        if (isRightMotorHoldPos) {
            holdRightMotor();
        }
    }

    private void setLeftMotor(double perc) {
        leftMotor.set(perc);
    }

    private void setRightMotor(double perc) {
        rightMotor.set(perc);
    }

    public void requestSetLeftMotor(double perc) {
        if (!isLeftMotorHoldPos) {
            setLeftMotor(perc);
        }
    }

    public void requestSetRightMotor(double perc) {
        if (!isRightMotorHoldPos) {
            setRightMotor(perc);
        }
    }

    public void toggleHoldLeftMotor() {
        leftMotorHoldPos = leftMotor.getCurrentPosition();
    }

    public void toggleHoldRightMotor() {
        rightMotorHoldPos = rightMotor.getCurrentPosition();
    }

    public void holdLeftMotor() {
        setLeftMotor(leftMotorHoldPos - leftMotor.getCurrentPosition() * 0.25);
    }

    public void holdRightMotor() {
        setRightMotor(rightMotorHoldPos - rightMotor.getCurrentPosition() * 0.25);
    }
}
