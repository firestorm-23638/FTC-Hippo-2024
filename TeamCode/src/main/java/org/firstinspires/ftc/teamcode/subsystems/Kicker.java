package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Kicker extends SubsystemBase {
    private final ServoEx servo;
    public boolean isOpen = false;

    public enum state {
        OPEN,
        CLOSE,
        PUSH
    }

    public state currentState = state.CLOSE;

    public Kicker(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = new SimpleServo(hardwareMap, Constants.KICKER_CONFIG, 0, 180, AngleUnit.DEGREES);
    };

    @Override
    public void periodic() {
        if (currentState == state.OPEN) {
            open();
        }
        else if (currentState == state.CLOSE) {
            close();
        }
        else {
            push();
        }
    }

    public void toggle() {
        if (isOpen) {
            close();
        }
        else {
            open();
        }
        isOpen = !isOpen;
    }

    public void open() {
        servo.turnToAngle(90);
    }

    public void close() {
        servo.turnToAngle(0);
    }

    public void push() {
        servo.turnToAngle(65);
    }
}