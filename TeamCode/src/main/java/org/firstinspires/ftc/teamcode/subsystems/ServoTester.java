package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class ServoTester extends SubsystemBase {
    private List<ServoSettings> servoSettings;
    private int currentServoIndex;
    private Telemetry telemetry;

    public ServoTester(HardwareMap hardwareMap, Telemetry telemetry, String[] servoNames) {
        this.telemetry = telemetry;

        for (String s : servoNames) {
            servoSettings.add(new ServoSettings(s, hardwareMap.get(ServoEx.class, s)));
        }
    }

    @Override
    public void periodic() {
        ServoSettings currentServo = servoSettings.get(currentServoIndex);

        telemetry.addData("SERVO TESTER", "Developed by Firestorm");
        telemetry.addData("HOW TO USE", "");
        telemetry.addData(" - Left and Right DPad", "Switch Between Servos");
        telemetry.addData(" - Up and Down DPad", "Increment/Decrement Degree");
        telemetry.addData(" - A", "Toggle Set");
        telemetry.addData("", "");
        telemetry.addData("VALUES", "");
        telemetry.addData("CURRENT SERVO", currentServo.name);
        telemetry.addData("IS SETTING", currentServo.set);
        telemetry.addData("SETTING ANGLE", currentServo.deg);

        if (currentServo.set) {
            currentServo.obj.turnToAngle(currentServo.deg);
        }
    }

    public void toggleSet() {
        servoSettings.get(currentServoIndex).set = !servoSettings.get(currentServoIndex).set;
    }

    public void incrementDegree() {
        servoSettings.get(currentServoIndex).deg ++;
    }

    public void decrementDegree() {
        servoSettings.get(currentServoIndex).deg --;
    }

    public void incrementServo() {
        currentServoIndex ++;
        if (currentServoIndex > (servoSettings.size()-1)) {
            currentServoIndex = 0;
        }
    }

    public void decrementServo() {
        currentServoIndex --;
        if (currentServoIndex < 0) {
            currentServoIndex = servoSettings.size()-1;
        }
    }
}

class ServoSettings {
    public String name;
    public boolean set;
    public double deg;
    public ServoEx obj;

    public ServoSettings(String name, ServoEx obj) {
        this.name = name;
        this.set = false;
        this.deg = 0;
        this.obj = obj;
    }
}