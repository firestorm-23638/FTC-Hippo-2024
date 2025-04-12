package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class LightIndicator extends SubsystemBase {
    private Servo port0;
    private Servo port1;

    public enum state {
        RED,
        BLUE,
        YELLOW,
        WHITE,
        OFF
    }

    public LightIndicator(HardwareMap hardwareMap, Telemetry telemetry) {
        port0 = hardwareMap.get(Servo.class, Constants.LIGHT_INDICATOR0_CONFIG);
        port1 = hardwareMap.get(Servo.class, Constants.LIGHT_INDICATOR1_CONFIG);
    }

    public void setState(state s) {
        switch (s) {
            case OFF:
                turnOff();
                break;
            case RED:
                setRed();
                break;
            case WHITE:
                setWhite();
                break;
            case BLUE:
                setBlue();
                break;
            case YELLOW:
                setYellow();
                break;
        }
    }

    private void turnOff() {
        port0.setPosition(0);
        port1.setPosition(0);
    }

    private void setRed() {
        port0.setPosition(0.279);
        port1.setPosition(0.279);
    }

    private void setBlue() {
        port0.setPosition(0.611);
        port1.setPosition(0.611);
    }

    private void setYellow() {
        port0.setPosition(0.388);
        port1.setPosition(0.388);
    }

    private void setWhite() {
        port0.setPosition(1);
        port1.setPosition(1);
    }
}
