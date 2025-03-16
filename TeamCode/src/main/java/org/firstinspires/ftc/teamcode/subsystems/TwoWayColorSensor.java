package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TwoWayColorSensor extends SubsystemBase {
    private DigitalChannel pin0;
    private DigitalChannel pin1;

    private boolean leftState = false;
    private boolean rightState = false;

    public TwoWayColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        pin0 = hardwareMap.digitalChannel.get("rangefinder0");
        pin1 = hardwareMap.digitalChannel.get("rangefinder1");
    }

    @Override
    public void periodic() {
        leftState = pin0.getState();
        rightState = pin1.getState();
    }

    public boolean isYellow() {
        return leftState && rightState;
    }

    public boolean isRed() {
        return leftState && !rightState;
    }

    public boolean isBlue() {
        return !leftState && rightState;
    }

    public boolean isNone() {
        return !leftState && !rightState;
    }
}
