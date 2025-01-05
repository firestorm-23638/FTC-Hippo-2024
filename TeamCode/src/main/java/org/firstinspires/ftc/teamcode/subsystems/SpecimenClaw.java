package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

public class SpecimenClaw extends SubsystemBase {
    private final ServoEx servo;
    boolean mode = false;

    public SpecimenClaw(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = new SimpleServo(hardwareMap, Constants.specimenClawConfig, 0, 180, AngleUnit.DEGREES);
    }

    public void open() {
        servo.turnToAngle(Constants.specimenOpenAngle);
    }

    public void close() {
        servo.turnToAngle(Constants.specimenCloseAngle);
    }
}
