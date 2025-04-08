package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class SpecimenClaw extends SubsystemBase {
    private final ServoEx servo;
    boolean mode = false;

    public SpecimenClaw(HardwareMap hardwareMap, Telemetry telemetry) {
        servo = new SimpleServo(hardwareMap, Constants.MAIN_CLAW_CONFIG, 0, 180, AngleUnit.DEGREES);
    }

    public void open() {
        servo.turnToAngle(Constants.MAIN_CLAW_OPEN_ANGLE);
    }

    public void close() {
        servo.turnToAngle(Constants.MAIN_CLAW_CLOSE_ANGLE);
    }
}
