package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class SpecimenClaw extends SubsystemBase {
    private final ServoEx claw;     // Servo object

    private final Telemetry telemetry;     // Telemetry object, for printouts

    public SpecimenClaw(HardwareMap hardwareMap, Telemetry telemetry) {
        claw = new SimpleServo(hardwareMap, Constants.specimenClawConfig, 0, 180, AngleUnit.DEGREES);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        telemetry.addData("Servo Position", claw.getAngle());
    }

    public boolean open() {
        claw.turnToAngle(Constants.specimenOpenAngle);
        return false;
    }

    public boolean close() {
        claw.turnToAngle(Constants.specimenCloseAngle);
        return false;
    }
}
