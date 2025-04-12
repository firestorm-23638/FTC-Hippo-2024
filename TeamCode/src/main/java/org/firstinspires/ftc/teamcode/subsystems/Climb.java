package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Climb extends SubsystemBase {
    private final MotorEx leftMotor;
    private final MotorEx rightMotor;

    public Climb(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(MotorEx.class, Constants.CLIMB_LEFT_MOTOR_CONFIG);
        rightMotor = hardwareMap.get(MotorEx.class, Constants.CLIMB_RIGHT_MOTOR_CONFIG);

        rightMotor.setInverted(true);
    }

    public void setLeftMotor(double perc) {
        leftMotor.set(perc);
    }

    public void setRightMotor(double perc) {
        rightMotor.set(perc);
    }
}
