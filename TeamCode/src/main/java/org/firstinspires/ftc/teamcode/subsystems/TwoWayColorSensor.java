package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class TwoWayColorSensor extends SubsystemBase {
    private Telemetry telemetry;
    private RevColorSensorV3 colorSensor;

    private double r = 0;
    private double g = 0;
    private double b = 0;

    public TwoWayColorSensor(HardwareMap hardwareMap, Telemetry telemetry) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, Constants.INTAKE_REV_COLOR_SENSOR_CONFIG);
        this.telemetry = telemetry;
    }

    public void update() {
        this.r = colorSensor.red();
        this.g = colorSensor.green();
        this.b = colorSensor.blue();
    }

    @Override
    public void periodic() {
        telemetry.addData("R", r);
        telemetry.addData("G", g);
        telemetry.addData("B", b);

        telemetry.addData("COLOR", getColor());
    }

    private boolean withinRange(double val, double min, double max) {
        return (val > min) && (val < max);
    }

    public boolean isYellow() {
        // 480, 570, 135
        return withinRange(r, 400, 500) && withinRange(g, 475, 570) && withinRange(b, 100, 200);
    }

    public boolean isRed() {
        // 300, 150, 80
        return withinRange(r,250, 350) && withinRange(g, 100, 200) && withinRange(b, 0, 100);
    }

    public boolean isBlue() {
        // 62, 124, 300
        return withinRange(r,0, 100) && withinRange(g, 100, 200) && withinRange(b, 280, 380);
    }

    public Intake.color getColor() {
        if (isYellow()) {
            return Intake.color.YELLOW;
        }
        else if (isRed()) {
            return Intake.color.RED;
        }
        else if (isBlue()) {
            return Intake.color.BLUE;
        }
        else {
            return Intake.color.NONE;
        }
    }
}
