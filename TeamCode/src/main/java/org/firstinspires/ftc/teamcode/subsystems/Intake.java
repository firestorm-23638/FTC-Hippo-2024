package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

public class Intake extends SubsystemBase {
    private final ServoEx leftHorizontal;
    private final ServoEx rightHorizontal;
    private final ServoEx pivot;
    private final CRServo leftVacuum;
    private final CRServo rightVacuum;
    private final Telemetry telemetry;
    private final ColorSensor colorSensor;

    public enum state {
        INTAKING,
        RESTING,
        TRANSFERRING,
        SLIGHTLY
    }

    public enum vacuum {
        SUCKING,
        SPEWING,
        STATIONING
    }

    public enum color {
        RED,
        YELLOW,
        BLUE,
        NONE
    }

    public state intakeState = state.RESTING;
    public vacuum vacuumState = vacuum.STATIONING;
    public color colorToIgnore;
    public boolean ejecting = false;
    public Timing.Timer ejectTimer;
    public boolean extended = false;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, color colorToIgnore) {
        leftHorizontal = new SimpleServo(hardwareMap, Constants.intakeLeftHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        rightHorizontal = new SimpleServo(hardwareMap, Constants.intakeRightHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        pivot = new SimpleServo(hardwareMap, Constants.intakePivotConfig, 0, 220, AngleUnit.DEGREES);
        leftVacuum = new CRServo(hardwareMap, Constants.intakeLeftVacuumConfig);
        rightVacuum = new CRServo(hardwareMap, Constants.intakeRightVacuumConfig);
        colorSensor = hardwareMap.get(ColorSensor.class, "intakeColor");

        this.colorToIgnore = colorToIgnore;

        this.telemetry = telemetry;
    };

    // These functions directly command the vacuum to run. They are (and should) only be used internally.
    private void runVacuumEject() {
        leftVacuum.set(-Constants.intakeVacuumSpeed);
        rightVacuum.set(Constants.intakeVacuumSpeed);
    }

    private void runVacuumRun() {
        leftVacuum.set(Constants.intakeVacuumSpeed);
        rightVacuum.set(-Constants.intakeVacuumSpeed);
    }

    public void runVacuumStop() {
        leftVacuum.set(0);
        rightVacuum.set(0);
    }

    private boolean withinRange(double val, double min, double max) {
        return (val > min) && (val < max);
    }

    // The vacuum will only run if the color sensor detects nothing. Otherwise, it will either eject or stop and bring the intake back.
    @Override
    public void periodic() {
        telemetry.addData("ALLIANCE", Constants.isRed ? "RED" : "BLUE");
        telemetry.addData("Current Color", getCurrentColor());

        if (ejecting) {
            runVacuumEject();

            if (ejectTimer.done()) {
                ejecting = false;
            }
        }
        else if (vacuumState == vacuum.SPEWING) {
            runVacuumEject();
        }
        else {
            if (getCurrentColor() == color.NONE) {
                if (vacuumState == vacuum.SUCKING) {
                    runVacuumRun();
                }
                else {
                    runVacuumStop();
                }
            }
            else if (getCurrentColor() == colorToIgnore) {
                ejecting = true;
                ejectTimer = new Timing.Timer(500, TimeUnit.MILLISECONDS);
                ejectTimer.start();
            }
            else {
                runVacuumStop();
                vacuumState = vacuum.STATIONING;
                intakeState = state.RESTING;
                horizontalIn();
                pivotHome();
            }
        }
    }

    private void horizontalToPos(double targetPos) {
        leftHorizontal.turnToAngle(targetPos);
        rightHorizontal.turnToAngle(targetPos);
    }

    private void pivotToPos(double targetPos) {
        pivot.turnToAngle(targetPos);
    }

    public void horizontalIn() {
        horizontalToPos(Constants.intakeHorizontalToHomePose);
        intakeState = state.RESTING;
    }

    public void horizontalOut() {
        horizontalToPos(Constants.intakeHorizontalToIntakePose);
        intakeState = state.INTAKING;
    }

    public void pivotDown() {
        pivotToPos(Constants.intakePivotToDown);
    }

    public void pivotBasket() {
        if (intakeState == state.RESTING) {
            pivotToPos(Constants.intakePivotToBasket);
        }
    }

    public void pivotHome() {
        if ((vacuumState == vacuum.SPEWING) && (intakeState == state.INTAKING)) {
            return;
        }
        pivotToPos(Constants.intakePivotToBasket);
    }

    // Vacuum: these functions set the vacuum mode. However, this mode may be overridden.
    // For example: if the color sensor detects the wrong piece, the vacuum will eject.
    // Or, if the color sensor detects the right piece, the vacuum will stop.
    public void setVacuumEject() {
        vacuumState = vacuum.SPEWING;
    }

    public void setVacuumRun() {
        vacuumState = vacuum.SUCKING;
    }

    public void setVacuumStop() {
        vacuumState = vacuum.STATIONING;
    }

    public color getCurrentColor() {
        double colRed = colorSensor.red();
        double colGreen = colorSensor.green();
        double colBlue = colorSensor.blue();

        telemetry.addData("RED", colorSensor.red());
        telemetry.addData("GREEN", colorSensor.green());
        telemetry.addData("BLUE", colorSensor.blue());

        if (withinRange(colRed, 940, 1500) && withinRange(colGreen, 950, 1250) && withinRange(colBlue, 220, 310)) {
            return color.YELLOW;
        }
        else if (withinRange(colRed, 90, 150) && withinRange(colGreen, 220, 290) && withinRange(colBlue, 510, 650)) {
            return color.BLUE;
        }
        else if (withinRange(colRed, 480, 640) && withinRange(colGreen, 255, 340) && withinRange(colBlue, 130, 200)) {
            return color.RED;
        }
        else {
            return color.NONE;
        }
    }

    public void extendSlightly() {
        horizontalToPos(Constants.intakeHorizontalToHomePose + 40);
    }
}
