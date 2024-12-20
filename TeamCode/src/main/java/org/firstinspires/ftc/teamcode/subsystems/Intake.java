package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
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
    public Gamepad gamepad;
    public boolean isGamepad = false;
    public double trim = 0;
    public color currentColor = color.NONE;
    public boolean updatingColor = false;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, color colorToIgnore) {
        this(hardwareMap, telemetry, colorToIgnore, null);
        isGamepad = false;
    };

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, color colorToIgnore, Gamepad gamepad) {
        leftHorizontal = new SimpleServo(hardwareMap, Constants.intakeLeftHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        rightHorizontal = new SimpleServo(hardwareMap, Constants.intakeRightHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        pivot = new SimpleServo(hardwareMap, Constants.intakePivotConfig, 0, 220, AngleUnit.DEGREES);
        leftVacuum = new CRServo(hardwareMap, Constants.intakeLeftVacuumConfig);
        rightVacuum = new CRServo(hardwareMap, Constants.intakeRightVacuumConfig);
        colorSensor = hardwareMap.get(ColorSensor.class, "intakeColor");

        isGamepad = true;
        this.gamepad = gamepad;

        this.colorToIgnore = colorToIgnore;

        this.telemetry = telemetry;
    }

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

    public void setTrim(double val) {
        trim = val;
    }

    public void updateColorSensor(boolean val) {
        updatingColor = val;
    }

    // The vacuum will only run if the color sensor detects nothing. Otherwise, it will either eject or stop and bring the intake back.
    @Override
    public void periodic() {
        if (updatingColor) {
            currentColor = getCurrentColor();
        }

        if (isGamepad) {
            trim = (gamepad.left_trigger * 65);
        }
        else {
            trim = 20;
        }
        telemetry.addData("Intake Trim", trim);
        telemetry.addData("ALLIANCE", Constants.isRed ? "RED" : "BLUE");
        telemetry.addData("Current Color", currentColor);

        if (intakeState == state.INTAKING) {
            horizontalOut();
        }
        if ((currentColor == color.NONE) || (currentColor == colorToIgnore)) {
            if (vacuumState == vacuum.SUCKING) {
                runVacuumRun();
            }
            else {
                runVacuumStop();
            }
        }
        else {
            runVacuumStop();
            vacuumState = vacuum.STATIONING;
            intakeState = state.RESTING;
            horizontalIn();
            pivotHome();
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
        horizontalToPos(Constants.intakeHorizontalToIntakePose + trim);
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
        pivotToPos(Constants.intakePivotToRest);
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

    public color getCurrentColor() {//258 365 170
        double colRed = colorSensor.red();
        double colGreen = colorSensor.green();//878 1060 258
        double colBlue = colorSensor.blue();

        telemetry.addData("RED", colorSensor.red());
        telemetry.addData("GREEN", colorSensor.green());
        telemetry.addData("BLUE", colorSensor.blue());

        if (withinRange(colRed, 750, 1500) && withinRange(colGreen, 800, 1400) && withinRange(colBlue, 150, 450)) {
            return color.YELLOW;
        }
        else if (withinRange(colRed, 90, 220) && withinRange(colGreen, 250, 400) && withinRange(colBlue, 550, 750)) {
            return color.BLUE;
        }
        else if (withinRange(colRed, 480, 660) && withinRange(colGreen, 300, 450) && withinRange(colBlue, 150, 300)) {
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
