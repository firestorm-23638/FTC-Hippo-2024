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

public class Intake extends SubsystemBase {
    private final ServoEx leftHorizontal;
    private final ServoEx rightHorizontal;
    private final ServoEx pivot;
    private final CRServo leftVacuum;
    private final CRServo rightVacuum;
    private final Telemetry telemetry;
    private final ColorSensor colorSensor;
    private final DigitalChannel beamBrake;

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
    public Timing.Timer ejectTimer = new Timing.Timer(150, TimeUnit.MILLISECONDS);
    public boolean extended = false;
    public Gamepad gamepad;
    public boolean isGamepad = false;
    public double trim = 20;
    public color currentColor = color.NONE;
    public boolean updatingColor = false;
    public boolean colorDetected = false;
    public boolean hasTheRightColor = false;

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
        beamBrake = hardwareMap.get(DigitalChannel.class, Constants.intakeBeamBreakConfig);

        isGamepad = true;
        this.gamepad = gamepad;

        this.colorToIgnore = colorToIgnore;

        this.telemetry = telemetry;
    }

    // These functions directly command the vacuum to run. They are (and should) only be used internally.
    private void runVacuumEject() {
        leftVacuum.set(Constants.intakeEjectSpeed);
        rightVacuum.set(-Constants.intakeEjectSpeed);
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
        if (isGamepad) {
            trim = (gamepad.left_trigger * 80);
        }
        telemetry.addData("BEAM BREAK", beamBrake.getState());
        telemetry.addData("EXTENSION POS", rightHorizontal.getAngle());
        telemetry.addData("WRIST POS", pivot.getAngle());

        telemetry.addData("Current Color", currentColor);
        currentColor = getCurrentColor();

        if (intakeState == state.INTAKING) {
            horizontalOut();
            if (((intakeState == state.INTAKING) && (!beamBrake.getState() && (!colorDetected)))) {   // if it has a piece and it doesn't know the color yet
                hasTheRightColor = false;
                leftVacuum.set(.3);
                rightVacuum.set(-.3);
                if (currentColor != color.NONE) {
                    colorDetected = true;
                }
            }
            else if ((colorDetected)) {
                if (currentColor == color.NONE) {
                    colorDetected = false;
                }
                else if (currentColor == colorToIgnore) {
                    runVacuumRun();
                }
                else {
                    hasTheRightColor = true;
                    runVacuumStop();
                    horizontalIn();
                    pivotHome();
                }
            }
            else if (vacuumState == vacuum.SUCKING) {
                runVacuumRun();
            }
            else if (vacuumState == vacuum.SPEWING) {
                runVacuumEject();
            }
        }
        else if ((trim > 50) && (vacuumState == vacuum.SPEWING)) {
            runVacuumRun();
        }
        else if (vacuumState == vacuum.SPEWING) {
            runVacuumEject();
        }
        else {
            runVacuumStop();
            horizontalIn();
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
        horizontalToPos(Constants.intakeHorizontalToHomePose + trim);
        intakeState = state.RESTING;
    }

    public void horizontalTransfer() {
        horizontalToPos(Constants.intakeHorizontalToHomePose - 10);
    }

    public void horizontalOut() {
        horizontalToPos(Constants.intakeHorizontalToIntakePose + trim);
        intakeState = state.INTAKING;
    }

    public void horizontalOut(double t) {
        horizontalToPos(Constants.intakeHorizontalToIntakePose + t);
        intakeState = state.INTAKING;
    }

    public double getPivotPos() {
        return pivot.getAngle();
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

        if (withinRange(colRed, 500, 1500) && withinRange(colGreen, 750, 1400) && withinRange(colBlue, 100, 450)) {
            return color.YELLOW;
        }
        else if (withinRange(colRed, 50, 150) && withinRange(colGreen, 150, 300) && withinRange(colBlue, 400, 650)) { //98 204 516   89 182 461   89 182 457
            return color.BLUE;
        }
        else if (withinRange(colRed, 350, 550) && withinRange(colGreen, 150, 400) && withinRange(colBlue, 60, 200)) {  //456 226 114
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
