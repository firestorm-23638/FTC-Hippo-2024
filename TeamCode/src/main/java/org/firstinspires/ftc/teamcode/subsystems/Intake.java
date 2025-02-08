package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

public class Intake extends SubsystemBase {
    private final ServoEx leftHorizontal;
    private final ServoEx rightHorizontal;
    private final ServoEx pivot;
    private final ServoEx blocker;
    private final CRServo leftVacuum;
    private final CRServo rightVacuum;
    private final Telemetry telemetry;
    private final ColorSensor colorSensor;
    private final DigitalChannel beamBrake;
    private final AnalogInput pivotEncoder;
    private final AnalogInput extensionEncoder;

    public enum state {
        INTAKING,
        RESTING,
        TRANSFERRING,
        BARFING,
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

    public state currentState = state.RESTING;
    public vacuum vacuumState = vacuum.STATIONING;
    public color colorToIgnore;
    public boolean ejecting = false;
    public Timing.Timer ejectTimer = new Timing.Timer(150, TimeUnit.MILLISECONDS);
    public Timing.Timer blockerTimer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
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
        blocker = new SimpleServo(hardwareMap, "blockerServo", 0, 180);
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
        extensionEncoder = hardwareMap.get(AnalogInput.class, "extensionEncoder");
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

    public void runVacuumInch() {
        leftVacuum.set(Constants.intakeInchingSpeed);
        rightVacuum.set(-Constants.intakeInchingSpeed);
    }

    public void blockerUp() {
        blocker.turnToAngle(90);
    }

    public void blockerDown() {
        blocker.turnToAngle(45);
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
            trim = (gamepad.left_trigger * 70);
        }

        currentColor = getCurrentColor();
/* WITHOUT CATCHER
        if (currentState == state.INTAKING) {
            pivotDown();
            horizontalOut();
            if ((!beamBrake.getState() && (!colorDetected))) {   // if it has a piece and it doesn't know the color yet
                hasTheRightColor = false;
                leftVacuum.set(.2);
                rightVacuum.set(-.2);
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
                    currentState = state.RESTING;
                }
            }
            else {
                runVacuumRun();
            }
        }
        else if (currentState == state.RESTING) {
            pivotHome();
            horizontalIn();
            if (!beamBrake.getState()) {
                runVacuumInch();
            }
            else {
                runVacuumStop();
            }
        }
        else if (currentState == state.TRANSFERRING) {
            runVacuumEject();
        }
        else if (currentState == state.BARFING) {
            horizontalOut();
            pivotEject();
            runVacuumRun();
        }*/
        //WITH CATCHER
        if (currentState == state.INTAKING) {
            pivotDown();
            horizontalOut();

            if (!beamBrake()) {
                runVacuumStop();
                if (currentColor == colorToIgnore) {
                    blockerUp();
                    runVacuumRun();
                    blockerTimer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
                    blockerTimer.start();
                }
                else if ((currentColor != color.NONE)) {
                    currentState = state.RESTING;
                }
            }
            else {
                if (blockerTimer.done() || (!blockerTimer.isTimerOn())) {
                    blockerDown();
                }
                runVacuumRun();
            }
        }
        else if (currentState == state.RESTING) {
            blockerUp();
            pivotHome();
            horizontalIn();
            if (!beamBrake.getState()) {
                runVacuumInch();
            }
            else {
                runVacuumStop();
            }
        }
        else if (currentState == state.TRANSFERRING) {
            runVacuumEject();
        }
        else if (currentState == state.BARFING) {
            horizontalOut();
            pivotEject();
            runVacuumRun();
            blockerUp();
        }

        telemetry.addData("beam", beamBrake());
    }

    public boolean beamBrake() {
        return beamBrake.getState();
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
    }

    public void horizontalTransfer() {
        horizontalToPos(Constants.intakeHorizontalToHomePose - 10);
    }

    public void horizontalOut() {
        horizontalToPos(Constants.intakeHorizontalToIntakePose + trim);
    }

    public void horizontalOut(double t) {
        horizontalToPos(Constants.intakeHorizontalToIntakePose + t);
    }

    public void pivotDown() {
        pivotToPos(Constants.intakePivotToDown);
    }

    public void pivotEject() {
        pivotToPos(Constants.intakePivotToEject);
    }

    public void pivotBasket() {
        if (currentState == state.RESTING) {
            pivotToPos(Constants.intakePivotToBasket);
        }
    }

    public void pivotHome() {
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

    public boolean isAtPos(double hP, double pP) {
        double hPose = hP;
        double pPose = pP;

        return ((rightHorizontal.getAngle() >= hPose - 1) && (rightHorizontal.getAngle() <= hPose + trim + 1)) && ((pivot.getAngle() >= pPose - 1) && (pivot.getAngle() <= pPose + trim + 1));
    }

    public boolean isAtPos(state state) {
        double hPose = 0;
        double pPose = 0;
        if (state == Intake.state.INTAKING) {
            hPose = Constants.intakeHorizontalToIntakePose + trim;
            pPose = Constants.intakePivotToDown;
        }
        else if (state == Intake.state.RESTING){
            hPose = Constants.intakeHorizontalToHomePose;
            pPose = Constants.intakePivotToRest;
        }
        else if (state == Intake.state.TRANSFERRING){
            hPose = Constants.intakeHorizontalToHomePose;
            pPose = Constants.intakePivotToBasket;
        }

        return isAtPos(hPose, pPose);
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

    public double getExtensionPos() {
        return extensionEncoder.getVoltage() / 3.3 * 360;
    }

    public double getPivotPos() {
        return pivotEncoder.getVoltage() / 3.3 * 360;
    }

    public void extendSlightly() {
        horizontalToPos(Constants.intakeHorizontalToHomePose + 40);
    }
}
