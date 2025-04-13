package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

/*
EXTENSION SERVO CONFIGURATION:
Left:
    P - 23
    I - 13
    D - 55
    I - 10
    LeftRange - 0
    RightRange - 127.5
    Direction - 0
    Power - 450
Right:
    same as left, but Direction - 1
 */

public class Intake extends SubsystemBase {
    private final ServoEx leftHorizontal;
    private final ServoEx rightHorizontal;
    private final ServoEx pivot;
    private final ServoEx blocker;
    private final CRServo leftVacuum;
    private final CRServo rightVacuum;
    private final Telemetry telemetry;
    private final TwoWayColorSensor colorSensor;
    private final AnalogInput pivotEncoder;
    private final AnalogInput extensionEncoder;
    private final LynxModule controlHub;

    public enum state {
        INTAKING,
        DELAYED_INTAKING,
        RESTING,
        SPECIMEN,
        VERTICAL_TRANSITION,
        PRIME_TRANSFERRING,
        VERTICAL_TRANSFERRING_AND_BARFING,
        BARFING,
        SLIGHTLY,
        DOWN_EJECTING
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
        RED_YELLOW,
        BLUE_YELLOW,
        NONE
    }

    public state currentState = state.RESTING;
    public vacuum vacuumState = vacuum.STATIONING;
    public color targetColor = color.BLUE_YELLOW;
    public Timing.Timer blockerTimer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
    public Timing.Timer intakeLowerTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
    public Gamepad gamepad;
    public boolean isGamepad = false;
    public double trim = 20;
    public color currentColor = color.NONE;
    public boolean updatingColor = false;
    public boolean hasTheRightColor = false;
    public color[] colorSamples = new color[100];
    public short colorSampleAmt = 0;
    public color finalSampleColor;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this(hardwareMap, telemetry, null);
        isGamepad = false;
    };

    public Intake(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        leftHorizontal  = new SimpleServo(hardwareMap, Constants.LEFT_EXTENSION_CONFIG, 0, 180, AngleUnit.DEGREES);
        rightHorizontal = new SimpleServo(hardwareMap, Constants.RIGHT_EXTENSION_CONFIG, 0, 180, AngleUnit.DEGREES);
        blocker = new SimpleServo(hardwareMap, "blockerServo", 0, 180);
        pivotEncoder = hardwareMap.get(AnalogInput.class, "pivotEncoder");
        extensionEncoder = hardwareMap.get(AnalogInput.class, "extensionEncoder");
        pivot = new SimpleServo(hardwareMap, Constants.INTAKE_PIVOT_CONFIG, 0, 220, AngleUnit.DEGREES);
        leftVacuum = new CRServo(hardwareMap, Constants.INTAKE_LEFT_VACUUM_CONFIG);
        rightVacuum = new CRServo(hardwareMap, Constants.INTAKE_RIGHT_VACUUM_CONFIG);
        colorSensor = new TwoWayColorSensor(hardwareMap, telemetry);
        controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        isGamepad = true;
        this.gamepad = gamepad;

        this.telemetry = telemetry;
    }

    // These functions directly command the vacuum to run. They are (and should) only be used internally.
    private void runVacuumEject() {
        leftVacuum.set(Constants.INTAKE_EJECT_SPEED);
        rightVacuum.set(-Constants.INTAKE_EJECT_SPEED);
    }

    private void runVacuumRun() {
        leftVacuum.set(Constants.INTAKE_SUCK_SPEED);
        rightVacuum.set(-Constants.INTAKE_SUCK_SPEED);
    }

    public void runVacuumStop() {
        leftVacuum.set(0);
        rightVacuum.set(0);
    }

    public void runVacuumInch() {
        leftVacuum.set(Constants.INTAKE_INCHING_SPEED);
        rightVacuum.set(-Constants.INTAKE_INCHING_SPEED);
    }

    public void blockerUp() {
        blocker.turnToAngle(90);
    }

    public void blockerDown() {
        blocker.turnToAngle(45);
    }

    public void setTargetColor(color color) {
        targetColor = color;
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

    public double getControlHubMilliamps() {
        return controlHub.getCurrent(CurrentUnit.MILLIAMPS);
    }

    // The vacuum will only run if the color sensor detects nothing. Otherwise, it will either eject or stop and bring the intake back.
    @Override
    public void periodic() {
        telemetry.addData("Current", getControlHubMilliamps());
        telemetry.addData("Current Recorded Color", currentColor);
        telemetry.addData("TRIM", trim);

        if (isGamepad) {
            trim = (gamepad.left_trigger * 70);
        }

        if (currentState != state.INTAKING) {
            intakeLowerTimer = new Timing.Timer(200, TimeUnit.MILLISECONDS);
            intakeLowerTimer.start();
        }

        switch (currentState) {
            case INTAKING:
                colorSensor.update();
                currentColor = getCurrentColor();

                if (intakeLowerTimer.done()) {
                    pivotDown();
                }
                horizontalOut();

                if (!intakeEmpty()) {
                    runVacuumStop();
                    if (!hasCorrectColor()) {
                        blockerUp();
                        runVacuumRun();
                        blockerTimer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
                        blockerTimer.start();
                    }
                    else if ((currentColor != color.NONE)) {
                        currentState = state.RESTING;
                    }
//                    if (colorSampleAmt < 100) {
//                        colorSamples[colorSampleAmt] = currentColor;
//                        colorSampleAmt ++;
//                    }
//                    else {
//                        double yellowOcc = 0;
//                        double redOcc = 0;
//                        double blueOcc = 0;
//                        for (int x = 0; x < 1000; x ++) {
//                            Intake.color c = colorSensor.getColor();
//                            if (c == Intake.color.BLUE) {
//                                blueOcc ++;
//                            }
//                            else if (c == Intake.color.YELLOW) {
//                                yellowOcc ++;
//                            }
//                            else if (c == Intake.color.RED) {
//                                redOcc ++;
//                            }
//                        }
//                        if ((yellowOcc > redOcc) && (yellowOcc > blueOcc)) {
//                            finalSampleColor = Intake.color.YELLOW;
//                        }
//                        else if ((redOcc > yellowOcc) && (redOcc > blueOcc)) {
//                            finalSampleColor = Intake.color.RED;
//                        }
//                        else if ((blueOcc > yellowOcc) && (blueOcc > redOcc)) {
//                            finalSampleColor = Intake.color.BLUE;
//                        }
//                        else {
//                            finalSampleColor = Intake.color.NONE;
//                        }
//                    }
//                    if (finalSampleColor == colorToIgnore) {
//                        blockerUp();
//                        runVacuumRun();
//                        blockerTimer = new Timing.Timer(300, TimeUnit.MILLISECONDS);
//                        blockerTimer.start();
//                    }
//                    else if ((finalSampleColor != color.NONE)) {
//                        currentState = state.RESTING;
//                    }
                }
                else {
                    if (blockerTimer.done() || (!blockerTimer.isTimerOn())) {
                        blockerDown();
                    }
                    runVacuumRun();
                }
                break;
            case RESTING:
                blockerDown();
                pivotHome();
                horizontalIn();
                runVacuumStop();
                break;
            case VERTICAL_TRANSITION:
                pivotTransfer();
                horizontalTransfer();
                break;
            case VERTICAL_TRANSFERRING_AND_BARFING:
                pivotTransfer();
                horizontalTransfer();
                runVacuumEject();
                break;
            case BARFING:
                horizontalOut();
                pivotEject();
                runVacuumEject();
                blockerUp();
                break;
            case DOWN_EJECTING:
                horizontalOut();
                pivotDown();
                runVacuumEject();
                blockerUp();
                break;
            case SPECIMEN:
                pivotSpecimen();
                break;
        }

        telemetry.addData("beam", intakeEmpty());
        telemetry.addData("target color", targetColor);
    }

    public boolean hasCorrectColor() {
        if (targetColor == color.BLUE_YELLOW) {
            return (currentColor == color.BLUE) || (currentColor == color.YELLOW);
        }
        else if (targetColor == color.RED_YELLOW) {
            return (currentColor == color.RED) || (currentColor == color.YELLOW);
        }
        else if (targetColor == color.BLUE) {
            return (currentColor == color.BLUE);
        }
        else if (targetColor == color.RED) {
            return (currentColor == color.RED);
        }
        return false;
    }

    public boolean intakeEmpty() {
        return currentColor == color.NONE;
    }

    private void horizontalToPos(double targetPos) {
        leftHorizontal.turnToAngle(targetPos);
        rightHorizontal.turnToAngle(targetPos);
    }

    private void pivotToPos(double targetPos) {
        pivot.turnToAngle(targetPos);
    }

    public void horizontalIn() {
        horizontalToPos(Constants.EXTENSION_IN_ANGLE + trim);
    }

    public void horizontalPrimeTransfer() {
        horizontalToPos(Constants.EXTENSION_TRANSFER_ANGLE);
    }

    public void horizontalTransfer() {
        horizontalToPos(Constants.EXTENSION_TRANSFER_ANGLE);
    }

    public void horizontalOut() {
        horizontalToPos(Constants.EXTENSION_OUT_ANGLE + trim);
    }

    public void horizontalOut(double t) {
        horizontalToPos(Constants.EXTENSION_OUT_ANGLE + t);
    }

    public void pivotDown() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_DOWN_ANGLE);
    }

    public void pivotEject() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_EJECT_ANGLE);
    }

    public void pivotTransfer() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_TRANSITION_ANGLE);
    }

    public void pivotSpecimen() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_SPECIMEN_ANGLE);
    }

    public void pivotBasket() {

    }

    public void pivotHome() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_REST_ANGLE);
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
            hPose = Constants.EXTENSION_OUT_ANGLE + trim;
            pPose = Constants.INTAKE_PIVOT_TO_DOWN_ANGLE;
        }
        else if (state == Intake.state.RESTING){
            hPose = Constants.EXTENSION_IN_ANGLE;
            pPose = Constants.INTAKE_PIVOT_TO_REST_ANGLE;
        }
        else if (state == Intake.state.VERTICAL_TRANSITION){
            hPose = Constants.EXTENSION_IN_ANGLE;
            pPose = Constants.INTAKE_PIVOT_TO_TRANSITION_ANGLE;
        }

        return isAtPos(hPose, pPose);
    }

    public color getCurrentColor() {
        return colorSensor.getColor();
    }

    public double getExtensionPos() {
        return extensionEncoder.getVoltage() / 3.3 * 360;
    }

    public double getPivotPos() {
        return pivotEncoder.getVoltage() / 3.3 * 360;
    }

    public void extendSlightly() {
        horizontalToPos(Constants.EXTENSION_IN_ANGLE + 40);
    }
}
