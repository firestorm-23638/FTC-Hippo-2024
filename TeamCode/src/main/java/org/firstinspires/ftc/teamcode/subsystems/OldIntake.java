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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

import java.util.concurrent.TimeUnit;

public class OldIntake extends SubsystemBase {
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

    public OldIntake(HardwareMap hardwareMap, Telemetry telemetry, color colorToIgnore) {
        this(hardwareMap, telemetry, colorToIgnore, null);
        isGamepad = false;
    };

    public OldIntake(HardwareMap hardwareMap, Telemetry telemetry, color colorToIgnore, Gamepad gamepad) {
        leftHorizontal = new SimpleServo(hardwareMap, Constants.LEFT_EXTENSION_CONFIG, 0, 180, AngleUnit.DEGREES);
        rightHorizontal = new SimpleServo(hardwareMap, Constants.RIGHT_EXTENSION_CONFIG, 0, 180, AngleUnit.DEGREES);
        pivot = new SimpleServo(hardwareMap, Constants.INTAKE_PIVOT_CONFIG, 0, 220, AngleUnit.DEGREES);
        leftVacuum = new CRServo(hardwareMap, Constants.INTAKE_LEFT_VACUUM_CONFIG);
        rightVacuum = new CRServo(hardwareMap, Constants.INTAKE_RIGHT_VACUUM_CONFIG);
        colorSensor = hardwareMap.get(ColorSensor.class, "intakeColor");
        beamBrake = hardwareMap.get(DigitalChannel.class, Constants.INTAKE_BEAM_BREAK_CONFIG);

        isGamepad = true;
        this.gamepad = gamepad;

        this.colorToIgnore = colorToIgnore;

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
        else if (vacuumState == vacuum.SPEWING) {
            runVacuumEject();
        }
        else {
            horizontalIn();
            if (!beamBrake.getState()) {
                leftVacuum.set(.1);
                rightVacuum.set(-.1);
            }
            else {
                runVacuumStop();
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
        horizontalToPos(Constants.EXTENSION_IN_ANGLE + trim);
        intakeState = state.RESTING;
    }

    public void horizontalTransfer() {
        horizontalToPos(Constants.EXTENSION_IN_ANGLE - 10);
    }

    public void horizontalOut() {
        horizontalToPos(Constants.EXTENSION_OUT_ANGLE + trim);
        intakeState = state.INTAKING;
    }

    public void horizontalOut(double t) {
        horizontalToPos(Constants.EXTENSION_OUT_ANGLE + t);
        intakeState = state.INTAKING;
    }

    public double getPivotPos() {
        return pivot.getAngle();
    }

    public void pivotDown() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_DOWN_ANGLE);
    }

    public void pivotEject() {
        pivotToPos(Constants.INTAKE_PIVOT_TO_EJECT_ANGLE);
    }

    public void pivotBasket() {
        if (intakeState == state.RESTING) {
            pivotToPos(Constants.INTAKE_PIVOT_TO_TRANSITION_ANGLE);
        }
    }

    public void pivotHome() {
        if ((vacuumState == vacuum.SPEWING) && (intakeState == state.INTAKING)) {
            return;
        }
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
        if (state == OldIntake.state.INTAKING) {
            hPose = Constants.EXTENSION_OUT_ANGLE + trim;
            pPose = Constants.INTAKE_PIVOT_TO_DOWN_ANGLE;
        }
        else if (state == OldIntake.state.RESTING){
            hPose = Constants.EXTENSION_IN_ANGLE;
            pPose = Constants.INTAKE_PIVOT_TO_REST_ANGLE;
        }
        else if (state == OldIntake.state.TRANSFERRING){
            hPose = Constants.EXTENSION_IN_ANGLE;
            pPose = Constants.INTAKE_PIVOT_TO_TRANSITION_ANGLE;
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

    public void extendSlightly() {
        horizontalToPos(Constants.EXTENSION_IN_ANGLE + 40);
    }
}
