package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    private final ServoEx horizontal;
    private final ServoEx pivot;
    private final CRServo intake;
    private final Telemetry telemetry;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        horizontal = new SimpleServo(hardwareMap, Constants.intakeHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        pivot = new SimpleServo(hardwareMap, Constants.intakePivotConfig, 0, 180, AngleUnit.DEGREES);
        intake = new CRServo(hardwareMap, Constants.intakeVacuumConfig);

        this.telemetry = telemetry;
    };

    private void horizontalToPos(double targetPos) {
        horizontal.turnToAngle(targetPos);
    }

    private void pivotToPos(double targetPos) {
        pivot.turnToAngle(targetPos);
    }

    public boolean horizontalIn() {
        horizontalToPos(Constants.intakeHorizontalToHomePose);
        return horizontal.getAngle() == Constants.intakeHorizontalToHomePose;
    }

    public boolean horizontalOut() {
        horizontalToPos(Constants.intakeHorizontalToIntakePose);
        return horizontal.getAngle() == Constants.intakeHorizontalToIntakePose;
    }

    public boolean pivotDown() {
        pivotToPos(Constants.intakePivotToDown);
        return pivot.getAngle() == Constants.intakePivotToDown;
    }

    public boolean pivotUp() {
        pivotToPos(Constants.intakePivotToUp);
        return pivot.getAngle() == Constants.intakePivotToUp;
    }

    public void vacuumRun() {
        intake.set(Constants.intakeVacuumSpeed);
    }

    public void vacuumStop() {
        intake.set(0);
    }

    public void horizontalOut(boolean controlInput) {
        if (controlInput) {
            if (horizontalOut()) {
                pivotDown();
                vacuumRun();
            }
            else {
                pivotUp();
                vacuumStop();
            }
        }
        else {
            vacuumStop();
            if (pivotUp()) {
                horizontalIn();
            }

        }
    }

    // Tests

    public void printHorizontalPose() {
        telemetry.addData("Horizontal Position", horizontal.getAngle());
    }

    public void printPivotPose() {
        telemetry.addData("Pivot Position", pivot.getAngle());
    }
}
