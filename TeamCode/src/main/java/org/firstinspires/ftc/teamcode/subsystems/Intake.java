package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;

public class Intake extends SubsystemBase {
    private final ServoEx horizontal;
    private final ServoEx pivot;
    private final CRServo intake;
    private final Telemetry telemetry;

    private int intakePivotPos = 0;
    private double pivotPoses[] = {Constants.intakePivotToBasket, Constants.intakePivotToUp, Constants.intakePivotToDown};
    private boolean intakeSpinning = false;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        horizontal = new SimpleServo(hardwareMap, Constants.intakeHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        pivot = new SimpleServo(hardwareMap, Constants.intakePivotConfig, 0, 210, AngleUnit.DEGREES);
        intake = new CRServo(hardwareMap, Constants.intakeVacuumConfig);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        telemetry.addData("Horizontal Position", horizontal.getAngle());
        telemetry.addData("Pivot Position", pivot.getAngle());

        pivotToPos(pivotPoses[intakePivotPos]);

        if (intakePivotPos == 2) {
            vacuumRun();
        }
        else {
            vacuumStop();
        }

    }

    private void horizontalToPos(double targetPos) {
        horizontal.turnToAngle(targetPos);
    }

    private void pivotToPos(double targetPos) {
        pivot.turnToAngle(targetPos);
    }

    public void horizontalIn() {
        horizontalToPos(Constants.intakeHorizontalToHomePose);
    }

    public void horizontalOut() {
        horizontalToPos(Constants.intakeHorizontalToIntakePose);
    }

    public void pivotDown() {
        intakePivotPos = 2;
    }

    public void pivotUp() {
        intakePivotPos = 1;
    }

    public void pivotBasket() {
        intakePivotPos = 0;
    }

    public void vacuumRun() {
        intake.set(Constants.intakeVacuumSpeed);
    }

    public void vacuumStop() {
        intake.set(0);
    }

    public void horizontalOut(boolean controlInput) {
        /*if (controlInput) {
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

        }*/
    }
}
