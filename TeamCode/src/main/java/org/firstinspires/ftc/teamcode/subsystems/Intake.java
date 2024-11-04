package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
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

    public enum state {
        INTAKING,
        RESTING,
        TRANSFERRING
    }

    public enum vacuum {
        SUCKING,
        SPEWING,
        STATIONING
    }

    public state intakeState = state.RESTING;
    public vacuum vacuumState = vacuum.STATIONING;

    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        horizontal = new SimpleServo(hardwareMap, Constants.intakeHorizontalConfig, 0, 180, AngleUnit.DEGREES);
        pivot = new SimpleServo(hardwareMap, Constants.intakePivotConfig, 0, 220, AngleUnit.DEGREES);
        intake = new CRServo(hardwareMap, Constants.intakeVacuumConfig);

        this.telemetry = telemetry;
    };

    @Override
    public void periodic() {
        telemetry.addData("Horizontal Position", horizontal.getAngle());
        telemetry.addData("Pivot Position", pivot.getAngle());

        if (vacuumState == vacuum.SUCKING) {
            intake.set(Constants.intakeVacuumSpeed);
        }
        else if (vacuumState == vacuum.SPEWING) {
            intake.set(Constants.intakeEjectSpeed);
        }
        else {
            intake.set(0);
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
        pivotToPos(Constants.intakePivotToBasket+20);
    }

    public void vacuumEject() {
        vacuumState = vacuum.SPEWING;
    }

    public void vacuumRun() {
        vacuumState = vacuum.SUCKING;
    }

    public void vacuumStop() {
        vacuumState = vacuum.STATIONING;
    }
}
