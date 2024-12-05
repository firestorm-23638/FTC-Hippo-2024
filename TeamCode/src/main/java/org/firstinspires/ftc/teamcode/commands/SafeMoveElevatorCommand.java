package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class SafeMoveElevatorCommand extends CommandBase {
    protected Elevator elevator;
    protected Intake intake;
    protected Elevator.BasketState desiredState;
    protected Boolean isFinished = false;

    public SafeMoveElevatorCommand(Elevator elevator, Elevator.BasketState state, Intake intake) {
        this.elevator = elevator;
        this.desiredState = state;
        this.intake = intake;
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        isFinished = elevator.isAtPos();

        // don't do anything if we are at the home state
        if (desiredState == Elevator.BasketState.HOME
                && elevator.getCurrentStage() == Elevator.BasketState.HOME) {
            isFinished = true;
            return;
        }
        // A check should be added in case the intake is already out.
        boolean moveIntake = (desiredState == Elevator.BasketState.HOME
                || elevator.getCurrentStage() == Elevator.BasketState.HOME);

        if (moveIntake) {
            intake.extendSlightly();
        }

        elevator.setDesiredState(desiredState);
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
