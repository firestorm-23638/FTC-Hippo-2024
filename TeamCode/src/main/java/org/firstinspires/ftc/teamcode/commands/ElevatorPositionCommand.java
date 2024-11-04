package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorPositionCommand extends CommandBase {
    private final Elevator elevator;
    private Elevator.basketState state;
    private boolean isFinished = false;

    public ElevatorPositionCommand(Elevator elevator, Elevator.basketState state) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.state = state;
    }

    @Override
    public void execute() {
        elevator.currentStage = state;
        isFinished = elevator.isAtPos();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
