package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

public class ElevatorPositionCommand extends CommandBase {
    private final Elevator elevator;
    private Elevator.basketState state;
    private boolean isFinished = false;
    private double trim = 0;

    public ElevatorPositionCommand(Elevator elevator, Elevator.basketState state, double trim) {
        addRequirements(elevator);
        this.elevator = elevator;
        this.state = state;
        this.trim = trim;
    }

    public ElevatorPositionCommand(Elevator elevator, Elevator.basketState state) {
        this(elevator, state, 0);
    }

    @Override
    public void initialize() {
        elevator.isZeroed = false;
    }

    @Override
    public void execute() {
        elevator.currentStage = state;
        isFinished = elevator.isAtPos();
        elevator.setTrim(trim);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
