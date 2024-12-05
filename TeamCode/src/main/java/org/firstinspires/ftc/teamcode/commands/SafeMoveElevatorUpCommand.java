package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class SafeMoveElevatorUpCommand extends SafeMoveElevatorCommand {

    public SafeMoveElevatorUpCommand(Elevator elevator, Intake intake) {
        super(elevator, null,intake);
        Elevator.BasketState desiredState = elevator.increaseStage();
        this.desiredState = desiredState;
    }
}
