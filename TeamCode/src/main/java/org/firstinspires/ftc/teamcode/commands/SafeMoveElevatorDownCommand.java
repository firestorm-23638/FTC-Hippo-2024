package org.firstinspires.ftc.teamcode.commands;

import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class SafeMoveElevatorDownCommand extends SafeMoveElevatorCommand {

    public SafeMoveElevatorDownCommand(Elevator elevator, Intake intake) {
        super(elevator, null,intake);
        Elevator.BasketState desiredState = elevator.decreaseStage();
        this.desiredState = desiredState;
    }
}
