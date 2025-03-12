package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.RumbleManager;

/*
new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),
                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                new WaitCommand(100),
                new DepositorCommand(depositor, Depositor.state.CLAWCLOSE).withTimeout(300),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(100),
                new DepositorCommand(depositor, Depositor.state.HOME)
*/
public class TransitionCommand extends SequentialCommandGroup {
    private final Depositor depositor;
    private final Intake intake;
    private final Elevator elevator;

    public TransitionCommand(Depositor depositor, Intake intake, Elevator elevator) {
        addRequirements(depositor, intake, elevator);
        this.depositor = depositor;
        this.intake = intake;
        this.elevator = elevator;

        this.addCommands(
                new IntakePositionCommand(intake, Intake.state.RESTING, 100),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(50),
                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                new DepositorCommand(depositor, Depositor.state.CLAWCLOSE).withTimeout(100),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(300)
        );
    }

}