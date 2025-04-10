package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

/*
new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),
                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                new WaitCommand(100),
                new DepositorCommand(depositor, Depositor.state.CLAWCLOSE).withTimeout(300),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(100),
                new DepositorCommand(depositor, Depositor.state.HOME)
*/
public class SpeedyTransitionCommand extends SequentialCommandGroup {
    private final Depositor depositor;
    private final Intake intake;
    private final Elevator elevator;

    public SpeedyTransitionCommand(Depositor depositor, Intake intake, Elevator elevator) {
        addRequirements(depositor, intake, elevator);
        this.depositor = depositor;
        this.intake = intake;
        this.elevator = elevator;

        this.addCommands(
                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(150),
                new IntakePositionCommand(intake, Intake.state.VERTICAL_TRANSFERRING_AND_BARFING).withTimeout(100)
        );
    }

}