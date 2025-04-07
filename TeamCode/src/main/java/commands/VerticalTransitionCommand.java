package commands;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import subsystems.Depositor;
import subsystems.Elevator;
import subsystems.Intake;

/*
new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),
                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                new WaitCommand(100),
                new DepositorCommand(depositor, Depositor.state.CLAWCLOSE).withTimeout(300),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(100),
                new DepositorCommand(depositor, Depositor.state.HOME)
*/
public class VerticalTransitionCommand extends SequentialCommandGroup {
    private final Depositor depositor;
    private final Intake intake;
    private final Elevator elevator;

    public VerticalTransitionCommand(Depositor depositor, Intake intake, Elevator elevator) {
        addRequirements(depositor, intake, elevator);
        this.depositor = depositor;
        this.intake = intake;
        this.elevator = elevator;

        this.addCommands(
                new SequentialCommandGroup(
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(10),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(10),
                                new IntakePositionCommand(intake, Intake.state.VERTICAL_TRANSITION).withTimeout(10)
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new DepositorCommand(depositor, Depositor.state.VERTICAL_TRANSITION).withTimeout(100),
                                        new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(100),
                                        new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(200)
                                ),
                                new IntakePositionCommand(intake, Intake.state.VERTICAL_TRANSFERRING_AND_BARFING).withTimeout(100)
                        ),
                        new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(100)
                )
        );
    }

}