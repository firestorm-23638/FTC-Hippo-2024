package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.actions.SpecimenActions;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class FiveSpecimenAuto extends CommandOpMode {
    private Drivetrain drive;
    private Elevator elevator;
    private Intake intake;
    private Depositor depositor;

    @Override
    public void initialize() {
        Constants.IS_RED = false;

        Pose2d home = SpecimenActions.startingPos;

        depositor = new Depositor(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, home, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);

//        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToSpecimen = SpecimenActions.startToScore(drive);

        Action pushAllSamples = SpecimenActions.pushAllSamples(drive);
        Action scoreSecondSpecimen = SpecimenActions.scoreSecondSpecimen(drive);
        Action secondSpecimenToPickup = SpecimenActions.secondSpecimenToPickup(drive);
        Action scoreThirdSpecimen = SpecimenActions.scoreThirdSpecimen(drive);
        Action thirdSpecimenToPickup = SpecimenActions.thirdSpecimenToPickup(drive);
        Action scoreFourthSpecimen = SpecimenActions.scoreFourthSpecimen(drive);
        Action fourthSpecimenToPickup = SpecimenActions.fourthSpecimenToPickup(drive);

        depositor.toPosition(Depositor.state.CLAWTIGHTEN);

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new SequentialCommandGroup(
//                new InstantCommand(() -> drive.setCurrentPose(home)),
//                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(10),
//                new DepositorCommand(depositor, Depositor.state.SCORE_SPECIMEN).withTimeout(100),
                new IntakePositionCommand(intake, Intake.state.RESTING, 10)

//                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(100)
//                new WaitCommand(200),
//                new IntakePositionCommand(intake, Intake.state.RESTING)
        ));


        schedule(new SequentialCommandGroup(
                    new ParallelCommandGroup(
                            new DepositorCommand(depositor, Depositor.state.SCORE_SPECIMEN).withTimeout(400),
                            new TrajectoryGotoCommand(drive, startToSpecimen)
                    ),
                    // score first
                    new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                    new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),
                    new ParallelCommandGroup(
                            new TrajectoryGotoCommand(drive, pushAllSamples),
                            new DepositorCommand(depositor, Depositor.state.INTAKE_SPECIMEN).withTimeout(400),
                            new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                    ),
                    new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(150),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new DepositorCommand(depositor, Depositor.state.PRIME_SPECIMEN).withTimeout(400),
                                    new DepositorCommand(depositor, Depositor.state.PLACE_SPECIMEN).withTimeout(500)
                            ),
                            new SequentialCommandGroup(
                                    new WaitCommand(200),
                                    new TrajectoryGotoCommand(drive, scoreSecondSpecimen)
                            )
                    ),
                    // score second
                    new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                    new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),

                    new ParallelCommandGroup(
                            new TrajectoryGotoCommand(drive, secondSpecimenToPickup),
                            new DepositorCommand(depositor, Depositor.state.INTAKE_SPECIMEN).withTimeout(400),
                            new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                    ),
                    new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(150),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new DepositorCommand(depositor, Depositor.state.PRIME_SPECIMEN).withTimeout(400),
                                    new DepositorCommand(depositor, Depositor.state.PLACE_SPECIMEN).withTimeout(500)
                            ),
                            new SequentialCommandGroup(
                                    new WaitCommand(200),
                                    new TrajectoryGotoCommand(drive, scoreThirdSpecimen)
                            )
                    ),
                    // score third
                    new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                    new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),
                    new ParallelCommandGroup(
                            new TrajectoryGotoCommand(drive, thirdSpecimenToPickup),
                            new DepositorCommand(depositor, Depositor.state.INTAKE_SPECIMEN).withTimeout(400),
                            new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                    ),
                    new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(150),
                    new ParallelCommandGroup(
                            new SequentialCommandGroup(
                                    new DepositorCommand(depositor, Depositor.state.PRIME_SPECIMEN).withTimeout(400),
                                    new DepositorCommand(depositor, Depositor.state.PLACE_SPECIMEN).withTimeout(500)
                            ),
                            new SequentialCommandGroup(
                                    new WaitCommand(200),
                                    new TrajectoryGotoCommand(drive, scoreFourthSpecimen)
                            )
                    ),
                    // score fourth
                    new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                    new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100),
                    new ParallelCommandGroup(
                            new TrajectoryGotoCommand(drive, fourthSpecimenToPickup),
                            new DepositorCommand(depositor, Depositor.state.INTAKE_SPECIMEN).withTimeout(400),
                            new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                    )
//                    new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
//                    new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100)
                )
        );
    }
}