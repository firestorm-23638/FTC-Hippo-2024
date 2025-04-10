package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeHasSampleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalTransitionCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.actions.SampleActions;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous
public class SixSampleAuto extends CommandOpMode {
    private Drivetrain drive;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;
    private Depositor depositor;

    @Override
    public void initialize() {
        Constants.IS_RED = false;

        Pose2d home = new Pose2d(-38,-60.5, Math.toRadians(90));

        depositor = new Depositor(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, home, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        kicker = new Kicker(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToBasket = SampleActions.startToBasket(drive, home);

        Action basketToFirstSample = SampleActions.basketToFirstSample(drive);
        Action basketToSecondSample = SampleActions.basketToSecondSample(drive);
        Action basketToThirdSample = SampleActions.basketToThirdSample(drive);
        Action basketToSubmersible = SampleActions.basketToSubmersible2(drive);
        Action submersibleToBasket = SampleActions.submersibleToBasket(drive);
        Action basketToSubmersible2 = SampleActions.basketToSubmersible(drive);
        Action submersible2ToBasket = SampleActions.submersible2ToBasket(drive);

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> drive.setCurrentPose(home)),
                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(10),
                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(100)

//                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(100)
//                new WaitCommand(200),
//                new IntakePositionCommand(intake, Intake.state.RESTING)
                ));


        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new TrajectoryGotoCommand(drive, startToBasket),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(200)
                        )
                ),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(200),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(300),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(200)
                        ),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 44),
                                        new TrajectoryGotoCommand(drive, basketToFirstSample)
                                ),
                                new ParallelRaceGroup(
                                        new IntakeHasSampleCommand(intake),
                                        new RawDrivetrainCommand(drive, 0.3, 0, 0).withTimeout(2000)
                                )
                        )
                ),

                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new VerticalTransitionCommand(depositor, intake, elevator),
                                new SequentialCommandGroup(
                                        new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(350),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(300)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new StrafeToPositionCommand(SampleActions.basketPos, drive)
                        )
                ),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(200),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(300),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(200)
                        ),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 40),
                                        new TrajectoryGotoCommand(drive, basketToSecondSample)
                                ),
                                new ParallelRaceGroup(
                                        new IntakeHasSampleCommand(intake),
                                        new RawDrivetrainCommand(drive, 0.3, 0, 0).withTimeout(2000)
                                )
                        )
                ),

                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new VerticalTransitionCommand(depositor, intake, elevator),
                                new SequentialCommandGroup(
                                        new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(350),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(300)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new StrafeToPositionCommand(SampleActions.basketPos, drive)
                        )
                ),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(200),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(300),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(200)
                        ),
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 10),
                                        new TrajectoryGotoCommand(drive, basketToThirdSample)
                                ),
                                new ParallelRaceGroup(
                                        new IntakeHasSampleCommand(intake),
                                        new RawDrivetrainCommand(drive, 0.25, 0, 0).withTimeout(2000)
                                )
                        )
                ),

                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new VerticalTransitionCommand(depositor, intake, elevator),
                                new SequentialCommandGroup(
                                        new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(350),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(300)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new StrafeToPositionCommand(SampleActions.basketPos, drive)
                        )
                ),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(200),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(200),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),
                        new TrajectoryGotoCommand(drive, basketToSubmersible)
                ),
                new KickerCommand(kicker, Kicker.state.OPEN).withTimeout(150),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 300, 0),
                new SlideUntilHasPieceCommand(intake, 0),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new VerticalTransitionCommand(depositor, intake, elevator),
                                new ParallelCommandGroup(
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                        new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(350)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(200)
                        ),
                        new TrajectoryGotoCommand(drive, submersibleToBasket),
                        new KickerCommand(kicker, Kicker.state.CLOSE).withTimeout(300)
                ),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(200),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(200),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),
                        new TrajectoryGotoCommand(drive, basketToSubmersible2)
                ),
                new KickerCommand(kicker, Kicker.state.OPEN).withTimeout(150),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 300, 0),
                new SlideUntilHasPieceCommand(intake, 0),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new VerticalTransitionCommand(depositor, intake, elevator),
                                new ParallelCommandGroup(
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                        new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(350)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(200)
                        ),
                        new TrajectoryGotoCommand(drive, submersible2ToBasket),
                        new KickerCommand(kicker, Kicker.state.CLOSE).withTimeout(300)
                )


//                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
//                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(1000),
//                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 30),
//                new ParallelCommandGroup(
//                        new TrajectoryGotoCommand(drive, basketToFirstSample)
//                ),
//                new ParallelRaceGroup(
//                        new IntakeHasSampleCommand(intake),
//                        new RawDrivetrainCommand(drive, 0.25, 0, 0).withTimeout(2000)
//                ),
                //new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(700)

//                new StrafeToPositionCommand(SampleActions.basketPos, drive),
////
//                new TrajectoryGotoCommand(drive, basketToSecondSample),
//                new StrafeToPositionCommand(SampleActions.basketPos, drive),
////
//                new TrajectoryGotoCommand(drive, basketToThirdSample),
//                new StrafeToPositionCommand(SampleActions.basketPos, drive)
        ));
    }
}