package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.ConditionalCommand;
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
import org.firstinspires.ftc.teamcode.commands.SpeedyTransitionCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.commands.TurnAndForwardCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalTransitionCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.actions.SampleActions;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous
public class SevenSampleAuto extends CommandOpMode {
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
        intake = new Intake(hardwareMap, telemetry);
        kicker = new Kicker(hardwareMap, telemetry);

        intake.setTargetColor(Intake.color.BLUE_YELLOW);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToBasket = SampleActions.startToAngledBasket(drive, home);

        Action basketToFirstSample = SampleActions.basketToFirstSample(drive);
        Action basketToSecondSample = SampleActions.basketToSecondSample(drive);
        Action basketToThirdSample = SampleActions.basketToThirdSample(drive);
        Action basketToSubmersible = SampleActions.basketToSubmersible2(drive);
        Action submersibleToBasket = SampleActions.submersibleToBasket(drive);
        Action basketToSubmersible2 = SampleActions.basketToSubmersible(drive);
        Action submersible2ToBasket = SampleActions.submersible2ToBasket(drive);
        Action submersibleToSubmersible2 = SampleActions.submersibleToSubmersible2(drive);

        Action basketToSubmersible3 = SampleActions.too(drive);
        Action submersible3ToBasket = SampleActions.from(drive);

        depositor.toPosition(Depositor.state.CLAWTIGHTEN);

        final long primeBasketMs = 200;
        final long bucketMs = 200;
        final long clawMs = 250;

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new SequentialCommandGroup(
                new InstantCommand(() -> drive.setCurrentPose(home)),
                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(10),
                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(100)
                ));


        schedule(new SequentialCommandGroup(
                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN).withTimeout(50),
                new ParallelCommandGroup(
                        new TrajectoryGotoCommand(drive, startToBasket),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(bucketMs)
                        )
                ),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 44).withTimeout(10),
                // first sample
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(clawMs),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                                new DepositorCommand(depositor, Depositor.state.VERTICAL_TRANSITION).withTimeout(200)
                        ),
                        new SequentialCommandGroup(
                                new ParallelRaceGroup(
                                        new IntakeHasSampleCommand(intake),
                                        new RawDrivetrainCommand(drive, 0.25, 0, 0).withTimeout(2000)
                                )
                        )
                ),
//
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),
//
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new SpeedyTransitionCommand(depositor, intake, elevator),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs)
                                        ),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(bucketMs)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(0),
                                new StrafeToPositionCommand(SampleActions.basketPos, drive)
                        )
                ),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 40).withTimeout(10),
                // second sample
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(clawMs),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                                new DepositorCommand(depositor, Depositor.state.VERTICAL_TRANSITION).withTimeout(200)
                        ),
                        new SequentialCommandGroup(
                                new TurnAndForwardCommand(drive, 0, 93).withTimeout(600),
                                new ParallelRaceGroup(
                                        new IntakeHasSampleCommand(intake),
                                        new TurnAndForwardCommand(drive, 0.25, 96).withTimeout(2000)
                                )
                        )
                ),

                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 550),
                                new SpeedyTransitionCommand(depositor, intake, elevator),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs)
                                        ),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(bucketMs)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new StrafeToPositionCommand(SampleActions.basketPos, drive)
                        )
                ),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 10, 43).withTimeout(10),
                //third sample
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(clawMs),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                                new DepositorCommand(depositor, Depositor.state.VERTICAL_TRANSITION).withTimeout(200)
                        ),
                        new SequentialCommandGroup(
                                new TurnAndForwardCommand(drive, 0, 100).withTimeout(500),
                                new ParallelRaceGroup(
                                        new IntakeHasSampleCommand(intake),
                                        new TurnAndForwardCommand(drive, 0.25, 115)
                                )
                        )
                ),

                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 550),
                                new SpeedyTransitionCommand(depositor, intake, elevator),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs)
                                        ),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(bucketMs)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new StrafeToPositionCommand(SampleActions.basketPos, drive)
                        )
                ),
                // fourth sample
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(clawMs),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.VERTICAL_TRANSITION).withTimeout(200),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),
                        new TrajectoryGotoCommand(drive, basketToSubmersible).withTimeout(2200)
                ),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(20),
                new KickerCommand(kicker, Kicker.state.OPEN).withTimeout(100),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 200, 0),
                new SlideUntilHasPieceCommand(intake, 0),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                                                new SpeedyTransitionCommand(depositor, intake, elevator),
                                                new ParallelCommandGroup(
                                                        new SequentialCommandGroup(
                                                                new WaitCommand(200),
                                                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs)
                                                        ),
                                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                                ),
                                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(bucketMs)
                                        ),
                                        new TrajectoryGotoCommand(drive, submersibleToBasket),
                                        new KickerCommand(kicker, Kicker.state.CLOSE).withTimeout(300)
                                ),
                                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(clawMs),

                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new DepositorCommand(depositor, Depositor.state.VERTICAL_TRANSITION).withTimeout(200),
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                        ),
                                        new TrajectoryGotoCommand(drive, basketToSubmersible2).withTimeout(2200)
                                ),
                                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10)
                        ),
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.BARFING, 200, 0),
                                new KickerCommand(kicker, Kicker.state.OPEN).withTimeout(50),
                                new TrajectoryGotoCommand(drive, submersibleToSubmersible2)
                        ),
                        () -> (!intake.intakeEmpty()) && (!intake.hasCorrectColor())
                ),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 200, 0),
                new SlideUntilHasPieceCommand(intake, 0),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 600),
                                new SpeedyTransitionCommand(depositor, intake, elevator),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new WaitCommand(200),
                                                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(primeBasketMs)
                                        ),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(bucketMs)
                        ),
                        new TrajectoryGotoCommand(drive, submersible3ToBasket),
                        new KickerCommand(kicker, Kicker.state.CLOSE).withTimeout(300)
                ),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(clawMs)


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