package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.StrafeToNearestBlueGroupLimelight;
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeHasSampleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.commands.TransitionCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToNearestYellowSampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous
public class TESTAuto0_6Limelight extends CommandOpMode {
    private Drivetrain drive;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;
    private Depositor depositor;

    @Override
    public void initialize() {
        Constants.isRed = false;

        Pose2d home = new Pose2d(-38,-60.5, Math.toRadians(90));

        depositor = new Depositor(hardwareMap, telemetry);
        drive = new Drivetrain(hardwareMap, home, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        limelight = new Limelight(hardwareMap, telemetry);
        kicker = new Kicker(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToBasket = BlueActions.startToBasket(drive, home);

        Action basketToFirstSample = BlueActions.basketToFirstSample(drive);
        Action basketToSecondSample = BlueActions.basketToSecondSample(drive);
        Action basketToThirdSample = BlueActions.basketToThirdSample(drive);
        Action basketToSubmersible = BlueActions.basketToSubmersible2(drive);
        Action submersibleToBasket = BlueActions.submersibleToBasket(drive);
        Action basketToSubmersible2 = BlueActions.basketToSubmersible(drive);
        Action submersible2ToBasket = BlueActions.submersible2ToBasket(drive);

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new InstantCommand(() -> {
            depositor.clawClose();
            depositor.toTransition();
            elevator.currentStage = Elevator.basketState.MIDDLE_BASKET;
            intake.currentState = Intake.state.RESTING;
        }));

        Command intakeRightmostSample = new SequentialCommandGroup(
                new TurnToNearestYellowSampleCommand(limelight, drive).withTimeout(1000),
                // First Sample
                new IntakePositionCommand(intake, Intake.state.INTAKING, 100, 40),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .3, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 500)
        );

        Command transferAndRaise = new SequentialCommandGroup(
                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                new WaitCommand(100),
                new DepositorCommand(depositor, Depositor.state.CLAWCLOSE).withTimeout(300),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(400),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                        new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                )
        );


        schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new TrajectoryGotoCommand(startToBasket, drive),
                                new SequentialCommandGroup(
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                        new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                                )
                        ),
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                        //new WaitCommand(20000),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.TRANSITIONING).withTimeout(500),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET)
                                ),
                                //FIRST SAMPLE
                                new SequentialCommandGroup(
                                        new TrajectoryGotoCommand(basketToFirstSample, drive),
                                        //new IntakePositionCommand(intake, Intake.state.INTAKING, 500, 15),
                                        //new TurnToNearestYellowSampleCommand(limelight, drive).withTimeout(1000),
                                        new SequentialCommandGroup(
                                                new ParallelCommandGroup(
                                                        new ParallelRaceGroup(
                                                                new StrafeToNearestBlueGroupLimelight(limelight, drive, telemetry, 0.25).withTimeout(1500),
                                                                new SequentialCommandGroup(
                                                                        new WaitCommand(300),
                                                                        new SlideUntilHasPieceCommand(intake, Intake.color.YELLOW)
                                                                )
                                                        )
                                                )
                                        ),
                                        new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50)
                                )
                        ),
                        new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                        new ParallelCommandGroup(
                                new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new TransitionCommand(depositor, intake, elevator),
                                        new SequentialCommandGroup(
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                                        )
                                )
                        ),
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.TRANSITIONING).withTimeout(500),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET)
                                ),
                                //SECOND SAMPLE
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 30),
                                                new TrajectoryGotoCommand(basketToSecondSample, drive)
                                        ),
                                        //new TurnToNearestYellowSampleCommand(limelight, drive).withTimeout(1000),
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 100, 50),
                                        new ParallelRaceGroup(
                                                new RawDrivetrainCommand(drive, .25, 0, 0).withTimeout(1200),
                                                new IntakeHasSampleCommand(intake)
                                        ),
                                        new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50)
                                )
                        ),
                        new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                        new ParallelCommandGroup(
                                new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new TransitionCommand(depositor, intake, elevator),
                                        new SequentialCommandGroup(
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                                        )
                                )
                        ),
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.TRANSITIONING).withTimeout(500),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ParallelCommandGroup(
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET)
                                        )
                                ),
                                // Third Sample
                                new SequentialCommandGroup(
                                        new ParallelCommandGroup(
                                                new TrajectoryGotoCommand(basketToThirdSample, drive),
                                                new IntakePositionCommand(intake, Intake.state.INTAKING, 500, 15)
                                        ),
                                        //new TurnToNearestYellowSampleCommand(limelight, drive).withTimeout(1000),
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 100, 20),
                                        new ParallelRaceGroup(
                                                new RawDrivetrainCommand(drive, .15, 0, 0).withTimeout(1500),
                                                new IntakeHasSampleCommand(intake)
                                        ),
                                        new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50)
                                )
                        ),
                        new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                        new ParallelCommandGroup(
                                new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new TransitionCommand(depositor, intake, elevator),
                                        new SequentialCommandGroup(
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                                        )
                                )
                        ),
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.TRANSITIONING).withTimeout(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET),
                                // Fourth Sample
                                new TrajectoryGotoCommand(basketToSubmersible, drive)
                        ),
                        new KickerCommand(kicker, 500, Kicker.state.OPEN),
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 0),
                                new SlideUntilHasPieceCommand(intake, Intake.color.BLUE)
                        ),
                        new ParallelCommandGroup(
                                new KickerCommand(kicker, 500, Kicker.state.CLOSE),
                                new SequentialCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.RESTING, 700),
                                        new TransitionCommand(depositor, intake, elevator),
                                        new SequentialCommandGroup(
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                                        )
                                ),
                                new TrajectoryGotoCommand(submersibleToBasket, drive)
                        ),
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.TRANSITIONING).withTimeout(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET),
                                // Fifth Sample
                                new TrajectoryGotoCommand(basketToSubmersible2, drive)
                        ),
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 0),
                                new SlideUntilHasPieceCommand(intake, Intake.color.BLUE)
                        ),
                        new ParallelCommandGroup(
                                new KickerCommand(kicker, 500, Kicker.state.CLOSE),
                                new SequentialCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.RESTING, 700),
                                        new TransitionCommand(depositor, intake, elevator),
                                        new SequentialCommandGroup(
                                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                                                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(500)
                                        )
                                ),
                                new TrajectoryGotoCommand(submersible2ToBasket, drive)
                        ),
                        new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(300),
                        new ParallelCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.HOME),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET)
                                )
                        )
                )
        );
    }
}
