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
import org.firstinspires.ftc.teamcode.commands.HorizontalTransitionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeHasSampleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.commands.VerticalTransitionCommand;
import org.firstinspires.ftc.teamcode.opmode.auto.actions.SampleActions;
import org.firstinspires.ftc.teamcode.opmode.auto.actions.SpecimenActions;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

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
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToSpecimen = SpecimenActions.startToScore(drive);

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
                new DepositorCommand(depositor, Depositor.state.SCORE_SPECIMEN).withTimeout(100),
                new IntakePositionCommand(intake, Intake.state.RESTING, 10)

//                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(100)
//                new WaitCommand(200),
//                new IntakePositionCommand(intake, Intake.state.RESTING)
        ));


        schedule(new SequentialCommandGroup(
                    new TrajectoryGotoCommand(drive, startToSpecimen),
                    new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                    new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(100)
                )
        );
    }
}