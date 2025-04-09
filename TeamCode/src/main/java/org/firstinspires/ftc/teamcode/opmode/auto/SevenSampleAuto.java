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
import org.firstinspires.ftc.teamcode.commands.IntakingCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
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
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
//        kicker = new Kicker(hardwareMap, telemetry);

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
                new DepositorCommand(depositor, Depositor.state.CLAWCLOSE).withTimeout(10),
                new DepositorCommand(depositor, Depositor.state.PRIME_BASKET).withTimeout(1000),
                new WaitCommand(200),
                new IntakePositionCommand(intake, Intake.state.RESTING)
                ));


        schedule(new SequentialCommandGroup(
                new TrajectoryGotoCommand(drive, startToBasket),
                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                new DepositorCommand(depositor, Depositor.state.BUCKET).withTimeout(1000),
                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(200),

                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                new DepositorCommand(depositor, Depositor.state.PRIME1).withTimeout(1000),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 50),
                new ParallelCommandGroup(
                        new TrajectoryGotoCommand(drive, basketToFirstSample)
                ),
                new ParallelRaceGroup(
                        new IntakeHasSampleCommand(intake),
                        new RawDrivetrainCommand(drive, 0.25, 0, 0).withTimeout(2000)
                ),
                new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(700)

//                new StrafeToPositionCommand(SampleActions.basketPos, drive),
//
//                new TrajectoryGotoCommand(drive, basketToSecondSample),
//                new StrafeToPositionCommand(SampleActions.basketPos, drive),
//
//                new TrajectoryGotoCommand(drive, basketToThirdSample),
//                new StrafeToPositionCommand(SampleActions.basketPos, drive)
        ));
    }
}