package org.firstinspires.ftc.teamcode.opmode.auto.blue;

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
import org.firstinspires.ftc.teamcode.commands.BasketPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeHasSampleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous
public class BLUE0Specimen4Sample extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;

    @Override
    public void initialize() {
        Constants.isRed = false;

        Pose2d home = new Pose2d(-38,-60.5, Math.toRadians(90));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        limelight = new Limelight(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToBasket = BlueActions.startToBasket(drive, home);
        Action basketToFirstSample = BlueActions.basketToFirstSample(drive);
        Action basketToSecondSample = BlueActions.basketToSecondSample(drive);
        Action basketToThirdSample = BlueActions.basketToThirdSample(drive);

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new InstantCommand(() -> {
            basket.toHome();
            elevator.currentStage = Elevator.basketState.HOME;
            intake.pivotHome();
            intake.horizontalIn();
        }));

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new TrajectoryGotoCommand(startToBasket, drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(700),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),

                        new TrajectoryGotoCommand(basketToFirstSample, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING, 1000)
                ),
                //new TurnToNearestSampleCommand(limelight, drive),
                // First Sample
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .3, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 700),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 700),
                new ParallelCommandGroup(
                        new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET),
                        new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(700),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),

                        // Second Sample
                        new TrajectoryGotoCommand(basketToSecondSample, drive)
                ),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 1000),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .3, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 700),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                new ParallelCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.RESTING, 200),
                        new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(700),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),

                        // Third Sample
                        new TrajectoryGotoCommand(basketToThirdSample, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING, 1000)
                ),

                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive,.2, 0, 0).withTimeout(2000),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, -.2, 0, 0).withTimeout(1000),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 700),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                new ParallelCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.RESTING, 200),
                        new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(500),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new SequentialCommandGroup(
                                new RawDrivetrainCommand(drive, .3, 0, 0).withTimeout(400),
                                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        )
                ),
                new InstantCommand(() -> Constants.pose = drive.getCurrentPose())
                )
        );
        //.andThen(new TrajectoryGotoCommand(basketToObservation, drive)));

    }
}