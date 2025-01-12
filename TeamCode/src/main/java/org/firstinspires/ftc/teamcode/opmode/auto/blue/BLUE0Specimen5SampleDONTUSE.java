package org.firstinspires.ftc.teamcode.opmode.auto.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous
public class BLUE0Specimen5SampleDONTUSE extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;

    @Override
    public void initialize() {
        Constants.isRed = false;

        Pose2d home = new Pose2d(-38,-60.5, Math.toRadians(90));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        basket = new Basket(hardwareMap, telemetry);
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
        Action basketToSubmersible = BlueActions.basketToSubmersible(drive);

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new InstantCommand(() -> {
            basket.toHome();
            elevator.currentStage = Elevator.basketState.HOME;
            intake.pivotHome();
            intake.horizontalIn();
        }));

        final int basketDepositLen = 600;
        final int intakeRestingLen = 300;

        schedule(new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new TrajectoryGotoCommand(startToBasket, drive),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                        ),
                        new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(basketDepositLen),
                        new ParallelCommandGroup(
                                new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                new TrajectoryGotoCommand(basketToFirstSample, drive),
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 40)
                        ),
                        //new TurnToNearestSampleCommand(limelight, drive),
                        // First Sample
                        new SlideUntilHasPieceCommand(intake, Intake.color.BLUE, 40),
                        new IntakePositionCommand(intake, Intake.state.RESTING, intakeRestingLen),
//                        new ParallelRaceGroup(
//                                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 55).withTimeout(1500),
//                                new IntakeHasSampleCommand(intake)
//                        ),
                        new ParallelCommandGroup(
                                new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                ),
                                new SequentialCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                        new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                                )
                        ),
                        new ParallelCommandGroup(
                                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(basketDepositLen),
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 500, 40)
                        ),
                        new ParallelCommandGroup(
                                new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(500),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                // Second Sample
                                new TrajectoryGotoCommand(basketToSecondSample, drive)
                        ),
                        new SlideUntilHasPieceCommand(intake, Intake.color.BLUE, 40),
//                        new ParallelRaceGroup(
//                                new IntakePositionCommand(intake, Intake.state.INTAKING, 500, 55),
//                                new IntakeHasSampleCommand(intake)
//                        ),
                        new IntakePositionCommand(intake, Intake.state.RESTING, intakeRestingLen),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                        new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                                ),
                                new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                )
                        ),
                        new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(basketDepositLen),
                        new ParallelCommandGroup(
                                new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),

                                // Third Sample
                                new TrajectoryGotoCommand(basketToThirdSample, drive),
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 40)
                        ),
                        new ParallelRaceGroup(
                                new RawDrivetrainCommand(drive,.15, 0, 0).withTimeout(2000),
                                new IntakeHasSampleCommand(intake)
                        ),
                        new RawDrivetrainCommand(drive, -.3, 0, 0).withTimeout(300),
                        new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),
                        //new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                        new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                                ),
                                new StrafeToPositionCommand(new Pose2d(BlueActions.basketPos, Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                )
                        ),
                        new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(basketDepositLen),
                        new ParallelCommandGroup(
                                new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                // Fourth Sample
                                new TrajectoryGotoCommand(basketToSubmersible, drive)
                        ),
                        new KickerCommand(kicker, 500, Kicker.state.OPEN),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 300, 0),
                                        new SlideUntilHasPieceCommand(intake, Intake.color.BLUE)
                                ),
                                new KickerCommand(kicker, 500, Kicker.state.CLOSE)
                        ),
                        new RawDrivetrainCommand(drive, -.4, 0, 0).withTimeout(10),
                        new IntakePositionCommand(intake, Intake.state.RESTING, intakeRestingLen),
                        new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(10),

                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                        new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                                ),
                                new StrafeToPositionCommand(new Pose2d(new Vector2d(BlueActions.basketPos.x-1, BlueActions.basketPos.y+1), Math.toRadians(45)), drive),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                                )
                        ),
                        new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(basketDepositLen),
                        new ParallelCommandGroup(
                                new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                new TrajectoryGotoCommand(basketToSubmersible, drive)
                        )
                )
        );
    }
}
