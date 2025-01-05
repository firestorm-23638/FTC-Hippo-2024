package org.firstinspires.ftc.teamcode.opmode.auto.red;

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
public class RED1Specimen4Sample extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;

    private Pose2d shiftForward(double inches, double currentAngle) {
        return new Pose2d(new Vector2d(Math.cos(currentAngle) * -inches, Math.sin(currentAngle) * inches), currentAngle);
    }

    private Pose2d addPoses(Pose2d first, Pose2d second) {
        return new Pose2d(first.position.x + second.position.x, first.position.y + second.position.y, first.heading.log());
    }

    @Override
    public void initialize() {
        Constants.isRed = false;

        Pose2d home = new Pose2d(-14,-60.5, Math.toRadians(180));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        limelight = new Limelight(hardwareMap, telemetry);
        kicker = new Kicker(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToSpecimen = RedActions.startToSpecimen(drive, home, RedActions.leftSpecimenPos);
        Action specimenToFirstSample = RedActions.specimenToFirstSample(drive);
        Action basketToSecondSample = RedActions.basketToSecondSample(drive);
        Action basketToThirdSample  = RedActions.basketToThirdSample(drive);
        Action basketToSubmersible = RedActions.basketToSubmersible(drive);

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
                new ParallelCommandGroup( // drive to specimen, raise elevator
                        new TrajectoryGotoCommand(startToSpecimen, drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET, 100)
                ),
                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME), // place specimen
                new ParallelCommandGroup(  // drive to first sample, intake out
                        new TrajectoryGotoCommand(specimenToFirstSample, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING, 600, 40)
                ),
                //new TurnToNearestSampleCommand(limelight, drive),
                // First Sample
                new ParallelRaceGroup(  // go forward until has piece
                        new RawDrivetrainCommand(drive, .15, 0, 0).withTimeout(2000),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                new ParallelCommandGroup(
                        new StrafeToPositionCommand(new Pose2d(RedActions.basketPos, Math.toRadians(45)), drive),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                        ),

                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                        )

                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(700),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new SequentialCommandGroup(
                                new WaitCommand(700),
                                new ParallelCommandGroup(
                                        new IntakePositionCommand(intake, Intake.state.INTAKING, 500, 30),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                )
                        ),
                        // Second Sample
                        new TrajectoryGotoCommand(basketToSecondSample, drive)
                ),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .3, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                        ),
                        new StrafeToPositionCommand(new Pose2d(RedActions.basketPos, Math.toRadians(45)), drive),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                        )
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
                        new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 30)
                ),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive,.2, 0, 0).withTimeout(2000),
                        new IntakeHasSampleCommand(intake)
                ),
                new RawDrivetrainCommand(drive, -.2, 0, 0).withTimeout(700),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 500),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                        ),
                        new StrafeToPositionCommand(new Pose2d(RedActions.basketPos, Math.toRadians(45)), drive),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                        )
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(500),
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
                                new WaitCommand(300),
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 400, 0),
                                new SlideUntilHasPieceCommand(intake, Intake.color.RED)
                        ),
                        new KickerCommand(kicker, 500, Kicker.state.CLOSE)
                ),
                new RawDrivetrainCommand(drive, -.3, 0, 0).withTimeout(50),
                new IntakePositionCommand(intake, Intake.state.RESTING, 700),
                new RawDrivetrainCommand(drive, 0, 0, 0).withTimeout(50),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.TRANSFERRING, 500),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 200)
                        ),
                        new StrafeToPositionCommand(new Pose2d(new Vector2d(RedActions.basketPos.x-1, RedActions.basketPos.y+1), Math.toRadians(45)), drive),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                        )
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(700),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new SequentialCommandGroup(
                                new WaitCommand(500),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        )
                ),
                new InstantCommand(() -> Constants.pose = drive.getCurrentPose())
        ));
    }
}