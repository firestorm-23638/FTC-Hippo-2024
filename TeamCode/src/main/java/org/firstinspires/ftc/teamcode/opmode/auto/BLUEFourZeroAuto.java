package org.firstinspires.ftc.teamcode.opmode.auto;

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
import org.firstinspires.ftc.teamcode.commands.BlankCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeHasSampleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToNearestSampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@Autonomous
public class BLUEFourZeroAuto extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;

    private Pose2d shiftForward(double inches, double currentAngle) {
        return new Pose2d(new Vector2d(Math.cos(currentAngle) * -inches, Math.sin(currentAngle) * inches), currentAngle);
    }

    private Pose2d addPoses(Pose2d first, Pose2d second) {
        return new Pose2d(first.position.x + second.position.x, first.position.y + second.position.y, first.heading.log());
    }

    @Override
    public void initialize() {
        Constants.isRed = false;

        Pose2d home = new Pose2d(12, -61, Math.toRadians(180));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        limelight = new Limelight(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        final Vector2d specimenPos = new Vector2d(0, -31.5);
        final Vector2d pickupPos = new Vector2d(50, -66);

        final Pose2d firstSample = new Pose2d(new Vector2d(-36, -35), Math.toRadians(100));
        final Pose2d inchToFirstSamplePose = addPoses(shiftForward(5, Math.toRadians(100)), firstSample);

        Action startToSpecimen = drive.getTrajectoryBuilder(home)
                .strafeTo(specimenPos)
                .build();

        Action specimenToPushAll = drive.getTrajectoryBuilder(new Pose2d(specimenPos, Math.toRadians(180)))
                .setReversed(true)
                // from specimen to first spike mark
                .splineToConstantHeading(new Vector2d(10, -37), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(30.5, -37, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(30.5, -10, Math.toRadians(180)), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(49.5, -12, Math.toRadians(270)), Math.toRadians(270))
                // from first spike to obs then back
                .splineToConstantHeading(new Vector2d(49.5, -55), Math.toRadians(270))
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(49.5, -13), Math.toRadians(90))
//                .strafeTo(new Vector2d(60, -13))
//                // second spike to obs then back
//                .strafeTo(new Vector2d(60, -55))
                .waitSeconds(0.1)
                .setReversed(true)
                .build();

        Action observationToPickup = drive.getTrajectoryBuilder(new Pose2d(new Vector2d(48, -55), Math.toRadians(270)))
                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(50, -57), Math.toRadians(0))
                .strafeTo(pickupPos)
                .build();

        Action specimenToPickup = drive.getTrajectoryBuilder(new Pose2d(specimenPos, Math.toRadians(180)))
                .strafeToSplineHeading(new Vector2d(50, -57), Math.toRadians(0))
                .strafeTo(pickupPos)
                .build();

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
                        new TrajectoryGotoCommand(startToSpecimen, drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET, 100)
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET, 50),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new TrajectoryGotoCommand(specimenToPushAll, drive),
                                new TrajectoryGotoCommand(observationToPickup, drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(specimenPos, Math.toRadians(180)), drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET, 50),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new TrajectoryGotoCommand(specimenToPickup, drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(specimenPos, Math.toRadians(180)), drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET, 50),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new TrajectoryGotoCommand(specimenToPickup, drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(specimenPos, Math.toRadians(180)), drive)
                        )
                ),
                /*new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET, 50),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new TrajectoryGotoCommand(specimenToPickup, drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(specimenPos, Math.toRadians(180)), drive)
                        )
                ),*/
                new InstantCommand(() -> Constants.pose = drive.getCurrentPose())
        ));
        //.andThen(new TrajectoryGotoCommand(basketToObservation, drive)));

    }
}
