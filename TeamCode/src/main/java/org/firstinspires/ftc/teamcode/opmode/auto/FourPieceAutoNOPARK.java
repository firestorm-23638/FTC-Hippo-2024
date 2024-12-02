package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.BasketPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.StrafeToPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class FourPieceAutoNOPARK extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;

    private Pose2d shiftForward(double inches, double currentAngle) {
        return new Pose2d(new Vector2d(Math.cos(currentAngle) * -inches, Math.sin(currentAngle) * inches), currentAngle);
    }

    private Pose2d addPoses(Pose2d first, Pose2d second) {
        return new Pose2d(first.position.x + second.position.x, first.position.y + second.position.y, first.heading.log());
    }

    @Override
    public void initialize() {
        Constants.isRed = true;

        Pose2d home = new Pose2d(-38,-60.5, Math.toRadians(90));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.NONE);

        final Vector2d basketPos = new Vector2d(-59.923881554, -54.0502525317);
        final Pose2d firstSample = new Pose2d(new Vector2d(-36, -35), Math.toRadians(100));
        final Pose2d inchToFirstSamplePose = addPoses(shiftForward(5, Math.toRadians(100)), firstSample);

        Action startToBasket = drive.getTrajectoryBuilder(home)
                .strafeToLinearHeading(basketPos, Math.toRadians(45))
                .build();

        Action basketToFirstSample = drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .setTangent(0)
                .splineTo(new Vector2d(-28.5, -39), Math.toRadians(100))
                .build();

        Action basketToSecondSample = drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .setTangent(0)
                .splineTo(new Vector2d(-37, -40), Math.toRadians(100))
                .build();

        Action basketToThirdSample = drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .setTangent(0)
                .splineTo(new Vector2d(-49, -38.5), Math.toRadians(100))
                .build();

        Action basketToObservation = drive.getTrajectoryBuilder(new Pose2d(basketPos, Math.toRadians(45)))
                .splineTo(new Vector2d(-24, -35), Math.toRadians(0))
                .splineTo(new Vector2d(30, -35), Math.toRadians(0))
                .splineTo(new Vector2d(55, -61), Math.toRadians(0))
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

        final double strafeSpeed = -0.05;

        schedule(new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new TrajectoryGotoCommand(startToBasket, drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(600),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),
                        new TrajectoryGotoCommand(basketToFirstSample, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING).withTimeout(1000)
                ),

                // First Sample
                new RunCommand(() -> {
                    drive.driveArcade(.3, strafeSpeed, 0);
                }, drive).withTimeout(800),
                new RunCommand(() -> {
                    drive.driveArcade(0, strafeSpeed, 0);
                }, drive).withTimeout(550),
                new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(700),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(700),
                new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(200),
                new ParallelCommandGroup(
                        new StrafeToPositionCommand(new Pose2d(basketPos, Math.toRadians(45)), drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(600),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),

                        // Second Sample
                        new TrajectoryGotoCommand(basketToSecondSample, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING).withTimeout(1000)
                ),
                new RunCommand(() -> {
                    drive.driveArcade(.3, strafeSpeed, 0);
                }, drive).withTimeout(800),
                new RunCommand(() -> {
                    drive.driveArcade(0, strafeSpeed, 0);
                }, drive).withTimeout(550),
                new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(700),
                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(700),
                new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(200),
                new ParallelCommandGroup(
                        new StrafeToPositionCommand(new Pose2d(basketPos, Math.toRadians(45)), drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(600),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME),

                        // Third Sample
                        new TrajectoryGotoCommand(basketToThirdSample, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING).withTimeout(1000)
                ),
                new RunCommand(() -> {
                    drive.driveArcade(.2, strafeSpeed, 0);
                }, drive).withTimeout(1000),
                new RunCommand(() -> {
                    drive.driveArcade(-.2, strafeSpeed, 0);
                }, drive).withTimeout(550),
                new ParallelCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(700),
                        new RunCommand(() -> {
                            drive.driveArcade(0, 0, 0);
                        }, drive).withTimeout(500)
                ),

                new IntakePositionCommand(intake, Intake.state.TRANSFERRING).withTimeout(700),
                new IntakePositionCommand(intake, Intake.state.RESTING).withTimeout(200),
                new ParallelCommandGroup(
                        new StrafeToPositionCommand(new Pose2d(basketPos, Math.toRadians(45)), drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HIGH_BASKET)
                ),
                new BasketPositionCommand(basket, Basket.state.BUCKET).withTimeout(600),
                new ParallelCommandGroup(
                        new BasketPositionCommand(basket, Basket.state.HOME).withTimeout(1000),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        //new TrajectoryGotoCommand(basketToObservation, drive)
                )
        ));
        //.andThen(new TrajectoryGotoCommand(basketToObservation, drive)));

    }
}
