package org.firstinspires.ftc.teamcode.opmode.auto.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.KickerCommand;
import org.firstinspires.ftc.teamcode.commands.SpecimenClawCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;

@Autonomous
public class BLUE4Specimen0Sample extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;
    private Kicker kicker;
    private SpecimenClaw claw;

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
        kicker = new Kicker(hardwareMap, telemetry);
        claw = new SpecimenClaw(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToSpecimen = BlueActions.startToSpecimen(drive, home, BlueActions.rightSpecimenPos);
        Action toFirstSample = BlueActions.toFirstSample(drive);
        Action pushFirstSample = BlueActions.pushFirstSample(drive);

        Action toSecondSample = BlueActions.toSecondSample(drive);
        Action pushSecondSample = BlueActions.pushSecondSample(drive);
        Action placeSecondSpecimen = BlueActions.placeSecondSpecimen(drive);

        Action pickupThirdSpecimen = BlueActions.pickupThirdSpecimen(drive);
        Action placeThirdSpecimen = BlueActions.placeThirdSpecimen(drive);

        Action pickupFourthSpecimen = BlueActions.pickupFourthSpecimen(drive);
        Action placeFourthSpecimen = BlueActions.placeFourthSpecimen(drive);

//        Action observationToPickup = drive.getTrajectoryBuilder(new Pose2d(new Vector2d(49.5, -40), Math.toRadians(270)))
//                .waitSeconds(1)
//                .strafeToSplineHeading(new Vector2d(50, -57), Math.toRadians(0))
//                .strafeTo(pickupPos)
//                .build();
//
//        Action pushAllToPickup = drive.getTrajectoryBuilder(new Pose2d(new Vector2d(44, -55), 0))
//                .strafeTo(new Vector2d(50, -62))
//                .build();
//
//        Action specimenToPickup = drive.getTrajectoryBuilder(new Pose2d(specimenPos, Math.toRadians(180)))
//                .strafeToSplineHeading(new Vector2d(50, -57), Math.toRadians(0))
//                .strafeTo(new Vector2d(pickupPos.x, pickupPos.y-2))
//                .build();

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
                new IntakePositionCommand(intake, Intake.state.RESTING),
                new ParallelCommandGroup(
                        new TrajectoryGotoCommand(startToSpecimen, drive),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN)
                ),
                new SequentialCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN, 250),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),
                                new TrajectoryGotoCommand(toFirstSample, drive)
                        ),
                        new KickerCommand(kicker, 50, Kicker.state.PUSH),
                        new TrajectoryGotoCommand(pushFirstSample, drive),
                        new ParallelCommandGroup(
                                new TrajectoryGotoCommand(toSecondSample, drive)
                        ),
                        new KickerCommand(kicker, 50, Kicker.state.PUSH),
                        new TrajectoryGotoCommand(pushSecondSample, drive)

                ),
                // both samples are pushed in
                new SpecimenClawCommand(claw, false),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new TrajectoryGotoCommand(placeSecondSpecimen, drive),
                        new SequentialCommandGroup(
                                new WaitCommand(1000),
                                new KickerCommand(kicker, 100, Kicker.state.CLOSE)
                        )
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN, 250),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),
                        new SequentialCommandGroup(
                                new WaitCommand(200),
                                new TrajectoryGotoCommand(pickupThirdSpecimen, drive)
                        )

                ),

                new SpecimenClawCommand(claw, false),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new TrajectoryGotoCommand(placeThirdSpecimen, drive)
                ),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN, 250),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                        ),
                        new TrajectoryGotoCommand(pickupFourthSpecimen, drive)
                ),
                new SpecimenClawCommand(claw, false),
                new WaitCommand(200),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new TrajectoryGotoCommand(placeFourthSpecimen, drive)
                ),
                new SequentialCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN, 250),
                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                )
                /*
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(new Vector2d(specimenPos.x, specimenPos.y-7), Math.toRadians(180)), drive),
                                new StrafeToPositionCommand(new Pose2d(new Vector2d(specimenPos.x, specimenPos.y + 1.75), Math.toRadians(180)), drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET, 75),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new TrajectoryGotoCommand(specimenToPickup, drive)
                        )
                ),
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(new Vector2d(specimenPos.x-7, specimenPos.y-7), Math.toRadians(180)), drive),
                                new StrafeToPositionCommand(new Pose2d(new Vector2d(specimenPos.x-7, specimenPos.y + 8.25), Math.toRadians(170)), drive)
                        )
                ),
                new ElevatorPositionCommand(elevator, Elevator.basketState.MIDDLE_BASKET, 75),
                /*
                new ParallelCommandGroup(
                        new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN),
                        new SequentialCommandGroup(
                                new WaitCommand(300),
                                new StrafeToPositionCommand(new Pose2d(specimenPos, Math.toRadians(180)), drive)
                        )
                ),*/
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
        ));
        //.andThen(new TrajectoryGotoCommand(basketToObservation, drive)));

    }
}
