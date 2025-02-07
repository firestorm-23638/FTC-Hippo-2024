package org.firstinspires.ftc.teamcode.opmode.auto.blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
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
import org.firstinspires.ftc.teamcode.commands.DepositorCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeHasSampleCommand;
import org.firstinspires.ftc.teamcode.commands.IntakePositionCommand;
import org.firstinspires.ftc.teamcode.commands.RawDrivetrainCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.commands.TransitionCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToNearestBlueSampleCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToNearestYellowSampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Kicker;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;

@Autonomous
public class BLUE4Specimen0SamplePUSH extends CommandOpMode {
    private Drivetrain drive;
    private Basket basket;
    private Elevator elevator;
    private Intake intake;
    private Limelight limelight;
    private Depositor depositor;

    @Override
    public void initialize() {
        Constants.isRed = false;

        Pose2d home = new Pose2d(12, -61, Math.toRadians(270));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry, Intake.color.RED);
        limelight = new Limelight(hardwareMap, telemetry);
        depositor = new Depositor(hardwareMap, telemetry);

        drive.forwardSpeedlimit = 1;
        drive.strafeSpeedlimit = 1;
        drive.rotSpeedLimit = 1;

        Action startToSpecimen = BlueActions.startToSpecimen(drive, home, BlueActions.rightSpecimenPos);
        Action pushBothSamples = BlueActions.pushTwoSamples(drive);

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
            elevator.currentStage = Elevator.basketState.MIDDLE_BASKET;
            intake.currentState = Intake.state.RESTING;
            depositor.toHome();
            depositor.clawTighten();
        }));

        schedule(new SequentialCommandGroup(
                new TrajectoryGotoCommand(startToSpecimen, drive),
                new DepositorCommand(depositor, Depositor.state.SPECIMEN).withTimeout(Constants.depositorArmSpecimenTimeMs),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                                new DepositorCommand(depositor, Depositor.state.TRANSITIONING).withTimeout(Constants.depositorArmSpecimenTimeMs)
                        ),
                        new TrajectoryGotoCommand(pushBothSamples, drive)
                ),
                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 20),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .2, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 400, 20),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 400, 0),
                                new TransitionCommand(depositor, intake, elevator),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 5, 0),
                                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN)
                        ),
                        new TrajectoryGotoCommand(placeSecondSpecimen, drive)
                ),
                // Place 2nd Specimen
                new DepositorCommand(depositor, Depositor.state.SPECIMEN).withTimeout(Constants.depositorArmSpecimenTimeMs),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                                new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(Constants.depositorArmSpecimenTimeMs)
                        ),
                        new ParallelCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 60),
                                new TrajectoryGotoCommand(pickupThirdSpecimen, drive)
                        )
                ),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .2, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 400, 20),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 400, 0),
                                new TransitionCommand(depositor, intake, elevator),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 5, 0),
                                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN)
                        ),
                        new TrajectoryGotoCommand(placeThirdSpecimen, drive)
                ),
                // Place 3rd Specimen
                new DepositorCommand(depositor, Depositor.state.SPECIMEN).withTimeout(Constants.depositorArmSpecimenTimeMs),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                                new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(Constants.depositorArmSpecimenTimeMs)
                        ),
                        new ParallelCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 60),
                                new TrajectoryGotoCommand(pickupFourthSpecimen, drive)
                        )
                ),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .2, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new IntakePositionCommand(intake, Intake.state.RESTING, 400, 40),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 400, 0),
                                new TransitionCommand(depositor, intake, elevator),
                                new IntakePositionCommand(intake, Intake.state.RESTING, 5, 0),
                                new DepositorCommand(depositor, Depositor.state.CLAWTIGHTEN),
                                new ElevatorPositionCommand(elevator, Elevator.basketState.SPECIMEN)
                        ),
                        new TrajectoryGotoCommand(placeFourthSpecimen, drive)
                ),
                // Place 3rd Specimen
                new DepositorCommand(depositor, Depositor.state.SPECIMEN).withTimeout(Constants.depositorArmSpecimenTimeMs)
                /*
                // Drive to pickup 3rd Specimen
                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new DepositorCommand(depositor, Depositor.state.CLAWOPEN).withTimeout(Constants.depositorClawOpenTimeMs),
                                new DepositorCommand(depositor, Depositor.state.HOME).withTimeout(Constants.depositorArmSpecimenTimeMs)
                        ),
                        new TrajectoryGotoCommand(pickupThirdSpecimen, drive)
                ),
                new ParallelCommandGroup(
                        new TurnToNearestBlueSampleCommand(limelight, drive),
                        new IntakePositionCommand(intake, Intake.state.INTAKING, 700, 20)
                ),
                new ParallelRaceGroup(
                        new RawDrivetrainCommand(drive, .2, 0, 0).withTimeout(1500),
                        new IntakeHasSampleCommand(intake)
                ),
                new ParallelCommandGroup(
                        new IntakePositionCommand(intake, Intake.state.RESTING, 700),

                        new TrajectoryGotoCommand(placeSecondSpecimen, drive)
                ),
                // Place 3rd Specimen
                new DepositorCommand(depositor, Depositor.state.SPECIMEN).withTimeout(Constants.depositorArmSpecimenTimeMs),
                */

                /*
                new SequentialCommandGroup(
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(200),
                                        new ElevatorPositionCommand(elevator, Elevator.basketState.HOME)
                                ),

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
