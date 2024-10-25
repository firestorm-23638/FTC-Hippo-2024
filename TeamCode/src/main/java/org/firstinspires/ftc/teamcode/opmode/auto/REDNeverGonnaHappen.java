package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ElevatorGotoCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeGotoCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class REDNeverGonnaHappen extends CommandOpMode {
    private Drivetrain drive;
    private Depositor depositor;
    private Intake intake;

    Action shiftForward(Pose2d currentPose, int amt) {
        return drive.getTrajectoryBuilder(currentPose)
                .lineToY(amt)
                .build();
    }

    @Override
    public void initialize() {
        drive = new Drivetrain(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)), telemetry);
        depositor = new Depositor(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Pose2d home = new Pose2d(-47,-60.5, Math.toRadians(45));
        Vector2d basket = new Vector2d(-56.1923881554, -55.5502525317);
        //Vector2d[] toObservationZone = {new Vector2d(30, 20), new Vector2d(30, -70), new Vector2d(5, -70)};

        Action startToBasket = drive.getTrajectoryBuilder(home)
                .strafeTo(basket)
                .build();

        Action basketToFirstSample = drive.getTrajectoryBuilder(new Pose2d(basket, Math.toRadians(45)))
                .setTangent(0)
                .splineTo(new Vector2d(-36, -35), Math.toRadians(100))
                .build();

        Action basketToObservation = drive.getTrajectoryBuilder(new Pose2d(basket, Math.toRadians(45)))
                .splineTo(new Vector2d(-24, -35), Math.toRadians(0))
                .splineTo(new Vector2d(10, -35), Math.toRadians(0))
                .splineTo(new Vector2d(46.9, -61), Math.toRadians(0))
                .build();

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new InstantCommand(() -> {
            depositor.basketToHome();
        }));
        schedule(new TrajectoryCommand(startToBasket, drive)
                .alongWith(new ElevatorGotoCommand(depositor, Depositor.state.HIGH_BASKET))
                .andThen(new RunCommand(() -> depositor.basketToDeposit()).withTimeout(1000))
                .andThen(new RunCommand(() -> depositor.basketToHome()).withTimeout(1000))
                .andThen(new TrajectoryCommand(basketToFirstSample, drive))
                .alongWith(new ElevatorGotoCommand(depositor, Depositor.state.HOME))
                .andThen(new IntakeGotoCommand(intake, Intake.state.INTAKING).withTimeout(2000))
                //.alongWith(new TrajectoryCommand(shiftForward()))
        );
        //.andThen(new TrajectoryCommand(basketToObservation, drive)));

    }
}
