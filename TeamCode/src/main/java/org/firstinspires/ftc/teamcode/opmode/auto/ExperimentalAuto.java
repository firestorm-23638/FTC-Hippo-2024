package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.commands.ElevatorGotoCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous
public class ExperimentalAuto extends CommandOpMode {
    private Drivetrain drive;
    private Depositor depositor;


    @Override
    public void initialize() {
        drive = new Drivetrain(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)), telemetry);
        depositor = new Depositor(hardwareMap, telemetry);

        Vector2d home = new Vector2d(0, 0);
        Vector2d toBasket = new Vector2d(15, 20);
        Vector2d[] toObservationZone = {new Vector2d(30, 20), new Vector2d(30, -70), new Vector2d(5, -70)};

        Action startToBasket = drive.getTrajectoryBuilder(new Pose2d(home, Math.toRadians(0)))
                .strafeToLinearHeading(toBasket, Math.toRadians(45))
                .build();


        Action basketToObservation = drive.getTrajectoryBuilder(new Pose2d(toBasket, Math.toRadians(0)))
                .splineTo(toObservationZone[0], Math.toRadians(0))
                .splineTo(toObservationZone[1], Math.toRadians(0))
                .splineTo(toObservationZone[2], Math.toRadians(0))
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
                .andThen(new TrajectoryCommand(basketToObservation, drive))
                .alongWith(new ElevatorGotoCommand(depositor, Depositor.state.HOME))
        );
        //.andThen(new TrajectoryCommand(basketToObservation, drive)));

    }
}
