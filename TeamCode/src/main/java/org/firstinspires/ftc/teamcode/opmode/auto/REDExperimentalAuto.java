package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.ElevatorGotoCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryCommand;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

@Autonomous
public class REDExperimentalAuto extends CommandOpMode {
    private Drivetrain drive;
    private Depositor depositor;


    @Override
    public void initialize() {
        Pose2d home = new Pose2d(-47,-60.5, Math.toRadians(45));

        drive = new Drivetrain(hardwareMap, home, telemetry);
        depositor = new Depositor(hardwareMap, telemetry);

        Vector2d basket = new Vector2d(-56.1923881554, -55.5502525317);
        //Vector2d[] toObservationZone = {new Vector2d(30, 20), new Vector2d(30, -70), new Vector2d(5, -70)};

        Action startToBasket = drive.getTrajectoryBuilder(home)
                .strafeTo(basket)
                .build();


        Action basketToObservation = drive.getTrajectoryBuilder(new Pose2d(basket, Math.toRadians(45)))
                .splineTo(new Vector2d(-24, -40), Math.toRadians(0))
                .splineTo(new Vector2d(23, -40), Math.toRadians(0))
                .splineTo(new Vector2d(46.9, -61), Math.toRadians(0))
                .build();

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new InstantCommand(() -> {
            depositor.basketToHome();
        }));
        schedule(new TrajectoryCommand(startToBasket, drive)
                .andThen(new ElevatorGotoCommand(depositor, Depositor.state.HIGH_BASKET))
                .andThen(new RunCommand(() -> depositor.basketToDeposit()).withTimeout(1000))
                .andThen(new RunCommand(() -> depositor.basketToHome()).withTimeout(1000))
                .andThen(new ElevatorGotoCommand(depositor, Depositor.state.HOME))
                .andThen(new TrajectoryCommand(basketToObservation, drive))
        );
        //.andThen(new TrajectoryCommand(basketToObservation, drive)));

    }
}
