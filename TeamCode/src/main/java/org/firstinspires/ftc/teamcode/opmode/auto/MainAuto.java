package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.BasketPositionCommand;
import org.firstinspires.ftc.teamcode.commands.ElevatorPositionCommand;
import org.firstinspires.ftc.teamcode.commands.TrajectoryGotoCommand;
import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@Autonomous
public class MainAuto extends CommandOpMode {
    private Drivetrain drive;
    private Elevator elevator;
    private Basket basket;
    private Intake intake;


    @Override
    public void initialize() {
        drive = new Drivetrain(hardwareMap, new Pose2d(new Vector2d(0, 0), Math.toRadians(0)), telemetry);
        elevator = new Elevator(hardwareMap, telemetry);
        basket = new Basket(hardwareMap, telemetry);
        intake = new Intake(hardwareMap, telemetry);

        Vector2d home = new Vector2d(0, 0);
        Vector2d toBasket = new Vector2d(-3, 10);
        Vector2d toObservationZone = new Vector2d(-70, 0);

        Action startToBasket = drive.getTrajectoryBuilder(new Pose2d(home, Math.toRadians(0)))
                .strafeTo(toBasket)
                .build();


        Action basketToObservation = drive.getTrajectoryBuilder(new Pose2d(toBasket, Math.toRadians(0)))
                .splineTo(toObservationZone, Math.toRadians(90))
                .build();

        register(drive);
        schedule(new RunCommand(telemetry::update));
        waitForStart();
        schedule(new InstantCommand(() -> {
            basket.toHome();
            elevator.currentStage = Elevator.state.HOME;
            intake.pivotHome();
            intake.horizontalIn();
        }));
        schedule(new TrajectoryGotoCommand(startToBasket, drive)
                .alongWith(new ElevatorPositionCommand(elevator, Elevator.state.HIGH_BASKET))
                .andThen(new BasketPositionCommand(basket, Basket.state.BUCKET)).withTimeout(1000)
                .andThen(new BasketPositionCommand(basket, Basket.state.HOME)).withTimeout(1000)
                .andThen(new ElevatorPositionCommand(elevator, Elevator.state.HOME))
        );
                //.andThen(new TrajectoryCommand(basketToObservation, drive)));

    }
}
