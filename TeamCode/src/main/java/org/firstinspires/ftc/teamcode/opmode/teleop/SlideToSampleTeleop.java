package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.CalculateAndTurnLimelightCommand;
import org.firstinspires.ftc.teamcode.commands.SlideUntilHasPieceCommand;
import org.firstinspires.ftc.teamcode.commands.TurnToNearestYellowSampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp
public class SlideToSampleTeleop extends CommandOpMode {

    private Intake intake;

    private Limelight limelight;

    @Override
    public void initialize() {
        intake = new Intake(hardwareMap, telemetry,  Intake.color.RED, gamepad1);

        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));
        // If a subsystem has a default command, you don't need to register.
        // Automatically updates telemetry

        waitForStart();
        schedule(new RunCommand(telemetry::update));
        register(intake);

        // Put game start code here. i.e home everything
        schedule(new InstantCommand(() -> {
            intake.currentState = Intake.state.INTAKING;
            intake.trim = 0;
        }));
        schedule(new SlideUntilHasPieceCommand(intake, Intake.color.BLUE));
    }
}