package org.firstinspires.ftc.teamcode.opmode.teleop;


import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.TurnToNearestYellowSampleCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

@TeleOp
public class OrientToPixelTeleop extends CommandOpMode {

    private Drivetrain drive;

    private Limelight limelight;

    @Override
    public void initialize() {
        drive = new Drivetrain(hardwareMap, new Pose2d(0, 0, 0), telemetry);
        limelight = new Limelight(hardwareMap, telemetry);
        drive.setDefaultCommand(new TurnToNearestYellowSampleCommand(limelight, drive));
        // Reads limelight position for now
        //limelight.setDefaultCommand(new LimelightCommand(limelight, drive));
        // If a subsystem has a default command, you don't need to register.
        // Automatically updates telemetry

        schedule(new RunCommand(telemetry::update));
        waitForStart();
        // Put game start code here. i.e home everything
        schedule(new InstantCommand(() -> {

        }));
    }
}