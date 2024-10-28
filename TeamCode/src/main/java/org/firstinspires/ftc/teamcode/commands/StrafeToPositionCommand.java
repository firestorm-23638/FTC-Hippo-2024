package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class StrafeToPositionCommand extends CommandBase {
    private Action action;
    private final Pose2d targetPose;
    private boolean isFinished = false;
    private Drivetrain drivetrain;


    public StrafeToPositionCommand(Pose2d targetPose, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.targetPose = targetPose;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        action = drivetrain.getTrajectoryBuilder(drivetrain.getCurrentPose())
                .strafeToLinearHeading(targetPose.position, targetPose.heading.log())
                .build();
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        action.preview(packet.fieldOverlay());
        isFinished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
