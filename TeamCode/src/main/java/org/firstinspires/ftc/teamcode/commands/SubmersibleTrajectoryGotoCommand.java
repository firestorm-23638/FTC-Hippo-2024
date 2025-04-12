package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

public class SubmersibleTrajectoryGotoCommand extends CommandBase {
    private Drivetrain drivetrain;
    private Action trajectoryAction;
    private boolean isFinished = false;
    private Timing.Timer minimumTimer = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    public SubmersibleTrajectoryGotoCommand(Drivetrain drivetrain, Action trajectoryAction) {
        addRequirements(drivetrain);
        this.trajectoryAction = trajectoryAction;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        minimumTimer.start();
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        trajectoryAction.preview(packet.fieldOverlay());
        trajectoryAction.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        isFinished = (drivetrain.getCurrentPose().position.x > -11);
    }

    @Override
    public boolean isFinished() {
        return minimumTimer.done() && (isFinished);
    }
}