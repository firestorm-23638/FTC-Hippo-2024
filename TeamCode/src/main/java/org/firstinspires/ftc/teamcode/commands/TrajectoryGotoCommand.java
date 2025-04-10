package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.concurrent.TimeUnit;

public class TrajectoryGotoCommand extends CommandBase {
    private Action trajectoryAction;
    private boolean isFinished = false;
    private Timing.Timer minimumTimer = new Timing.Timer(100, TimeUnit.MILLISECONDS);

    public TrajectoryGotoCommand(Drivetrain drivetrain, Action trajectoryAction) {
        addRequirements(drivetrain);
        this.trajectoryAction = trajectoryAction;
    }

    @Override
    public void initialize() {
        minimumTimer.start();
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        trajectoryAction.preview(packet.fieldOverlay());
        isFinished = !trajectoryAction.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    @Override
    public boolean isFinished() {
        return minimumTimer.done() && (isFinished);
    }
}