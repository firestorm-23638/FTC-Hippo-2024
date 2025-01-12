package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class TrajectoryGotoCommand extends CommandBase {
    private Action trajectoryAction;
    private boolean isFinished = false;


    public TrajectoryGotoCommand(Action trajectoryAction, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.trajectoryAction = trajectoryAction;
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
        return isFinished;
    }
}
