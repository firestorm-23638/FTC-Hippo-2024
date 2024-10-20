package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class ElevatorGotoCommand extends CommandBase {
    private final Depositor m_depositor;
    private Depositor.state state;
    private boolean isFinished = false;

    public ElevatorGotoCommand(Depositor depositor, Depositor.state state) {
        addRequirements(depositor);
        m_depositor = depositor;
        this.state = state;
    }

    @Override
    public void execute() {
        m_depositor.currentStage = state;
        isFinished = m_depositor.isAtPos();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
