package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class TurnAndForwardCommand extends CommandBase {
    private final Drivetrain m_drivetrain;
    private double forward;
    private double targetAngle;

    public TurnAndForwardCommand(Drivetrain drive, double forward, double targetAngle) {
        m_drivetrain = drive;

        this.forward = forward;
        this.targetAngle = targetAngle;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.setRawPower(forward, 0, (targetAngle - Math.toDegrees(m_drivetrain.getCurrentPose().heading.log())) * -0.0135);
    }
}
