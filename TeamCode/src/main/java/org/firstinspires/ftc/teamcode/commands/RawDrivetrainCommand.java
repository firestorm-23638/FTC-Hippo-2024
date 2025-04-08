package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class RawDrivetrainCommand extends CommandBase {
    private final Drivetrain m_drivetrain;
    private double forward;
    private double strafe;
    private double turn;

    public RawDrivetrainCommand(Drivetrain drive, double forward, double strafe, double turn) {
        m_drivetrain = drive;

        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.setRawPower(forward, strafe, turn);
    }
}
