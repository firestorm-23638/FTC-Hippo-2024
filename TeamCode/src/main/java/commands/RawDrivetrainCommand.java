package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Drivetrain;

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
        m_drivetrain.setRawSpeeds(forward, strafe, turn);
    }
}
