package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Drivetrain;
import subsystems.Limelight;

public class LimelightCommand extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    public LimelightCommand(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(limelight, drivetrain);
    }

    @Override
    public void execute() {
        limelight.updatePosition();
    }
}
