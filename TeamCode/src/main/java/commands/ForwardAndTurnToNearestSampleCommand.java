package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Drivetrain;
import subsystems.Limelight;

public class ForwardAndTurnToNearestSampleCommand extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    double currentAngle;
    double turnSpeed = 0;
    double speed = 0;
    boolean hasTarget = false;
    boolean isAligned = false;

    public ForwardAndTurnToNearestSampleCommand(Limelight limelight, Drivetrain drivetrain, double speed) {
        this(limelight, drivetrain, speed, 0);
    }

    public ForwardAndTurnToNearestSampleCommand(Limelight limelight, Drivetrain drivetrain, double speed, double turnSpeed) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.speed = speed;
        this.turnSpeed = turnSpeed;

        addRequirements(limelight, drivetrain);

        limelight.toYellowAndBlue();
    }

    @Override
    public void execute() {
        double[] results = limelight.lookForSamples();
        if (results != null) {
            currentAngle = results[0];
            hasTarget = true;
        }
        if (hasTarget) {
            drivetrain.setRawSpeeds(speed, 0, currentAngle * 0.025);
            isAligned = (currentAngle > -1) && (currentAngle < 1);
        }
        else {
            drivetrain.setRawSpeeds(speed, 0, turnSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
