package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

public class TurnToNearestBlueSampleCommand extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    double currentAngle;
    double startingSpeed = 0;
    boolean hasTarget = false;
    boolean isAligned = false;

    public TurnToNearestBlueSampleCommand(Limelight limelight, Drivetrain drivetrain) {
        this(limelight, drivetrain, 0);
    }

    public TurnToNearestBlueSampleCommand(Limelight limelight, Drivetrain drivetrain, double startingSpeed) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.startingSpeed = startingSpeed;

        addRequirements(limelight, drivetrain);

        limelight.toBlueSample();
    }

    @Override
    public void execute() {
        List<LLResultTypes.ColorResult> results = limelight.lookForSamples();
        if (results != null) {
            currentAngle = results.get(0).getTargetXDegrees();
            hasTarget = true;
        }
        if (hasTarget) {
            drivetrain.driveArcade(0, 0, currentAngle * 0.05);
            isAligned = (currentAngle > -1) && (currentAngle < 1);
        }
        else {
            drivetrain.driveArcade(0, 0, startingSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return isAligned;
    }
}
