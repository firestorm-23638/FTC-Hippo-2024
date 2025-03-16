package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

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

        limelight.toYellowAndRed();
    }

    @Override
    public void execute() {
        List<LLResultTypes.ColorResult> results = limelight.lookForSamples();
        if (results != null) {
            currentAngle = results.get(0).getTargetXDegrees();
            hasTarget = true;
        }
        if (hasTarget) {
            drivetrain.driveArcade(speed, 0, currentAngle * 0.025);
            isAligned = (currentAngle > -1) && (currentAngle < 1);
        }
        else {
            drivetrain.driveArcade(speed, 0, turnSpeed);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
