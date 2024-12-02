package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

public class TurnToNearestSampleCommand extends CommandBase {
    private final Limelight limelight;
    private final Drivetrain drivetrain;

    double currentAngle;
    boolean hasTarget = false;
    boolean isAligned = false;

    public TurnToNearestSampleCommand(Limelight limelight, Drivetrain drivetrain) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;

        addRequirements(limelight, drivetrain);

        limelight.toYellowSample();
    }

    @Override
    public void execute() {
        List<LLResultTypes.ColorResult> results = limelight.lookForYellowSample();
        if (results != null) {
            currentAngle = results.get(0).getTargetXDegrees();
            hasTarget = true;
        }
        if (hasTarget) {
            drivetrain.driveArcade(0, 0, currentAngle * 0.03);
            isAligned = (currentAngle > -1) && (currentAngle < 1);
        }
    }

    @Override
    public boolean isFinished() {
        return isAligned;
    }
}
