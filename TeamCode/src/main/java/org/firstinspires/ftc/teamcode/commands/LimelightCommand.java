package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

public class LimelightCommand extends CommandBase {
    private final Limelight limelight;

    public LimelightCommand(Limelight limelight) {
        this.limelight = limelight;

        addRequirements(limelight);
    }

    @Override
    public void execute() {
        limelight.updatePosition();
    }
}
