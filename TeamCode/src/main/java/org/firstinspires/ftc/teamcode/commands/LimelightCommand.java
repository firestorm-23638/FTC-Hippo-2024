package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

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
