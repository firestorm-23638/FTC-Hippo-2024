package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

public class ForwardUntilHasPieceCommand extends CommandBase {
    private final Intake intake;
    private final Drivetrain drivetrain;
    private final Intake.color colorToIgnore;
    private double speed;

    public ForwardUntilHasPieceCommand(Intake intake, Drivetrain drivetrain, double speed, Intake.color colorToIgnore) {
        this.intake = intake;
        this.drivetrain = drivetrain;
        this.colorToIgnore = colorToIgnore;
        this.speed = speed;

        addRequirements(intake, drivetrain);
    }

    @Override
    public void execute() {
        drivetrain.driveArcade(speed, 0, 0);
        intake.setVacuumRun();
    }

    @Override
    public boolean isFinished() {
        return (intake.getCurrentColor() != Intake.color.NONE) && (intake.getCurrentColor() != colorToIgnore);
    }
}
