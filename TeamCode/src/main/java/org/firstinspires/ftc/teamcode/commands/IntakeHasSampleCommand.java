package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

// Keeps running until the intake has a sample
public class IntakeHasSampleCommand extends CommandBase {
    private final Intake m_intake;
    private boolean hasPiece = false;

    public IntakeHasSampleCommand(Intake intake) {
        m_intake = intake;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        hasPiece = m_intake.getCurrentColor() != Intake.color.NONE;
    }

    @Override
    public boolean isFinished() {
        return hasPiece;
    }
}