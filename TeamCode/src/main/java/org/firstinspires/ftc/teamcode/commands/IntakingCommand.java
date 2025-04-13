package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakingCommand extends CommandBase {
    private final Intake m_intake;

    public IntakingCommand(Intake intake) {
        m_intake = intake;

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        m_intake.currentState = Intake.state.INTAKING;
    }

    @Override
    public void execute() {

    }

    @Override
    public boolean isFinished() {
        return (m_intake.currentColor != Intake.color.NONE) && m_intake.hasCorrectColor();
    }
}