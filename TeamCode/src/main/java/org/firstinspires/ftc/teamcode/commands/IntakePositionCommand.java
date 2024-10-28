package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakePositionCommand extends CommandBase {
    private final Intake m_intake;
    private Intake.state state = Intake.state.RESTING;

    public IntakePositionCommand(Intake intake, Intake.state state) {
        m_intake = intake;
        this.state = state;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        if (state == Intake.state.INTAKING) {
            m_intake.pivotDown();
            m_intake.horizontalOut();
            m_intake.vacuumRun();
        }
        else if (state == Intake.state.RESTING){
            m_intake.pivotHome();
            m_intake.horizontalIn();
            m_intake.vacuumStop();
        }
        else {
            m_intake.pivotBasket();
            m_intake.vacuumEject();
        }
    }
}
