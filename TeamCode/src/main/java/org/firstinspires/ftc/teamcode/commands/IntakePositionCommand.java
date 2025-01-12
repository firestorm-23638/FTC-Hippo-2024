package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakePositionCommand extends CommandBase {
    private final Intake m_intake;
    private Intake.state state = Intake.state.RESTING;
    private Timing.Timer extendTimer;
    private boolean isTimer = false;
    double trim = 0;

    public IntakePositionCommand(Intake intake, Intake.state state) {
        this(intake, state, 0, 0);
    }

    public IntakePositionCommand(Intake intake, Intake.state state, long ms) {
        this(intake, state, ms, 0);
    }

    public IntakePositionCommand(Intake intake, Intake.state state, long ms, double trim) {
        m_intake = intake;
        this.state = state;
        extendTimer = new Timing.Timer(ms, TimeUnit.MILLISECONDS);
        isTimer =  true;
        this.trim = trim;

        addRequirements(m_intake);
    }

    @Override
    public void initialize() {
        extendTimer.start();
        m_intake.trim = trim;
    }

    @Override
    public void execute() {
        if (state == Intake.state.INTAKING) {
            m_intake.pivotDown();
            m_intake.horizontalOut();
            m_intake.setVacuumRun();
        }
        else if (state == Intake.state.RESTING){
            m_intake.trim = 0;
            m_intake.pivotHome();
            m_intake.horizontalIn();
            m_intake.setVacuumStop();
       }
        else if (state == Intake.state.SLIGHTLY) {
            m_intake.pivotHome();
            m_intake.extendSlightly();
            m_intake.setVacuumStop();
        }
        else {
            m_intake.trim = 0;
            m_intake.pivotBasket();
            m_intake.setVacuumEject();
            m_intake.horizontalTransfer();
        }
    }

    @Override
    public boolean isFinished() {
        if (!isTimer) {
            return false;
        }
        return extendTimer.done();
    }
}