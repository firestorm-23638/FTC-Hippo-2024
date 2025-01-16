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
    private boolean isPos = false;
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
        m_intake.currentState = state;
    }

    @Override
    public boolean isFinished() {
        if (m_intake.isAtPos(state)) {
            return true;
        }
        if (!isTimer) {
            return false;
        }
        return extendTimer.done();
    }
}