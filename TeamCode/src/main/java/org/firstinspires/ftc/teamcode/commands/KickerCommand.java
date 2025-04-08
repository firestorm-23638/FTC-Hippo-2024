package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.subsystems.Kicker;

import java.util.concurrent.TimeUnit;

public class KickerCommand extends CommandBase {
    private final Kicker kicker;
    private double ms;
    boolean toggled = false;
    private Timing.Timer timer;
    private Kicker.state state;

    public KickerCommand(Kicker kicker, long ms, Kicker.state state) {
        this.kicker = kicker;
        timer = new Timing.Timer(ms, TimeUnit.MILLISECONDS);
        this.state = state;

        addRequirements(kicker);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        kicker.currentState = state;
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }
}