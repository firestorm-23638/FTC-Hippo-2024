package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LightIndicator;

public class LightIndicatorCommand extends CommandBase {
    private final LightIndicator indicator;
    private final LightIndicator.ColorState state;

    public LightIndicatorCommand(LightIndicator lightIndicator, LightIndicator.ColorState state) {
        this.indicator = lightIndicator;
        this.state = state;

        addRequirements(indicator);
    }

    @Override
    public void execute() {
        this.indicator.setState(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
