package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.LightIndicator;

public class LightIndicatorCommand extends CommandBase {
    private final LightIndicator indicator;
    private final LightIndicator.state state;

    public LightIndicatorCommand(LightIndicator lightIndicator, LightIndicator.state state) {
        this.indicator = lightIndicator;
        this.state = state;

        addRequirements(indicator);
    }

    @Override
    public void execute() {
        this.indicator.setState(state);
    }
}
