package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.LightIndicator;

import java.util.List;

public class LightIndicatorListCommand extends SequentialCommandGroup {
    private final LightIndicator indicator;
    private final List<LightIndicator.LightAction> list;

    public LightIndicatorListCommand(LightIndicator indicator, List<LightIndicator.LightAction> list) {
        this.list = list;
        this.indicator = indicator;

        addRequirements(indicator);

        SequentialCommandGroup buffer = new SequentialCommandGroup();

        for (LightIndicator.LightAction action : list) {
            buffer.addCommands(new LightIndicatorCommand(indicator, action.state));
            buffer.addCommands(new WaitCommand(action.millis));
        }

        this.addCommands(buffer);

    }
}