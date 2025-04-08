package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Depositor;


public class DepositorCommand extends CommandBase {
    private final Depositor depositor;
    private Depositor.state state;

    public DepositorCommand(Depositor depositor, Depositor.state state) {
        addRequirements(depositor);
        this.depositor = depositor;
        this.state = state;
    }

    @Override
    public void execute() {
        this.depositor.toPosition(state);
    }
}