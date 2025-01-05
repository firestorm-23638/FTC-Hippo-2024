package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Basket;
import org.firstinspires.ftc.teamcode.subsystems.Depositor;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenClaw;

public class SpecimenClawCommand extends CommandBase {
    private final SpecimenClaw claw;
    boolean open;

    public SpecimenClawCommand(SpecimenClaw claw, boolean open) {
        this.claw = claw;
        this.open = open;

        addRequirements(claw);
    }

    @Override
    public void execute() {
        if (open) {
            claw.open();
        }
        else {
            claw.close();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
