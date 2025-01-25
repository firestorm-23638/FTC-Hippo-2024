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
        if (state == Depositor.state.TRANSITIONING) {
            depositor.toTransition();
        }
        else if (state == Depositor.state.HOME){
            depositor.toHome();
        }
        else if (state == Depositor.state.BUCKET) {
            depositor.toBasket();
        }
        else if (state == Depositor.state.CLAWOPEN) {
            depositor.clawOpen();
        }
        else if (state == Depositor.state.CLAWCLOSE) {
            depositor.clawClose();
        }
        else if (state == Depositor.state.SPECIMEN) {
            depositor.toSpecimen();
        }
        else if (state == Depositor.state.CLAWTIGHTEN) {
            depositor.clawTighten();
        }
    }
}