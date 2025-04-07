package commands;

import com.arcrobotics.ftclib.command.CommandBase;

import subsystems.Depositor;

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
        if (state == Depositor.state.VERTICAL_TRANSITION) {
            depositor.toVerticalTransition();
            depositor.clawOpen();
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
            depositor.toSpecimenPickup();
        }
        else if (state == Depositor.state.CLAWTIGHTEN) {
            depositor.clawTighten();
        }
        else if (state == Depositor.state.PRIME1) {
            depositor.toPrime1();
        }
        else if (state == Depositor.state.PRIME_SPECIMEN) {
            depositor.toPrimeSpecimens();
        }
        else if (state == Depositor.state.PLACE_SPECIMEN) {
            depositor.toPlaceSpecimen();
        }
        else if (state == Depositor.state.SCORE_SPECIMEN) {
            depositor.toScoreSpecimen();
        }
    }
}