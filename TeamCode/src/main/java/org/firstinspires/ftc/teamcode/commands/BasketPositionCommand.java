package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Basket;

public class BasketPositionCommand extends CommandBase {
    private final Basket basket;
    private Basket.state state;

    public BasketPositionCommand(Basket basket, Basket.state state) {
        addRequirements(basket);
        this.basket = basket;
        this.state = state;
    }

    @Override
    public void execute() {
        if (state == Basket.state.BUCKET) {
            basket.toDeposit();
        }
        else {
            basket.toHome();
        }
    }
}
