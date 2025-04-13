package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climb;

import java.util.function.Supplier;

public class ClimbCommand extends CommandBase {
    private final Climb climb;
    private Supplier<Double> leftAmt;
    private Supplier<Double> rightAmt;

    public ClimbCommand(Climb climb, Supplier<Double> leftAmt, Supplier<Double> rightAmt) {
        this.climb = climb;
        this.leftAmt = leftAmt;
        this.rightAmt = rightAmt;

        addRequirements(climb);
    }

    @Override
    public void execute() {
        climb.setLeftMotor(leftAmt.get());
        climb.setRightMotor(rightAmt.get());
    }
}
