package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Climb;

public class ClimbCommand extends CommandBase {
    private final Climb climb;
    private double leftAmt;
    private double rightAmt;

    public ClimbCommand(Climb climb, double leftAmt, double rightAmt) {
        this.climb = climb;
        this.leftAmt = leftAmt;
        this.rightAmt = rightAmt;
    }

    @Override
    public void execute() {
        climb.setState(state);
    }
}
