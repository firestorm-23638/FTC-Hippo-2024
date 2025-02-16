package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class PathChainCommand extends CommandBase {
    private PathChain chain;
    private Drivetrain drivetrain;
    private boolean isFinished = false;


    public PathChainCommand(PathChain chain, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.chain = chain;
    }

    @Override
    public void initialize() {
        drivetrain.follower.followPath(chain ,true);
    }

    @Override
    public void execute() {
        isFinished = !drivetrain.follower.isBusy();
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }
}
