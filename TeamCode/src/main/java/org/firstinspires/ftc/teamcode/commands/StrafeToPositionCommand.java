package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

public class StrafeToPositionCommand extends CommandBase {
    private PathChain action;
    private Pose currentPose;
    private Pose targetPose;
    private boolean isFinished = false;
    private Drivetrain drivetrain;


    public StrafeToPositionCommand(Pose targetPose, Drivetrain drivetrain) {
        addRequirements(drivetrain);
        this.targetPose = targetPose;
        this.drivetrain = drivetrain;
    }

    @Override
    public void initialize() {
        currentPose = drivetrain.getCurrentPose();

        action = drivetrain.getBuilder()
                .addPath(new BezierLine(new Point(currentPose), new Point(targetPose)))
                .setLinearHeadingInterpolation(currentPose.getHeading(), targetPose.getHeading())
                .build();

        drivetrain.follower.followPath(action,true);
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