package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class SlideUntilHasPieceCommand extends CommandBase {
    private final Intake intake;
    private final Intake.color colorToReceive;
    private Timing.Timer timer;
    public double startTrim = 0;

    public SlideUntilHasPieceCommand(Intake intake, Intake.color colorToReceive) {
        this(intake, colorToReceive, 0);
    }

    public SlideUntilHasPieceCommand(Intake intake, Intake.color colorToReceive, double startingTrim) {
        this.intake = intake;
        this.colorToReceive = colorToReceive;
        this.startTrim = startingTrim;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.hasTheRightColor = false;
        intake.trim = startTrim;
        timer = new Timing.Timer(120, TimeUnit.MILLISECONDS);
        timer.start();
    }

    @Override
    public void execute() {
        intake.horizontalOut();
        intake.pivotDown();
        intake.setVacuumRun();
        if (intake.getPivotPos() > (Constants.intakePivotToDown - 5)) {
            if (timer.done()) {
                intake.trim += 5;
                timer = new Timing.Timer(120, TimeUnit.MILLISECONDS);
                timer.start();
            }
        }
    }

    @Override
    public boolean isFinished() {
        return (intake.trim == 100) || ((intake.getCurrentColor() == colorToReceive) || intake.getCurrentColor() == Intake.color.YELLOW) || (intake.hasTheRightColor);
    }
}
