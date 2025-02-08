package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.ArrayList;
import java.util.List;

public class CalculateAndTurnLimelightCommand extends CommandBase {
    public class AngleAmount {
        public double angle;
        public int amount;
        public AngleAmount(double angle) {
            this.angle = angle;
            this.amount = 1;
        }
    }

    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final Telemetry telemetry;

    private final int MAX_TICKS = 10;
    private AngleAmount finalAngle;
    private boolean hasAngle = false;
    private double allAngles[] = new double[MAX_TICKS];
    private List<AngleAmount> uniqueAngles = new ArrayList<>();
    private double goalAngle;
    private double initialAngle;

    private double forSpeed = 0;

    private int ticks = 0;

    public CalculateAndTurnLimelightCommand(Limelight limelight, Drivetrain drivetrain, Telemetry telemetry, double forwardSpeed) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.forSpeed = forwardSpeed;
        this.telemetry = telemetry;

        addRequirements(limelight, drivetrain);

        limelight.toYellowSample();
    }

    public double coterm(double angle) {
        while (angle > 360) {
            angle -= 360;
        }
        while (angle < 0) {
            angle += 360;
        }
        return angle;
    }

    public void smartAdd(double angle) {
        for (AngleAmount a: uniqueAngles) {
            double MAX_ERROR = 0.75;
            if ((angle >= a.angle - MAX_ERROR) && (angle <= a.angle + MAX_ERROR)) {
                a.amount ++;
                return;
            }
        }
        uniqueAngles.add(new AngleAmount(angle));
    }

    public double calculateError(double goalAngle, double currentAngle) {
        if (Math.abs(goalAngle - currentAngle) > 180) {
            currentAngle += 180;
        }
        telemetry.addData("new goal", goalAngle);
        telemetry.addData("new current", currentAngle);
        return (goalAngle - currentAngle);
    }

    @Override
    public void execute() {
        telemetry.addData("Has Angle", hasAngle);
        //telemetry.addData("Final Limelight Angle", finalAngle.angle);
        if (hasAngle) {
            telemetry.addData("Final Limelight Angle", finalAngle.angle);
            double currentAngle = Math.toDegrees(drivetrain.getCurrentPose().heading.log());
            double amtTurned = initialAngle - currentAngle;
            telemetry.addData("Amount turned", amtTurned);
            double kP = 0.03;
            double error = finalAngle.angle - amtTurned;
            telemetry.addData("Error", finalAngle.angle - amtTurned);
            telemetry.addData("Setting to", error*kP);
            drivetrain.driveArcade(forSpeed, 0, error*kP);
        }
        else {
            if (ticks < MAX_TICKS) {
                List<LLResultTypes.ColorResult> results = limelight.lookForSamples();
                if (results != null) {
                    allAngles[ticks] = results.get(0).getTargetXDegrees();
                    ticks ++;
                }
            }
            else {      // now we have all angle samples
                double LIMELIGHT_ANGLE_MULTIPLIER = 1;
                for (double a: allAngles) {
                    smartAdd(a);
                }
                for (AngleAmount u: uniqueAngles) {
                    if (finalAngle != null) {
                        if (u.amount > finalAngle.amount) {
                            finalAngle = u;
                        }
                    }
                    else {
                        finalAngle = u;
                    }
                }
                goalAngle = coterm(Math.toDegrees(drivetrain.getCurrentPose().heading.log()) + (-finalAngle.angle * LIMELIGHT_ANGLE_MULTIPLIER));
                initialAngle = Math.toDegrees(drivetrain.getCurrentPose().heading.log());
                hasAngle = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
