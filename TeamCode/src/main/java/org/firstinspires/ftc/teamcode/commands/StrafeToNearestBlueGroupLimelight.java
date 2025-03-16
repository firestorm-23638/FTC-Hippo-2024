package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.ArrayList;
import java.util.List;

public class StrafeToNearestBlueGroupLimelight extends CommandBase {
    public class PixelAmount {
        public double pixel;
        public int amount;
        public PixelAmount(double angle) {
            this.pixel = angle;
            this.amount = 1;
        }
    }

    private final Limelight limelight;
    private final Drivetrain drivetrain;
    private final Telemetry telemetry;
    private final double resX = 640;
    private final double pixelsToInches = 0.025;

    private final int MAX_TICKS = 10;
    private PixelAmount finalPixel;
    private boolean hasPixel = false;
    private double allPixels[] = new double[MAX_TICKS];
    private List<PixelAmount> uniquePixels = new ArrayList<>();
    private double goalInches;
    private double initialPose;

    private double forSpeed = 0;

    private int ticks = 0;

    public StrafeToNearestBlueGroupLimelight(Limelight limelight, Drivetrain drivetrain, Telemetry telemetry, double forwardSpeed) {
        this.limelight = limelight;
        this.drivetrain = drivetrain;
        this.forSpeed = forwardSpeed;
        this.telemetry = telemetry;

        addRequirements(limelight, drivetrain);

        limelight.toYellowAndBlue();
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

    public void smartAdd(double pixel) {
        for (PixelAmount a: uniquePixels) {
            double MAX_ERROR = 25;
            if ((pixel >= a.pixel - MAX_ERROR) && (pixel <= a.pixel + MAX_ERROR)) {
                a.amount ++;
                return;
            }
        }
        uniquePixels.add(new PixelAmount(pixel));
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
        telemetry.addData("Has Pixel", hasPixel);
        telemetry.addData("Ticks", ticks);
        //telemetry.addData("Final Limelight Angle", finalAngle.angle);
        if (hasPixel) {
            telemetry.addData("Final Pixel", finalPixel.pixel);
            telemetry.addData("Final Inches", goalInches);
            telemetry.addData("Robot X", drivetrain.getCurrentPose().getX());
            double currentPos = drivetrain.getCurrentPose().getX();
            double amtTraveled = initialPose - currentPos;
            telemetry.addData("Amount Traveled", amtTraveled);
            double kP = 0.1;
            double error = finalPixel.pixel - amtTraveled;
            telemetry.addData("Error", goalInches - amtTraveled);
            telemetry.addData("Setting to", error*kP);
            //drivetrain.driveArcade(0, error*kP, 0);
        }
        else {
            if (ticks < MAX_TICKS) {
                double[] results = limelight.lookForSamples();
                telemetry.addData("Results", results == null);
                if (results != null) {
                    allPixels[ticks] = results[1];
                    ticks ++;
                }
            }
            else {      // now we have all angle samples
                double LIMELIGHT_ANGLE_MULTIPLIER = 1;
                for (double a: allPixels) {
                    smartAdd(a);
                }
                for (PixelAmount u: uniquePixels) {
                    if (finalPixel != null) {
                        if (u.amount > finalPixel.amount) {
                            finalPixel = u;
                        }
                    }
                    else {
                        finalPixel = u;
                    }
                }
                goalInches = finalPixel.pixel * pixelsToInches;
                initialPose = drivetrain.getCurrentPose().getX();
                hasPixel = true;
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
