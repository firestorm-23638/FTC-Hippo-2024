package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.function.Supplier;

public class DrivetrainCommand extends CommandBase {
    private final Drivetrain m_drivetrain;

    private final Supplier<Double> forwardSupplier;
    private final Supplier<Double> strafeSupplier;
    private final Supplier<Double> turnSupplier;

    private boolean isFieldCentric = false;

    public DrivetrainCommand(Drivetrain drive, Supplier<Double> forward, Supplier<Double> strafe, Supplier<Double> turn, boolean isFieldCentric) {
        m_drivetrain = drive;
        forwardSupplier = forward;
        strafeSupplier = strafe;
        turnSupplier = turn;
        this.isFieldCentric = isFieldCentric;

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        m_drivetrain.driveFieldCentric(forwardSupplier.get(), -strafeSupplier.get(), -turnSupplier.get(), this.isFieldCentric);
    }
}
